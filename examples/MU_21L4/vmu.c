/*
 *  iec61850_9_2_LE_example.c
 *
 *  Copyright 2016 Michael Zillgith
 *
 *  This file is part of libIEC61850.
 *
 *  libIEC61850 is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  libIEC61850 is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with libIEC61850.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  See COPYING file for the complete license text.
 */

#include "goose_receiver.h"
#include "goose_subscriber.h"
#include "iec61850_server.h"
#include "sv_publisher.h"
#include "hal_thread.h"
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

#define _USE_MATH_DEFINES
#include <math.h>

/* Include the generated header with the model access handles */
#include "static_model.h"

/* import IEC 61850 device model created from SCL-File */
extern IedModel iedModel;
static IedServer iedServer = NULL;

static int running = 0;
static int svcbEnabled = 1;

void sigint_handler(int signalId)
{
    running = 0;
}

static int amp1;
static int amp2;
static int amp3;
static int amp4;

static int amp1q;
static int amp2q;
static int amp3q;
static int amp4q;

static int vol1;
static int vol2;
static int vol3;
static int vol4;

static int vol1q;
static int vol2q;
static int vol3q;
static int vol4q;

static SVPublisher svPublisher;
static SVPublisher_ASDU asdu;

static float vol10,vol20,vol30;
static float amp10,amp20,amp30;
static float an[6];
static float phaseAngle = 0.f;

static void
setupSVPublisher(const char* svInterface)
{
    svPublisher = SVPublisher_create(NULL, svInterface);

    if (svPublisher) {

        asdu = SVPublisher_addASDU(svPublisher, "VMU04", NULL, 1);

        amp1 = SVPublisher_ASDU_addINT32(asdu);
        amp1q = SVPublisher_ASDU_addQuality(asdu);
        amp2 = SVPublisher_ASDU_addINT32(asdu);
        amp2q = SVPublisher_ASDU_addQuality(asdu);
        amp3 = SVPublisher_ASDU_addINT32(asdu);
        amp3q = SVPublisher_ASDU_addQuality(asdu);
        amp4 = SVPublisher_ASDU_addINT32(asdu);
        amp4q = SVPublisher_ASDU_addQuality(asdu);

        vol1 = SVPublisher_ASDU_addINT32(asdu);
        vol1q = SVPublisher_ASDU_addQuality(asdu);
        vol2 = SVPublisher_ASDU_addINT32(asdu);
        vol2q = SVPublisher_ASDU_addQuality(asdu);
        vol3 = SVPublisher_ASDU_addINT32(asdu);
        vol3q = SVPublisher_ASDU_addQuality(asdu);
        vol4 = SVPublisher_ASDU_addINT32(asdu);
        vol4q = SVPublisher_ASDU_addQuality(asdu);

        SVPublisher_ASDU_setSmpCntWrap(asdu, 4800);
        SVPublisher_ASDU_setRefrTm(asdu, 0);

        SVPublisher_setupComplete(svPublisher);
    }
}

static void sVCBEventHandler (SVControlBlock* svcb, int event, void* parameter)
{
    if (event == IEC61850_SVCB_EVENT_ENABLE)
        svcbEnabled = 1;
    else if (event == IEC61850_SVCB_EVENT_DISABLE)
        svcbEnabled = 0;
}

static void
goCbEventHandler(MmsGooseControlBlock goCb, int event, void* parameter)
{
    printf("Access to GoCB: %s\n", MmsGooseControlBlock_getName(goCb));
    printf("         GoEna: %i\n", MmsGooseControlBlock_getGoEna(goCb));
}

static void
gooseListener(GooseSubscriber subscriber, void* parameter)
{
    MmsValue* values = GooseSubscriber_getDataSetValues(subscriber);

    char buffer[50];

    MmsValue_printToBuffer(values, buffer, 50);


    char b; char c; char d;

    b = buffer[1];
    c = buffer[7];
    d = buffer[11];
    uint64_t y = Hal_getTimeInMs();

    if(b == 116){
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_BinIO_BinaryInputs_LPDI3_In_stVal, true);
        FILE *file;
        file = fopen("AjustesMU.txt","r");
        fscanf(file,"%f\n",&amp10);
        fscanf(file,"%f\n",&an[0]);
        fscanf(file,"%f\n",&amp20);
        fscanf(file,"%f\n",&an[1]);
        fscanf(file,"%f\n",&amp30);
        fscanf(file,"%f\n",&an[2]);
        fscanf(file,"%f\n",&vol10);
        fscanf(file,"%f\n",&an[3]);
        fscanf(file,"%f\n",&vol20);
        fscanf(file,"%f\n",&an[4]);
        fscanf(file,"%f\n",&vol30);
        fscanf(file,"%f\n",&an[5]);
        fclose(file);
    }
    
    if(c == 116){
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_BinIO_BinaryInputs_LPDI3_In_stVal, false);
        amp10 = 0;
        amp20 = 0;
        amp30 = 0;
        vol10 = 0;
        vol20 = 0;
        vol30 = 0;
        an[0] = 0;
        an[1] = 0;
        an[2] = 0;
        an[3] = 0;
        an[4] = 0;
        an[5] = 0;
    }
    /*
    printf("-------------------------------------------------------------------------------------------------------------\n");            
    printf("                               PRIMEIRA MENSAGEM GOOSE ASSINADA VIED 1                                       \n");
    printf("-------------------------------------------------------------------------------------------------------------\n");
    */

}

int 
main(int argc, char** argv)
{
    char* svInterface;

    IedServerConfig config = IedServerConfig_create();

    iedServer = IedServer_createWithConfig(&iedModel, NULL, config);

    IedServerConfig_destroy(config);
    
    //IedServer iedServer = IedServer_create(&iedModel);

    if (argc > 1){
        svInterface = argv[1];
        char* ethernetIfcID = argv[1];
        printf("Using GOOSE interface: %s\n", ethernetIfcID);
        IedServer_setGooseInterfaceId(iedServer, ethernetIfcID);
    }
    
    if (argc > 2){
        svInterface = "eth0";
        char* ethernetIfcID = argv[2];
        printf("Using GOOSE interface for GenericIO/LLN0.gcbAnalogValues: %s\n", ethernetIfcID);
        //IedServer_setGooseInterfaceIdEx(iedServer, IEDMODEL_CFG_LLN0, "BRep0201", ethernetIfcID);
    }

    IedServer_enableGoosePublishing(iedServer);
    GooseReceiver receiver = GooseReceiver_create();
    GooseReceiver_setInterfaceId(receiver, "eth0");
    GooseSubscriber subscriber = GooseSubscriber_create("VIED_21L4CFG/LLN0$GO$CONTROL_BK", NULL); //Especificação de quem o ied irá receber as mensagens goose
    GooseSubscriber_setListener(subscriber, gooseListener, iedServer);
    GooseReceiver_addSubscriber(receiver, subscriber);
    GooseReceiver_start(receiver);

    IedServer_setGoCBHandler(iedServer, goCbEventHandler, NULL);

    /* MMS server will be instructed to start listening to client connections. */
    IedServer_start(iedServer, 120);

    if (!IedServer_isRunning(iedServer)) {
        printf("Starting server failed! Exit.\n");
        IedServer_destroy(iedServer);
        exit(-1);
    }

    running = 1;

    signal(SIGINT, sigint_handler);

    setupSVPublisher(svInterface);

    if (svPublisher) {

        IedServer_enableGoosePublishing(iedServer);

        SVControlBlock* svcb = IedModel_getSVControlBlock(&iedModel, IEDMODEL_Mod3_MU2_LLN0, "MSVCB01");

        if (svcb == NULL) {
            printf("Lookup svcb failed!\n");
            exit(1);
        }

        IedServer_setSVCBHandler(iedServer, svcb, sVCBEventHandler, NULL);

        Quality q = QUALITY_VALIDITY_GOOD;

        //int vol = (int) (6350.f * sqrt(2));
        //int amp = (int) (350.f);
        static float phaseAngle = 0.f;
        

        FILE *file;
        file = fopen("AjustesMU.txt","r");
        fscanf(file,"%f\n",&amp10);
        fscanf(file,"%f\n",&an[0]);
        fscanf(file,"%f\n",&amp20);
        fscanf(file,"%f\n",&an[1]);
        fscanf(file,"%f\n",&amp30);
        fscanf(file,"%f\n",&an[2]);
        fscanf(file,"%f\n",&vol10);
        fscanf(file,"%f\n",&an[3]);
        fscanf(file,"%f\n",&vol20);
        fscanf(file,"%f\n",&an[4]);
        fscanf(file,"%f\n",&vol30);
        fscanf(file,"%f\n",&an[5]);
        fclose(file);

        int voltageA;
        int voltageB;
        int voltageC;
        int voltageN;
        int currentA;
        int currentB;
        int currentC;
        int currentN;

        int sampleCount = 0;

        uint64_t nextCycleStart = Hal_getTimeInMs() + 1000;

        while (running) {

            int samplePoint = sampleCount % 80;

            /*double angleA = (2 * M_PI / 80) * samplePoint;
            double angleB = (2 * M_PI / 80) * samplePoint - ( 2 * M_PI / 3);
            double angleC = (2 * M_PI / 80) * samplePoint - ( 4 * M_PI / 3);*/

            double angleA1 = (2 * M_PI / 80) * samplePoint + (an[3] * M_PI / 180);
            double angleB1 = (2 * M_PI / 80) * samplePoint + (an[4] * M_PI / 180);
            double angleC1 = (2 * M_PI / 80) * samplePoint + (an[5] * M_PI / 180);

            double angleA2 = (2 * M_PI / 80) * samplePoint + (an[0] * M_PI / 180);
            double angleB2 = (2 * M_PI / 80) * samplePoint + (an[1] * M_PI / 180);
            double angleC2 = (2 * M_PI / 80) * samplePoint + (an[2] * M_PI / 180);

            voltageA = (vol10 * sqrt(2) * sin(angleA1)) * 100;
            voltageB = (vol20 * sqrt(2) * sin(angleB1)) * 100;
            voltageC = (vol30 * sqrt(2) * sin(angleC1)) * 100;
            voltageN = voltageA + voltageB + voltageC;

            currentA = (amp10 * sqrt(2) * sin(angleA2 - phaseAngle)) * 1000;
            currentB = (amp20 * sqrt(2) * sin(angleB2 - phaseAngle)) * 1000;
            currentC = (amp30 * sqrt(2) * sin(angleC2 - phaseAngle)) * 1000;
            currentN = currentA + currentB + currentC;

            IedServer_lockDataModel(iedServer);

            IedServer_updateInt32AttributeValue(iedServer, IEDMODEL_Mod3_MU2_I01ATCTR1_AmpSv_instMag_i, currentA);
            IedServer_updateQuality(iedServer, IEDMODEL_Mod3_MU2_I01ATCTR1_AmpSv_q, q);
            IedServer_updateInt32AttributeValue(iedServer, IEDMODEL_Mod3_MU2_I01BTCTR2_AmpSv_instMag_i, currentA);
            IedServer_updateQuality(iedServer, IEDMODEL_Mod3_MU2_I01BTCTR2_AmpSv_q, q);
            IedServer_updateInt32AttributeValue(iedServer, IEDMODEL_Mod3_MU2_I01CTCTR3_AmpSv_instMag_i, currentA);
            IedServer_updateQuality(iedServer, IEDMODEL_Mod3_MU2_I01CTCTR3_AmpSv_q, q);
            IedServer_updateInt32AttributeValue(iedServer, IEDMODEL_Mod3_MU2_I01NTCTR4_AmpSv_instMag_i, currentA);
            IedServer_updateQuality(iedServer, IEDMODEL_Mod3_MU2_I01NTCTR4_AmpSv_q, q);

            IedServer_updateInt32AttributeValue(iedServer, IEDMODEL_Mod3_MU2_U01ATVTR1_VolSv_instMag_i, voltageA);
            IedServer_updateQuality(iedServer, IEDMODEL_Mod3_MU2_U01ATVTR1_VolSv_q, q);
            IedServer_updateInt32AttributeValue(iedServer, IEDMODEL_Mod3_MU2_U01BTVTR2_VolSv_instMag_i, voltageB);
            IedServer_updateQuality(iedServer, IEDMODEL_Mod3_MU2_U01BTVTR2_VolSv_q, q);
            IedServer_updateInt32AttributeValue(iedServer, IEDMODEL_Mod3_MU2_U01CTVTR3_VolSv_instMag_i, voltageC);
            IedServer_updateQuality(iedServer, IEDMODEL_Mod3_MU2_U01CTVTR3_VolSv_q, q);
            IedServer_updateInt32AttributeValue(iedServer, IEDMODEL_Mod3_MU2_U01NTVTR4_VolSv_instMag_i, voltageN);
            IedServer_updateQuality(iedServer, IEDMODEL_Mod3_MU2_U01NTVTR4_VolSv_q, q);

            IedServer_unlockDataModel(iedServer);

            if (svcbEnabled) {

                SVPublisher_ASDU_setINT32(asdu, amp1, currentA);
                SVPublisher_ASDU_setQuality(asdu, amp1q, q);
                SVPublisher_ASDU_setINT32(asdu, amp2, currentB);
                SVPublisher_ASDU_setQuality(asdu, amp2q, q);
                SVPublisher_ASDU_setINT32(asdu, amp3, currentC);
                SVPublisher_ASDU_setQuality(asdu, amp3q, q);
                SVPublisher_ASDU_setINT32(asdu, amp4, currentN);
                SVPublisher_ASDU_setQuality(asdu, amp4q, q);

                SVPublisher_ASDU_setINT32(asdu, vol1, voltageA);
                SVPublisher_ASDU_setQuality(asdu, vol1q, q);
                SVPublisher_ASDU_setINT32(asdu, vol2, voltageB);
                SVPublisher_ASDU_setQuality(asdu, vol2q, q);
                SVPublisher_ASDU_setINT32(asdu, vol3, voltageC);
                SVPublisher_ASDU_setQuality(asdu, vol3q, q);
                SVPublisher_ASDU_setINT32(asdu, vol4, voltageN);
                SVPublisher_ASDU_setQuality(asdu, vol4q, q);

                SVPublisher_ASDU_setRefrTmNs(asdu, Hal_getTimeInNs());

                SVPublisher_ASDU_setSmpCnt(asdu, (uint16_t) sampleCount);

                SVPublisher_publish(svPublisher);
            }

            sampleCount = ((sampleCount + 1) % 4800);

            if ((sampleCount % 480) == 0) {
                uint64_t timeval = Hal_getTimeInMs();

                while (timeval < nextCycleStart + 100) {
                    Thread_sleep(1);

                    timeval = Hal_getTimeInMs();
                }

                nextCycleStart = nextCycleStart + 100;
            }
        }
    }
    else {
        printf("Cannot start SV publisher!\n");
    }

    /* stop MMS server - close TCP server socket and all client sockets */
    IedServer_stop(iedServer);

    /* Cleanup - free all resources */
    SVPublisher_destroy(svPublisher);
    IedServer_destroy(iedServer);

    return 0;
} /* main() */
