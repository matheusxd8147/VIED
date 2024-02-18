/*
 *  server_example_goose.c
 *
 *  This example demonstrates how to use GOOSE publishing, Reporting and the
 *  control model.
 *
 */

#include "goose_receiver.h"
#include "goose_subscriber.h"
#include "iec61850_server.h"
#include "hal_thread.h" /* for Thread_sleep() */
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "mms_value.h"
#include "goose_publisher.h"
#include "sv_subscriber.h"
#include <math.h>
#include "static_model.h"


 /* import IEC 61850 device model created from SCL-File */
extern IedModel iedModel;

int contador = 0; //variável para auxiliar na função de abertura do disjuntor
char* svID2;
static int running = 0;
static IedServer iedServer = NULL;
int contadorSV1 = 0;
int contadorSV2 = 0;
float SVrms_deltaA = 0;
float SVrms_deltaB = 0;
float SVrms_deltaC = 0;
static float corrente_primarioA = 0;
static float r_transformacao = 2.8868;
float corrente_teste = 100;

void sigint_handler(int signalId)
{
    running = 0;
}

/* Callback handler for received SV messages */
static void
svUpdateListener (SVSubscriber subscriber, void* parameter, SVSubscriber_ASDU asdu)
{
    const char* svID = SVSubscriber_ASDU_getSvId(asdu);    
    
    if ((strcmp(svID,"svpub1"))== 0){       
        SVrms_deltaA = (SVrms_deltaA + pow(SVSubscriber_ASDU_getFLOAT32(asdu, 0),2));
        SVrms_deltaB = (SVrms_deltaB + pow(SVSubscriber_ASDU_getFLOAT32(asdu, 4),2));
        SVrms_deltaC = (SVrms_deltaC + pow(SVSubscriber_ASDU_getFLOAT32(asdu, 8),2));
        contadorSV1 += 1;
    } 

    if (contadorSV1==80)
    {
        //system ("clear");
        //printf("  svID=(%s)\n", svID);
        //printf("  smpCnt: %i\n", SVSubscriber_ASDU_getSmpCnt(asdu));
        //printf("  confRev: %u\n", SVSubscriber_ASDU_getConfRev(asdu));
        corrente_primarioA = sqrt(SVrms_deltaA / 80);
        corrente_primarioB = sqrt(SVrms_deltaB / 80);
        corrente_primarioC = sqrt(SVrms_deltaC / 80);
        //printf("   A corrente RMS da fase A no primário é: %f [A]\n", corrente_primarioA );
        //printf("   A corrente RMS da fase B no primário é: %f [A]\n", corrente_primarioB );
        //printf("   A corrente RMS da fase C no primário é: %f [A]\n", corrente_primarioC );
        contadorSV1=0;
        SVrms_deltaA=0;
        SVrms_deltaB=0;
        SVrms_deltaC=0;
    }
}

static void
goCbEventHandler(MmsGooseControlBlock goCb, int event, void* parameter)
{
    printf("Access to GoCB: %s\n", MmsGooseControlBlock_getName(goCb));
    printf("         GoEna: %i\n", MmsGooseControlBlock_getGoEna(goCb));
}

//Função Listener

static void
gooseListener(GooseSubscriber subscriber, void* parameter)
{
    MmsValue* values = GooseSubscriber_getDataSetValues(subscriber);

    char buffer[50];

    MmsValue_printToBuffer(values, buffer, 50);

    if (corrente_primarioA>80)
    {
        printf("Corrente de curto-circuito.\nProteção Instantânea (51).\nAssinante Não Atuar.\nFalta Eliminada\n");
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_P1TPIOC1_Str_general, true);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true); //nó lógico para envio da mensagem
        //IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BFR1RBRF1_OpEx_general, true);
        Thread_sleep(50);
        if (corrente_primarioA>=1150)
        {
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BFR1RBRF1_OpEx_general, true);
        }
        exit(0);
    }else
    {
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_P1TPIOC1_Str_general, false);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, false);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BFR1RBRF1_OpEx_general, false);
    }
}

int
main(int argc, char** argv)
{
    IedServerConfig config = IedServerConfig_create();

    iedServer = IedServer_createWithConfig(&iedModel, NULL, config);

    IedServerConfig_destroy(config);
    
    SVReceiver receiverSV = SVReceiver_create();

    if (argc > 1) {
        char* ethernetIfcID = argv[1];

        printf("Using GOOSE interface: %s\n", ethernetIfcID);

        /* set GOOSE interface for all GOOSE publishers (GCBs) */
        IedServer_setGooseInterfaceId(iedServer, ethernetIfcID);
        SVReceiver_setInterfaceId(receiverSV, ethernetIfcID);
    }

    if (argc > 2) {
        char* ethernetIfcID = argv[2];

        printf("Using GOOSE interface for GenericIO/LLN0.gcbAnalogValues: %s\n", ethernetIfcID);

        /* set GOOSE interface for a particular GOOSE publisher (GCB) */
        IedServer_setGooseInterfaceIdEx(iedServer, IEDMODEL_CFG_LLN0, "BRep0201", ethernetIfcID);
    }

    /*Preparando o código para receber mensagens SV*/

    SVSubscriber subscriberSV = SVSubscriber_create(NULL, 0x4000);
    SVSubscriber_setListener(subscriberSV, svUpdateListener, NULL);
    SVReceiver_addSubscriber(receiverSV, subscriberSV);

    /*Preparando o código para publicar mensagens GOOSE*/

    IedServer_enableGoosePublishing(iedServer);
    GooseReceiver receiver = GooseReceiver_create();
    GooseReceiver_setInterfaceId(receiver, "lo");
    GooseSubscriber subscriber = GooseSubscriber_create("SEL_751_1CFG/LLN0$GO$GOOSE_SL_1", NULL); //Especificação de quem o ied irá receber as mensagens goose
    GooseSubscriber_setListener(subscriber, gooseListener, iedServer);
    GooseReceiver_addSubscriber(receiver, subscriber);

    /*Começa a receber mensagens SV e GOOSE*/

    GooseReceiver_start(receiver);
    SVReceiver_start(receiverSV);


    //Thread_sleep(1000);

    IedServer_setGoCBHandler(iedServer, goCbEventHandler, NULL);

    /* MMS server will be instructed to start listening to client connections. */
    IedServer_start(iedServer, 102);
    /*
        IedServer_setControlHandler(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01, (ControlHandler) controlHandlerForBinaryOutput,
        IEDMODEL_CON_RBGGIO1_SPCSO01);

        IedServer_setControlHandler(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02, (ControlHandler) controlHandlerForBinaryOutput,
        IEDMODEL_CON_RBGGIO1_SPCSO02);

        IedServer_setControlHandler(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO03, (ControlHandler) controlHandlerForBinaryOutput,
        IEDMODEL_CON_RBGGIO1_SPCSO03);

        IedServer_setControlHandler(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO04, (ControlHandler) controlHandlerForBinaryOutput,
        IEDMODEL_CON_RBGGIO1_SPCSO04);
    */
    if (!IedServer_isRunning(iedServer)) {
        printf("Starting server failed! Exit.\n");
        IedServer_destroy(iedServer);
        exit(-1);
    }

    running = 1;

    signal(SIGINT, sigint_handler);

    while (running) {

        Thread_sleep(0.20833);
    }

    /* stop MMS server - close TCP server socket and all client sockets */
    IedServer_stop(iedServer);

    /* Cleanup - free all resources */
    SVReceiver_destroy(receiverSV);
    IedServer_destroy(iedServer);

    return 0;
} /* main() */
