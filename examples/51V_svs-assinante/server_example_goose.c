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
#include <sys/timeb.h>
#include <sys/time.h>
#define MICRO_PER_SECOND 1000000
#define pi 3.14

 /* import IEC 61850 device model created from SCL-File */
extern IedModel iedModel;

bool contador = true;
bool contador1 = true;
bool contador2 = true;
bool contador3 = true;
bool contador4 = true;
bool contador5 = true;
bool contador6 = true;
bool resposta = true;
int curva = 0;
int funcao = 0;

char* svID2;
static int running = 0;
static IedServer iedServer = NULL;
int contadorSV1 = 0;
int contadorSV2 = 0;
int contadorSV3 = 0;
float Vn = 0, j = 0;
float SVrms_deltaA = 0;
float SVrms_deltaB = 0;
float SVrms_deltaC = 0;
float SVrms_deltaA1 = 0;
float SVrms_deltaB1 = 0;
float SVrms_deltaC1 = 0;
float max_corrente_a = 0;
float max_corrente_b = 0;
float max_corrente_c = 0;
float max_tensao_a = 0;
float max_tensao_b = 0;
float max_tensao_c = 0;
static float corrente_primarioA = 0;
static float corrente_primarioB = 0;
static float corrente_primarioC = 0;
static float tensao_primarioA = 0;
static float tensao_primarioB = 0;
static float tensao_primarioC = 0;
float teste[80];
static float pick_up;
static float M, K, a, t,T, B = 1;
float tempo_inicial = 0;
struct timeval start_time;
struct timeval stop_time;
struct timeval stop1_time;
static float ang1, ang2, ang3, ang4, ang5, ang6;
static float time_diff = 0;
float an[6], teta[6];

 

void sigint_handler(int signalId)
{
    running = 0;
}

/* Callback handler for received SV messages */
static void
svUpdateListener (SVSubscriber subscriber, void* parameter, SVSubscriber_ASDU asdu)
{   
    int i;
    const char* svID = SVSubscriber_ASDU_getSvId(asdu);

    if ((strcmp(svID,"IEDNameMU0101"))== 0){      
        //printf("teste 1: %f\n", SVSubscriber_ASDU_getINT32 (asdu,0)*0.001);
        //printf("teste 1: %d\n", SVSubscriber_ASDU_getDataSize(asdu));
        SVrms_deltaA = (SVrms_deltaA + pow((SVSubscriber_ASDU_getINT32 (asdu, 0)*0.001),2));
        SVrms_deltaB = (SVrms_deltaB + pow((SVSubscriber_ASDU_getINT32 (asdu, 8)*0.001),2));
        SVrms_deltaC = (SVrms_deltaC + pow((SVSubscriber_ASDU_getINT32 (asdu, 16)*0.001),2));
        contadorSV1 += 1;
    }    

    if (contadorSV1==80)
    {   
        corrente_primarioA = sqrt(SVrms_deltaA / 80);
        corrente_primarioB = sqrt(SVrms_deltaB / 80);
        corrente_primarioC = sqrt(SVrms_deltaC / 80);
        

        /*system ("clear");
        printf("  svID=(%s)\n", svID);
        printf("  smpCnt: %i\n", SVSubscriber_ASDU_getSmpCnt(asdu));
        printf("-----------------------------------------------------\n");  
        printf("   A tensão RMS da fase A no primário é: %.2f [V] %.1f°\n", tensao_primarioA, an[0] );
        printf("   A tensão RMS da fase B no primário é: %.2f [V] %.2f°\n", tensao_primarioB, an[1] );
        printf("   A tensão RMS da fase C no primário é: %.2f [V] %.1f°\n", tensao_primarioC, an[2] );
        printf("-----------------------------------------------------\n");
        printf("   A corrente RMS da fase A no primário é: %.2f [A] %.1f°\n", corrente_primarioA, an[3] );
        printf("   A corrente RMS da fase B no primário é: %.2f [A] %.1f°\n", corrente_primarioB, an[4] );
        printf("   A corrente RMS da fase C no primário é: %.2f [A] %.1f°\n", corrente_primarioC, an[5] );
        printf("-----------------------------------------------------\n");
*/
        //printf("\n%f\n",t);

        /*contadorSV1=0;
        SVrms_deltaA=0;
        SVrms_deltaB=0;
        SVrms_deltaC=0;
        SVrms_deltaA1=0;
        SVrms_deltaB1=0;
        SVrms_deltaC1=0;
        contador1 = true;
        contador2 = true;
        contador3 = true;
        contador4 = true;
        contador5 = true;
        contador6 = true;*/
        contadorSV3++;

        if(contadorSV3 == 60){
            
            FILE *file;
            file = fopen("VIEDeficaz.txt","w");
            fprintf(file,"%f\n",tensao_primarioA);
            fprintf(file,"%2.f\n",an[0]);
            fprintf(file,"%2.f\n",tensao_primarioB);
            fprintf(file,"%2.f\n",an[1]);
            fprintf(file,"%2.f\n",tensao_primarioC);
            fprintf(file,"%2.f\n",an[2]);
            fprintf(file,"%2.f\n",corrente_primarioA);
            fprintf(file,"%2.f\n",an[3]);
            fprintf(file,"%2.f\n",corrente_primarioB);
            fprintf(file,"%2.f\n",an[4]);
            fprintf(file,"%2.f\n",corrente_primarioC);
            fprintf(file,"%2.f\n",an[5]);
            fclose(file);
            
             
            system ("clear");
            printf("  svID=(%s)\n", svID);
            printf("  smpCnt: %i\n", SVSubscriber_ASDU_getSmpCnt(asdu));
            printf("-----------------------------------------------------\n");
            printf("   A corrente RMS da fase A no primário é: %.2f [A]\n", corrente_primarioA);
            printf("   A corrente RMS da fase B no primário é: %.2f [A]\n", corrente_primarioB);
            printf("   A corrente RMS da fase C no primário é: %.2f [A]\n", corrente_primarioC);
            printf("-----------------------------------------------------\n");
            contadorSV3 = 0;
        }

        contadorSV1=0;
        SVrms_deltaA=0;
        SVrms_deltaB=0;
        SVrms_deltaC=0;
        SVrms_deltaA1=0;
        SVrms_deltaB1=0;
        SVrms_deltaC1=0;
        contador1 = true;
        contador2 = true;
        contador3 = true;
        contador4 = true;
        contador5 = true;
        contador6 = true;

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

    /***********seletividade assinante*************/

    char b; char c; char d;

    b = buffer[1];
    c = buffer[6];
    d = buffer[11];

    //if((b!=116)&&(c!=116)&&(d!=116))
    //{
     //printf("Sem evento \n");
     //}


    if(corrente_primarioA>=600){
        /*if((b != 116) && (c != 116) && (d != 116)){
            printf("ATUAR - FUNÇÃO 50");
            IedServer_updateDbposValue(iedServer, IEDMODEL_PRO_BK1XCBR1_Pos_stVal, DBPOS_OFF);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
            //exit(0);
        }*/

        if((b = 116) && (c = 116) && (d != 116)){
            printf("CORRENTE DE FALTA - NÃO ATUAR SELETIVIDADE LÓGICA\n");
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, false);
            //exit(0);
        }

        if((b = 116) && (c = 116) && (d = 116)){
            printf("FALHA DE DISJUNTOR \n");
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
            printf("SINAL DE TRIP ENVIADO AO DISJUNTOR \n");
            printf("DISJUNTOR ABERTO \n");
            IedServer_updateDbposValue(iedServer, IEDMODEL_PRO_BK1XCBR1_Pos_stVal, DBPOS_OFF);
            exit(0);
        }
    }else{
        IedServer_updateDbposValue(iedServer, IEDMODEL_PRO_BK1XCBR1_Pos_stVal, DBPOS_ON);
    }

    
    /*
    if ((b != 116) && (c != 116) && (d != 116) && (contador == 0)) {
        IedServer_updateDbposValue(iedServer, IEDMODEL_PRO_BK1XCBR1_Pos_stVal, DBPOS_OFF);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, false);
    }

    if ((b == 116) && (c != 116) && (d != 116)) //116 é o valor da tabela ASCII para T (True)
    {
        printf("CORRENTE DE FALTA \n");
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, false);
    }

    if ((b == 116) && (c == 116) && (d != 116))//116 é o valor da tabela ASCII para T (True)
    {
        printf("NÃO ATUAR - SELETIVIDADE LÓGICA \n");
    }

    if ((b == 116) && (c == 116) && (d == 116))//116 é o valor da tabela ASCII para T (True)
    {
        contador = 1;
        printf("FALHA DE DISJUNTOR \n");
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
        printf("SINAL DE TRIP ENVIADO AO DISJUNTOR \n");
        printf("DISJUNTOR ABERTO \n");
        IedServer_updateDbposValue(iedServer, IEDMODEL_PRO_BK1XCBR1_Pos_stVal, DBPOS_OFF);
    }
    if (contador == 1) {
        IedServer_updateDbposValue(iedServer, IEDMODEL_PRO_BK1XCBR1_Pos_stVal, DBPOS_OFF);
    }*/
    

}

int
main(int argc, char** argv)
{   //while (i<10){

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
        IedServer_start(iedServer, 103);
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

            Thread_sleep(0.01);

        }

        /* stop MMS server - close TCP server socket and all client sockets */
        IedServer_stop(iedServer);

        /* Cleanup - free all resources */
        SVReceiver_destroy(receiverSV);
        IedServer_destroy(iedServer);

        return 0;

    //}
} /* main() */
