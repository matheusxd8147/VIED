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
    if(resposta == true){
        system ("clear");
        printf("Escolha uma das funções abaixo:\n- Função 51 [1]\n- Função 51V [2]\n");
        scanf("%d", &funcao);
        if (funcao == 2){
            printf("Defina a Tensão Nominal:\n");
            scanf("%f", &Vn);
        }
        printf("Escolha o percentual da curva\n");
        scanf("%f", &T);
        printf("Defina a corrente de ajuste:\n");
        scanf("%f", &pick_up);
        printf("Escolha uma das curvas abaixo:\n- Curva Inversa [1]\n- Moderadamente Inversa [2]\n- Muito Inversa [3]\n- Extremamente Inversa [4]\n");
        scanf("%d", &curva);
        if (curva == 1){
            K = 0.14;
            a = 0.02;
        }
        if (curva == 2){
            K = 0.05;
            a = 0.04;
        }
        if (curva == 3){
            K = 13.5;
            a = 1;
        }
        if (curva == 4){
            K = 80;
            a = 2;
        }
        resposta = false;
    }

    if ((strcmp(svID,"IEDNameMU0101"))== 0){      
        //printf("teste 1: %f\n", SVSubscriber_ASDU_getINT32 (asdu,0)*0.001);
        //printf("teste 1: %d\n", SVSubscriber_ASDU_getDataSize(asdu));
        SVrms_deltaA = (SVrms_deltaA + pow((SVSubscriber_ASDU_getINT32 (asdu, 0)*0.001),2));
        SVrms_deltaA1 = (SVrms_deltaA1 + pow((SVSubscriber_ASDU_getINT32 (asdu, 32)*0.01),2));
        if((SVSubscriber_ASDU_getINT32 (asdu, 32)*0.01)>=0){
            teste[contadorSV1] = (SVSubscriber_ASDU_getINT32 (asdu, 32)*0.01);
        }
        SVrms_deltaB = (SVrms_deltaB + pow((SVSubscriber_ASDU_getINT32 (asdu, 8)*0.001),2));
        SVrms_deltaB1 = (SVrms_deltaB1 + pow((SVSubscriber_ASDU_getINT32 (asdu, 40)*0.01),2));
        SVrms_deltaC = (SVrms_deltaC + pow((SVSubscriber_ASDU_getINT32 (asdu, 16)*0.001),2));
        SVrms_deltaC1 = (SVrms_deltaC1 + pow((SVSubscriber_ASDU_getINT32 (asdu, 48)*0.01),2));

        if ((contador1 == true)&&((SVSubscriber_ASDU_getINT32 (asdu, 32)*0.01)>=0)){
            max_tensao_a = (SVSubscriber_ASDU_getINT32 (asdu, 32)*0.01);
            contador1 = false;
        }
        if((contador2 == true)&&((SVSubscriber_ASDU_getINT32 (asdu, 40)*0.01)>=0)){
            max_tensao_b = (SVSubscriber_ASDU_getINT32 (asdu, 40)*0.01);
            contador2 = false;
        }

        if((contador3 == true)&&((SVSubscriber_ASDU_getINT32 (asdu, 48)*0.01)>=0)){
            max_tensao_c = (SVSubscriber_ASDU_getINT32 (asdu, 48)*0.01);
            contador3 = false;
        }
        if ((contador4 == true)&&((SVSubscriber_ASDU_getINT32 (asdu, 0)*0.01)>=0)){
            max_corrente_a = (SVSubscriber_ASDU_getINT32 (asdu, 0)*0.01);
            contador4 = false;
        }
        if((contador5 == true)&&((SVSubscriber_ASDU_getINT32 (asdu, 8)*0.01)>=0)){
            max_corrente_b = (SVSubscriber_ASDU_getINT32 (asdu, 8)*0.01);
            contador5 = false;
        }

        if((contador6 == true)&&((SVSubscriber_ASDU_getINT32 (asdu, 16)*0.01)>=0)){
            max_corrente_c = (SVSubscriber_ASDU_getINT32 (asdu, 16)*0.01);
            contador6 = false;
        }

        if(((SVSubscriber_ASDU_getINT32 (asdu, 32)*0.01)>=0)){
            if(teste[contadorSV1]>teste[contadorSV1-1]){
                if((SVSubscriber_ASDU_getINT32 (asdu, 32)*0.01)>max_tensao_a){
                    max_tensao_a = (SVSubscriber_ASDU_getINT32 (asdu, 32)*0.01);
                    ang1 = SVSubscriber_ASDU_getSmpCnt(asdu);
                }
            }   
        }

        if(((SVSubscriber_ASDU_getINT32 (asdu, 40)*0.01)>=0)){
                if((SVSubscriber_ASDU_getINT32 (asdu, 40)*0.01)>max_tensao_b){
                    max_tensao_b = (SVSubscriber_ASDU_getINT32 (asdu, 40)*0.01);
                    ang2 = SVSubscriber_ASDU_getSmpCnt(asdu);
                }
        }

        if(((SVSubscriber_ASDU_getINT32 (asdu, 48)*0.01)>=0)){
                if((SVSubscriber_ASDU_getINT32 (asdu, 48)*0.01)>max_tensao_c){
                    max_tensao_c = (SVSubscriber_ASDU_getINT32 (asdu, 48)*0.01);
                    ang3 = SVSubscriber_ASDU_getSmpCnt(asdu);
                }   
        }
        if(((SVSubscriber_ASDU_getINT32 (asdu, 0)*0.01)>=0)){
                if((SVSubscriber_ASDU_getINT32 (asdu, 0)*0.01)>max_corrente_a){
                    max_corrente_a = (SVSubscriber_ASDU_getINT32 (asdu, 0)*0.01);
                    ang4 = SVSubscriber_ASDU_getSmpCnt(asdu);
                }  
        }

        if(((SVSubscriber_ASDU_getINT32 (asdu, 8)*0.01)>=0)){
                if((SVSubscriber_ASDU_getINT32 (asdu, 8)*0.01)>max_corrente_b){
                    max_corrente_b = (SVSubscriber_ASDU_getINT32 (asdu, 8)*0.01);
                    ang5 = SVSubscriber_ASDU_getSmpCnt(asdu);
                }
        }

        if(((SVSubscriber_ASDU_getINT32 (asdu, 16)*0.01)>=0)){
                if((SVSubscriber_ASDU_getINT32 (asdu, 16)*0.01)>max_corrente_c){
                    max_corrente_c = (SVSubscriber_ASDU_getINT32 (asdu, 16)*0.01);
                    ang6 = SVSubscriber_ASDU_getSmpCnt(asdu);
                }   
        }
        contadorSV1 += 1;
    }    

    if (contadorSV1==80)
    {   
        corrente_primarioA = sqrt(SVrms_deltaA / 80);
        corrente_primarioB = sqrt(SVrms_deltaB / 80);
        corrente_primarioC = sqrt(SVrms_deltaC / 80);
        tensao_primarioA = sqrt(SVrms_deltaA1 / 80);
        tensao_primarioB = sqrt(SVrms_deltaB1 / 80);
        tensao_primarioC = sqrt(SVrms_deltaC1 / 80);
        
        an[1] = (ang1 - ang2)*4.5;
        an[2] = (ang1 - ang3)*4.5;
        an[3] = (ang1 - ang4)*4.5;
        an[4] = (ang1 - ang5)*4.5;
        an[5] = (ang1 - ang6)*4.5;
         if(an[1]<0){
            an[1] = 360 + an[1];
        }if(an[2]<0){
            an[2] = 360 + an[2];
        }if(an[3]<0){
            an[3] = 360 + an[3];
        }if(an[4]<0){
            an[4] = 360 + an[4];
        }if(an[5]<0){
            an[5] = 360 + an[5];
        }

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
        if(funcao == 1){
            j = 1;
        }if (funcao ==2){
            j = tensao_primarioA/Vn;
        }    
 
        M = corrente_primarioA/(j*pick_up);
        contadorSV3 ++;

        t=(T*(K/((pow(M,a))-B)));
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

        if ((t>0)&&(corrente_primarioA>(pick_up*50/100))){
                if (contador == true){
                    gettimeofday( &start_time, NULL );
                    contador = false;
                }
                gettimeofday( &stop_time, NULL );
                time_diff = (float)(stop_time.tv_sec - start_time.tv_sec);
                time_diff += (stop_time.tv_usec - start_time.tv_usec)/(float)MICRO_PER_SECOND;
                //printf("\n%f s\n",time_diff);
                //exit(0);
                if ((time_diff)>=t){
                    printf("-------------------------------------------------------------------------------------------------------------\n");            
                    printf("                         ATUAR FUNÇÃO 51V: SOBRECORRENTE TEMPORIZADA COM RESTRIÇÃO DE TENSÃO                 \n");
                    printf("-------------------------------------------------------------------------------------------------------------\n");
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_P1TPIOC1_Str_general, false);
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
                    /*contador = true;
                    exit(0);*/
                }
        }
        if ((corrente_primarioA<(pick_up*10/100))/*&&(corrente_primarioB<(pick_up*10/100))&&(corrente_primarioC<(pick_up*10/100))*/){
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, false);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_P1TPIOC1_Str_general, true);
                contador = true;
                /*IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_P2TPIOC2_Str_general, false);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_P3TPIOC3_Str_general, false);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BFR1RBRF1_OpEx_general, false);*/
        }

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
            printf("   A tensão RMS da fase A no primário é: %.2f [V] %.1f°\n", tensao_primarioA, an[0] );
            printf("   A tensão RMS da fase B no primário é: %.2f [V] %.2f°\n", tensao_primarioB, an[1] );
            printf("   A tensão RMS da fase C no primário é: %.2f [V] %.1f°\n", tensao_primarioC, an[2] );
            printf("-----------------------------------------------------\n");
            printf("   A corrente RMS da fase A no primário é: %.2f [A] %.1f°\n", corrente_primarioA, an[3] );
            printf("   A corrente RMS da fase B no primário é: %.2f [A] %.1f°\n", corrente_primarioB, an[4] );
            printf("   A corrente RMS da fase C no primário é: %.2f [A] %.1f°\n", corrente_primarioC, an[5] );
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
        GooseReceiver_setInterfaceId(receiver, "p5p1");
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

        }

        /* stop MMS server - close TCP server socket and all client sockets */
        IedServer_stop(iedServer);

        /* Cleanup - free all resources */
        SVReceiver_destroy(receiverSV);
        IedServer_destroy(iedServer);

        return 0;

    //}
} /* main() */
