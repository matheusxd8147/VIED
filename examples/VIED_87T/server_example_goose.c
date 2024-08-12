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
static int running = 0;
static IedServer iedServer = NULL;
int contadorSV1 = 0;
int contadorSV2 = 0;
float SVrms_deltaA = 0;
float SVrms_deltaB = 0;
float SVrms_deltaC = 0;
float SVrms_estrelaA = 0;
float SVrms_estrelaB = 0;
float SVrms_estrelaC = 0;
static float corrente_primarioA = 0;
static float corrente_primarioB = 0;
static float corrente_primarioC = 0;
static float corrente_secundarioA = 0;
static float corrente_secundarioB = 0;
static float corrente_secundarioC = 0;
static float r_transformacao = 2.8868;

void sigint_handler(int signalId)
{
    running = 0;
}

/* Callback handler for received SV messages */
static void
svUpdateListener (SVSubscriber subscriber, void* parameter, SVSubscriber_ASDU asdu)
{
    //printf("svUpdateListener called\n");

    const char* svID = SVSubscriber_ASDU_getSvId(asdu);
    int teste = 0;

    



    /*
     * Access to the data requires a priori knowledge of the data set.
     * For this example we assume a data set consisting of FLOAT32 values.
     * A FLOAT32 value is encoded as 4 bytes. You can find the first FLOAT32
     * value at byte position 0, the second value at byte position 4, the third
     * value at byte position 8, and so on.
     *
     * To prevent damages due configuration, please check the length of the
     * data block of the SV message before accessing the data.
     */
    /* if (SVSubscriber_ASDU_getDataSize(asdu) >= 8) {
        printf("   Corrente no primário: %f\n", SVSubscriber_ASDU_getFLOAT32(asdu, 0));
        printf("   Corrente no secundário: %f\n", SVSubscriber_ASDU_getFLOAT32(asdu, 4));
        //printf("   O contador é %d\n", contadorSV);
    } */

    //printf("   Valor ao quadrado1: %f\n", pow(SVSubscriber_ASDU_getFLOAT32(asdu, 0),2));
    //printf("   Valor ao quadrado2: %f\n", pow(SVSubscriber_ASDU_getFLOAT32(asdu, 4),2));

    //printf("fora if\n");
    
    
    if ((strcmp(svID,"IEDNameMU0101"))== 0){       
        SVrms_deltaA = (SVrms_deltaA + pow((SVSubscriber_ASDU_getINT32 (asdu, 0)*0.001),2));
        SVrms_deltaB = (SVrms_deltaB + pow((SVSubscriber_ASDU_getINT32 (asdu, 8)*0.001),2));
        SVrms_deltaC = (SVrms_deltaB + pow((SVSubscriber_ASDU_getINT32 (asdu, 16)*0.001),2));
        contadorSV1 += 1;
    } 
    if ((strcmp(svID,"IEDNameMU0201"))== 0){
        SVrms_estrelaA = (SVrms_estrelaA + pow((SVSubscriber_ASDU_getINT32 (asdu, 0)*0.001),2));
        SVrms_estrelaB = (SVrms_estrelaB + pow((SVSubscriber_ASDU_getINT32 (asdu, 8)*0.001),2));
        SVrms_estrelaC = (SVrms_estrelaC + pow((SVSubscriber_ASDU_getINT32 (asdu, 16)*0.001),2));
        contadorSV2 += 1;
    }

    if (contadorSV1==80)
    {
        system ("clear");
        printf("  svID=(%s)\n", svID);
        printf("  smpCnt: %i\n", SVSubscriber_ASDU_getSmpCnt(asdu));
        printf("  confRev: %u\n", SVSubscriber_ASDU_getConfRev(asdu));
        corrente_primarioA = sqrt(SVrms_deltaA / 80);
        corrente_primarioB = sqrt(SVrms_deltaB / 80);
        corrente_primarioC = sqrt(SVrms_deltaC / 80);
        printf("   A corrente RMS da fase A no primário é: %f [A]\n", corrente_primarioA );
        printf("   A corrente RMS da fase B no primário é: %f [A]\n", corrente_primarioB );
        printf("   A corrente RMS da fase C no primário é: %f [A]\n", corrente_primarioC );
        contadorSV1 += 1;
    }

        if (contadorSV2==80)
    {
        //system ("clear");
        //printf("   A corrente RMS da fase A no primário é: %f [A]\n", corrente_primarioA );
        printf("  svID=(%s)\n", svID);
        printf("  smpCnt: %i\n", SVSubscriber_ASDU_getSmpCnt(asdu));
        printf("  confRev: %u\n", SVSubscriber_ASDU_getConfRev(asdu));
        corrente_primarioA = sqrt(SVrms_deltaA / 80);
        corrente_primarioB = sqrt(SVrms_deltaB / 80);
        corrente_primarioC = sqrt(SVrms_deltaB / 80);
        corrente_secundarioA = sqrt(SVrms_estrelaA / (80))/r_transformacao;
        corrente_secundarioB = sqrt(SVrms_estrelaB / (80))/r_transformacao;
        corrente_secundarioC = sqrt(SVrms_estrelaC / (80))/r_transformacao;
        printf("   A corrente RMS da fase A no secundário refletida no primário é: %f [A]\n", corrente_secundarioA);
        printf("   A corrente RMS da fase B no secundário refletida no primário é: %f [A]\n", corrente_secundarioB);
        printf("   A corrente RMS da fase C no secundário refletida no primário é: %f [A]\n", corrente_secundarioC);
        contadorSV1=0;
        contadorSV2=0;
        SVrms_deltaA = 0;
        SVrms_deltaB = 0;
        SVrms_deltaC = 0;
        SVrms_estrelaA = 0;
        SVrms_estrelaB = 0;
        SVrms_estrelaC = 0;
        
        //(abs(corrente_primarioA-corrente_secundarioA)/corrente_primarioA)*100>30
        uint64_t timestamp = Hal_getTimeInMs();

       if ((abs(corrente_primarioA-corrente_secundarioA)/corrente_primarioA)*100>30)
        {
            printf("-------------------------------------------------------------------------------------------------------------\n");            
            printf("   ATUAR FUNÇÃO 87T: Diferença maior que 30%% na fase A -----> COMANDO DE ABERTURA DE 12T1 E 11T1 ENVIADO\n");
            printf("-------------------------------------------------------------------------------------------------------------\n");
            IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_t, timestamp);
            IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_LD0_A87T_PDIF1_Op_t, timestamp);
            IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_LD0_PTRC1_Tr_t, timestamp);
            IedServer_updateDbposValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_stVal, DBPOS_OFF);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_LD0_A87T_PDIF1_Op_general, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_LD0_PTRC1_Tr_general, true);
            printf("\n");
            teste = teste+1;
        }
        if((abs(corrente_primarioB-corrente_secundarioB)/corrente_primarioB)*100>30){
            printf("-------------------------------------------------------------------------------------------------------------\n");
            printf("   ATUAR FUNÇÃO 87T: Diferença maior que 30%% na fase B  -----> COMANDO DE ABERTURA DE 12T1 E 11T1 ENVIADO\n");
            printf("-------------------------------------------------------------------------------------------------------------\n");
            IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_t, timestamp);
            IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_LD0_A87T_PDIF1_Op_t, timestamp);
            IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_LD0_PTRC1_Tr_t, timestamp);
            IedServer_updateDbposValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_stVal, DBPOS_OFF);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_LD0_A87T_PDIF1_Op_general, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_LD0_PTRC1_Tr_general, true);
            printf("\n");
            teste = teste+1;
        }
        if((abs(corrente_primarioC-corrente_secundarioC)/corrente_primarioC)*100>30){
            printf("-------------------------------------------------------------------------------------------------------------\n");
            printf("   ATUAR FUNÇÃO 87T: Diferença maior que 30%% na fase C  -----> COMANDO DE ABERTURA DE 12T1 E 11T1 ENVIADO\n");
            printf("-------------------------------------------------------------------------------------------------------------\n");
            IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_t, timestamp);
            IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_LD0_A87T_PDIF1_Op_t, timestamp);
            IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_LD0_PTRC1_Tr_t, timestamp);
            IedServer_updateDbposValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_stVal, DBPOS_OFF);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_LD0_A87T_PDIF1_Op_general, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_LD0_PTRC1_Tr_general, true);
            printf("\n");
            teste = teste+1;
        }
        if (((abs(corrente_primarioA-corrente_secundarioA)/corrente_primarioA)*100>30)&&((abs(corrente_primarioB-corrente_secundarioB)/corrente_primarioB)*100>30)&&((abs(corrente_primarioC-corrente_secundarioC)/corrente_primarioC)*100>30)){
            IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_t, timestamp);
            IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_LD0_A87T_PDIF1_Op_t, timestamp);
            IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_LD0_PTRC1_Tr_t, timestamp);
            IedServer_updateDbposValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_stVal, DBPOS_ON);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_LD0_A87T_PDIF1_Op_general, false);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_LD0_PTRC1_Tr_general, false);
        }else{
            exit(0);
        }
       }
            
     }
    




/*void
controlHandlerForBinaryOutput(ControlAction action, void* parameter, MmsValue* value)
{
    uint64_t timestamp = Hal_getTimeInMs();

    if (parameter == IEDMODEL_CON_RBGGIO1_SPCSO01) {
        IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_t, timestamp);
        IedServer_updateAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, value);
    }

    if (parameter == IEDMODEL_CON_RBGGIO1_SPCSO02) {
        IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_t, timestamp);
        IedServer_updateAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, value);
    }

    if (parameter == IEDMODEL_CON_RBGGIO1_SPCSO03) {
        IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO03_t, timestamp);
        IedServer_updateAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO03_stVal, value);
    }

    if (parameter == IEDMODEL_CON_RBGGIO1_SPCSO04) {
        IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO04_t, timestamp);
        IedServer_updateAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO04_stVal, value);
    }
}
*/
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

    if ((b != 116) && (c != 116) && (d != 116) && (contador == 0)) {
        IedServer_updateDbposValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_stVal, DBPOS_OFF);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_LD0_PTRC1_Tr_general, false);
    }

    if ((b == 116) && (c != 116) && (d != 116)) //116 é o valor da tabela ASCII para T (True)
    {
        printf("CORRENTE DE FALTA \n");
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_LD0_PTRC1_Tr_general, false);
    }

    if ((b == 116) && (c == 116) && (d != 116))//116 é o valor da tabela ASCII para T (True)
    {
        printf("NÃO ATUAR - SELETIVIDADE LÓGICA \n");
    }

    if ((b == 116) && (c == 116) && (d == 116))//116 é o valor da tabela ASCII para T (True)
    {
        contador = 1;
        printf("FALHA DE DISJUNTOR \n");
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_LD0_PTRC1_Tr_general, true);
        printf("SINAL DE TRIP ENVIADO AO DISJUNTOR \n");
        printf("DISJUNTOR ABERTO \n");
        IedServer_updateDbposValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_stVal, DBPOS_ON);
    }
    if (contador == 1) {
        IedServer_updateDbposValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_stVal, DBPOS_OFF);
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
        IedServer_setGooseInterfaceIdEx(iedServer, IEDMODEL_LD0_LLN0, "BRep0201", ethernetIfcID);
    }

    /*Preparando o código para receber mensagens SV*/

    SVSubscriber subscriberSV = SVSubscriber_create(NULL, 0x4000);
    SVSubscriber_setListener(subscriberSV, svUpdateListener, NULL);
    SVReceiver_addSubscriber(receiverSV, subscriberSV);

    /*Preparando o código para publicar mensagens GOOSE*/

    IedServer_enableGoosePublishing(iedServer);
    GooseReceiver receiver = GooseReceiver_create();
    GooseReceiver_setInterfaceId(receiver, "enp1s0f1");
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

        Thread_sleep(1000);
    }

    /* stop MMS server - close TCP server socket and all client sockets */
    IedServer_stop(iedServer);

    /* Cleanup - free all resources */
    SVReceiver_destroy(receiverSV);
    IedServer_destroy(iedServer);

    return 0;
} /* main() */
