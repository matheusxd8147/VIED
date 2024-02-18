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

#include "static_model.h"


 /* import IEC 61850 device model created from SCL-File */
extern IedModel iedModel;

int contador = 0; //variável para auxiliar na função de abertura do disjuntor
static int running = 0;
static IedServer iedServer = NULL;

void sigint_handler(int signalId)
{
    running = 0;
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
        IedServer_updateDbposValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_stVal, DBPOS_ON);
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
        IedServer_updateDbposValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_stVal, DBPOS_OFF);
    }
    if (contador == 1) {
        IedServer_updateDbposValue(iedServer, IEDMODEL_LD0_XCBR1_Pos_stVal, DBPOS_OFF);
    }

    /***********seletividade editor****************/
/*
    int posicao=1;
    int final=sizeof(buffer)-posicao;
    char corrente[final];
    memcpy(corrente,&buffer[posicao],final);

    char *leitura_corrente;
    leitura_corrente=strtok(corrente,"}");

    printf("%f \n",atof(leitura_corrente));

    if(atof(leitura_corrente)>=1150)
    {
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_P1TPIOC1_Str_general, true);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true); //nó lógico para envio da mensagem
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BFR1RBRF1_OpEx_general, true);
        printf("Entrou no primeiro \n");
    }

    else
    {
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_P1TPIOC1_Str_general, false);
        printf("Entrou no segundo \n");
    }*/
}

int
main(int argc, char** argv)
{
    IedServerConfig config = IedServerConfig_create();

    iedServer = IedServer_createWithConfig(&iedModel, NULL, config);

    IedServerConfig_destroy(config);

    if (argc > 1) {
        char* ethernetIfcID = argv[1];

        printf("Using GOOSE interface: %s\n", ethernetIfcID);

        /* set GOOSE interface for all GOOSE publishers (GCBs) */
        IedServer_setGooseInterfaceId(iedServer, ethernetIfcID);
    }

    if (argc > 2) {
        char* ethernetIfcID = argv[2];

        printf("Using GOOSE interface for GenericIO/LLN0.gcbAnalogValues: %s\n", ethernetIfcID);

        /* set GOOSE interface for a particular GOOSE publisher (GCB) */
        IedServer_setGooseInterfaceIdEx(iedServer, IEDMODEL_LD0_LLN0, "BRep0201", ethernetIfcID);
    }

    IedServer_enableGoosePublishing(iedServer);

    GooseReceiver receiver = GooseReceiver_create();

    GooseReceiver_setInterfaceId(receiver, "enp1s0f1");

    GooseSubscriber subscriber = GooseSubscriber_create("SEL_751_1CFG/LLN0$GO$GOOSE_SL_1", NULL); //Especificação de quem o ied irá receber as mensagens goose

    GooseSubscriber_setListener(subscriber, gooseListener, iedServer);

    GooseReceiver_addSubscriber(receiver, subscriber);

    GooseReceiver_start(receiver);

    Thread_sleep(1000);

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
    IedServer_destroy(iedServer);

    return 0;
} /* main() */
