/*
 *  server_example_goose.c
 *
 *  This example demonstrates how to use GOOSE publishing, Reporting and the
 *  control model.
 *
 */

#include "iec61850_client.h"
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
#include <complex.h>
#define MICRO_PER_SECOND 1000000
#define pi 3.14

Thread thread_50, thread_50N, thread_51, thread_51V, thread_51N, thread_67, thread_67N;
Thread self_healing;


/* import IEC 61850 device model created from SCL-File */
extern IedModel iedModel;

bool contador = true;
bool contador10 = true;
bool contador9 = true;
bool contador11 = true;
bool contador7 = true;
bool contador8 = true;
bool contador1 = true;
bool contador2 = true;
bool contador3 = true;
bool contador4 = true;
bool contador5 = true;
bool contador6 = true;
bool contador12 = true;
bool contador13 = true;
bool maximo1 = true;
bool maximo2 = true;
bool resposta = true;
bool comando = false;
int curva = 0;
int funcao = 0;

char* svID2;
static int running = 0;
static IedServer iedServer = NULL;
int contadorSV1 = 0;
int contadorSV2 = 0;
int contadorSV3 = 0;
float j = 0;
float SVrms_deltaA = 0;
float SVrms_deltaB = 0;
float SVrms_deltaC = 0;
float SVrms_deltaN = 0;
float SVrms_deltaA1 = 0;
float SVrms_deltaB1 = 0;
float SVrms_deltaC1 = 0;
float SVrms_deltaN1 = 0;
float max_corrente_a = 0;
float max_corrente_b = 0;
float max_corrente_c = 0;
float max_corrente_n = 0;
float max_tensao_a = 0;
float max_tensao_b = 0;
float max_tensao_c = 0;
float max_tensao_n = 0;
static float corrente_primarioA = 0;
static float corrente_primarioB = 0;
static float corrente_primarioC = 0;
static float corrente_primarioN = 0;
static float tensao_primarioA = 0;
static float tensao_primarioB = 0;
static float tensao_primarioC = 0;
static float tensao_primarioN = 0;
float teste[80];
static float K_51, K_51N;
static float K_51V;
static float K_67, K_67N;
static float alfa_51, alfa_51N;
static float alfa_51V;
static float alfa_67, alfa_67N;
static float M1_51A, M2_51B, M3_51C, a, t, t1, t2, B = 1;
static float M1_51V_A, M2_51V_B, M3_51V_C;
static float M1_67A, M2_67B, M3_67C;
static float M1_50N, M1_51N, M1_67N;
static float act_51A, act_51B, act_51C, act_51N;
static float act_51V_A, act_51V_B, act_51V_C;
static float act_67A, act_67B, act_67C, act_67N;
static bool enable_51A = true;
static bool enable_51B = true;
static bool enable_51C = true; 
static bool enable_51V_A = true;
static bool enable_51V_B = true;
static bool enable_51V_C = true;
static bool enable_51N = true; 
static bool enable_67A = true;
static bool enable_67B = true;
static bool enable_67C = true; 
static bool enable_67N = true;
struct timeval start_51A, start_51B, start_51C, start_51N;
struct timeval stop_51A, stop_51B, stop_51C, stop_51N;
struct timeval start_51V_A, start_51V_B, start_51V_C, start_51V_N;
struct timeval stop_51V_A, stop_51V_B, stop_51V_C, stop_51V_N;
struct timeval start_67A, start_67B, start_67C, start_67N;
struct timeval stop_67A, stop_67B, stop_67C, stop_67N;
struct timeval start_50_62BF;
struct timeval stop_50_62BF;
static float ang1, ang2, ang3, ang4, ang5, ang6, ang7, ang8;
static float time_51A, time_51B, time_51C, time_51N, time_50_62BF;
static float time_51V_A, time_51V_B, time_51V_C, time_51V_N;
static float time_67A, time_67B, time_67C, time_67N;
static float ime_diff = 0, ime_diff1 = 0, ime_diff2 = 0;
double an[8], ar[6], br[6], teta, teta1, teta2, torque, torque1, torque2, tetaN, torqueN;
double complex yr[6], Vbc, aVbc, mVbc,Vca, aVca, mVca, Vab, aVab, mVab;

static int funcoes, curva_51, curva_51V, curva_51N, curva_67, curva_67N;
static float pick_up_50, pick_up_50N, pick_up_51, pick_up_51V, pick_up_51N, pick_up_67, pick_up_67N, atm_67, atm_67N;
static float dial_51, dial_51V, dial_51N, dial_67, dial_67N, tensao_51V;

static float a, b, c, d, e, f, g, h, i;
static float a1, b1, c1, d1, e1, f1, g1, h1, i1;
static float a2, b2, c2, d2, e2, f2, g2, h2, i2;
static float pMax_21l9, pMax_21l8;

static float pMaxS21l1 = 215, pMaxS21l2 = 215, pMaxS21l3 = 215;

static char sh, trip_21l9, trip_21l6, trip_21l7;

static int estado_dj_21l9, estado_dj_21l6, estado_dj_21l7;

static char comando_received_21l6_A, comando_received_21l7_A, comando_received_21l6_F, comando_received_21l7_F;

struct timeval delay1, delay2, delay3, delay4, delay5;
struct timeval delay6, delay7, delay8, delay9, delay10;
struct timeval delay1p, delay2p, delay3p, delay4p, delay5p;
struct timeval delay6p, delay7p, delay8p, delay9p, delay10p;
static float time_delay1, time_delay2, time_delay3, time_delay4, time_delay5;
static float time_delay6, time_delay7, time_delay8, time_delay9, time_delay10;

bool delay_1 = true;
bool delay_2 = true;
bool delay_3 = true;
bool delay_4 = true;
bool delay_5 = true;
bool delay_6 = true;
bool delay_7 = true;
bool delay_8 = true;
bool delay_9 = true;
bool delay_10 = true;

void sigint_handler(int signalId)
{
	running = 0;
}

void funcao_50_62BF()
{
    if (contador9 == true)
    {
        gettimeofday(&start_50_62BF, NULL);
        contador9 = false;
    }
    gettimeofday(&stop_50_62BF, NULL);
    time_50_62BF = (float)(stop_50_62BF.tv_sec - start_50_62BF.tv_sec);
    time_50_62BF += (stop_50_62BF.tv_usec - start_50_62BF.tv_usec) / (float)MICRO_PER_SECOND;
    if ((time_50_62BF) >= 0.250)
    {
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BFR1RBRF1_OpEx_general, true);
    }
}

void funcao_50()
{
    while (1)
    {

        if (corrente_primarioA > pick_up_50)
        {
            printf("-------------------------------------------------------------------------------------------------------------\n");
            printf("                         ATUAR FUNÇÃO 50: SOBRECORRENTE INSTANTÂNEA FASE A                                   \n");
            printf("-------------------------------------------------------------------------------------------------------------\n");
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind04_stVal, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
        }
        if (corrente_primarioB > pick_up_50)
        {
            printf("-------------------------------------------------------------------------------------------------------------\n");
            printf("                         ATUAR FUNÇÃO 50: SOBRECORRENTE INSTANTÂNEA FASE B                                   \n");
            printf("-------------------------------------------------------------------------------------------------------------\n");
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind05_stVal, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
        }
        if (corrente_primarioC > pick_up_50)
        {
            printf("-------------------------------------------------------------------------------------------------------------\n");
            printf("                         ATUAR FUNÇÃO 50: SOBRECORRENTE INSTANTÂNEA FASE C                                   \n");
            printf("-------------------------------------------------------------------------------------------------------------\n");
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind06_stVal, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
        }
        if (((corrente_primarioA) || (corrente_primarioB) || (corrente_primarioC)) > pick_up_50)
        {
            funcao_50_62BF();
        }
        Thread_sleep(17);
    }
}

void funcao_50N()
{
    while(1){
    if ((corrente_primarioN) >= pick_up_50N)
    {
        printf("-------------------------------------------------------------------------------------------------------------\n");
        printf("                       ATUAR FUNÇÃO 50N: SOBRECORRENTE TEMPORIZADA DE NEUTRO                                 \n");
        printf("-------------------------------------------------------------------------------------------------------------\n");
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind14_stVal, true);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
        funcao_50_62BF();
    }
    Thread_sleep(17);
    }
}

void funcao_51()
{
    while(1){
    M1_51A = corrente_primarioA / pick_up_51;
    act_51A = (dial_51 * (K_51 / ((pow(M1_51A, alfa_51)) - B)));

    M2_51B = corrente_primarioB / pick_up_51;
    act_51B = (dial_51 * (K_51 / ((pow(M2_51B, alfa_51)) - B)));

    M3_51C = corrente_primarioC / pick_up_51;
    act_51C = (dial_51 * (K_51 / ((pow(M3_51C, alfa_51)) - B)));

    if (act_51A > 0)
    {
        if (enable_51A == true)
        {
            gettimeofday(&start_51A, NULL);
            enable_51A = false;
        }
        gettimeofday(&stop_51A, NULL);
        time_51A = (float)(stop_51A.tv_sec - start_51A.tv_sec);
        time_51A += (stop_51A.tv_usec - start_51A.tv_usec) / (float)MICRO_PER_SECOND;
        if ((time_51A) >= act_51A)
        {
            if (funcao == 1)
            {
                // IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_ATPTOC20_Op_general, true);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind20_stVal, true);
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                              ATUAR FUNÇÃO 51: SOBRECORRENTE TEMPORIZADA FASE A                              \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
            }
            else
            {
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind01_stVal, true);
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                   ATUAR FUNÇÃO 51V: SOBRECORRENTE TEMPORIZADA COM RESTRIÇÃO DE TENSÃO FASE A                \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
            }
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
        }
    }
    if (act_51B > 0)
    {
        if (enable_51B == true)
        {
            gettimeofday(&start_51B, NULL);
            enable_51B = false;
        }
        gettimeofday(&stop_51B, NULL);
        time_51B = (float)(stop_51B.tv_sec - start_51B.tv_sec);
        time_51B += (stop_51B.tv_usec - start_51B.tv_usec) / (float)MICRO_PER_SECOND;
        if ((time_51B) >= act_51B)
        {
            if (funcao == 1)
            {
                // IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BTPTOC21_Op_general, true);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind21_stVal, true);
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                              ATUAR FUNÇÃO 51: SOBRECORRENTE TEMPORIZADA FASE B                              \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
            }
            else
            {
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind02_stVal, true);
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                   ATUAR FUNÇÃO 51V: SOBRECORRENTE TEMPORIZADA COM RESTRIÇÃO DE TENSÃO FASE B                \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
            }
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
        }
    }
    if (act_51C > 0)
    {
        if (enable_51C == true)
        {
            gettimeofday(&start_51C, NULL);
            enable_51C = false;
        }
        gettimeofday(&stop_51C, NULL);
        time_51C = (float)(stop_51C.tv_sec - start_51C.tv_sec);
        time_51C += (stop_51C.tv_usec - start_51C.tv_usec) / (float)MICRO_PER_SECOND;
        if ((time_51C) >= act_51C)
        {
            if (funcao == 1)
            {
                // IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_CTPTOC22_Op_general, true);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind22_stVal, true);
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                              ATUAR FUNÇÃO 51: SOBRECORRENTE TEMPORIZADA FASE C                              \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
            }
            else
            {
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind03_stVal, true);
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                   ATUAR FUNÇÃO 51V: SOBRECORRENTE TEMPORIZADA COM RESTRIÇÃO DE TENSÃO FASE C                \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
            }
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
        }
    }Thread_sleep(17);}
}

void funcao_51V()
{
    while(1){
    j = tensao_primarioA / tensao_51V;

    M1_51V_A = corrente_primarioA / (j * pick_up_51V);
    act_51V_A = (dial_51V * (K_51V / ((pow(M1_51V_A, alfa_51V)) - B)));

    M2_51V_B = corrente_primarioB / (j * pick_up_51V);
    act_51V_B = (dial_51V * (K_51V / ((pow(M2_51V_B, alfa_51V)) - B)));

    M3_51V_C = corrente_primarioC / (j * pick_up_51V);
    act_51V_C = (dial_51V * (K_51V / ((pow(M3_51V_C, alfa_51V)) - B)));

    if (act_51V_A > 0)
    {
        if (enable_51V_A == true)
        {
            gettimeofday(&start_51V_A, NULL);
            enable_51V_A = false;
        }
        gettimeofday(&stop_51V_A, NULL);
        time_51V_A = (float)(stop_51V_A.tv_sec - start_51V_A.tv_sec);
        time_51V_A += (stop_51V_A.tv_usec - start_51V_A.tv_usec) / (float)MICRO_PER_SECOND;
        if ((time_51V_A) >= act_51V_A)
        {
            if (funcao == 1)
            {
                // IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_ATPTOC20_Op_general, true);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind20_stVal, true);
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                              ATUAR FUNÇÃO 51: SOBRECORRENTE TEMPORIZADA FASE A                              \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
            }
            else
            {
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind01_stVal, true);
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                   ATUAR FUNÇÃO 51V: SOBRECORRENTE TEMPORIZADA COM RESTRIÇÃO DE TENSÃO FASE A                \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
            }
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
        }
    }
    if (act_51V_B > 0)
    {
        if (enable_51V_B == true)
        {
            gettimeofday(&start_51V_B, NULL);
            enable_51V_B = false;
        }
        gettimeofday(&stop_51V_B, NULL);
        time_51V_B = (float)(stop_51V_B.tv_sec - start_51V_B.tv_sec);
        time_51V_B += (stop_51V_B.tv_usec - start_51V_B.tv_usec) / (float)MICRO_PER_SECOND;
        if ((time_51V_B) >= act_51V_B)
        {
            if (funcao == 1)
            {
                // IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BTPTOC21_Op_general, true);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind21_stVal, true);
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                              ATUAR FUNÇÃO 51: SOBRECORRENTE TEMPORIZADA FASE B                              \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
            }
            else
            {
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind02_stVal, true);
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                   ATUAR FUNÇÃO 51V: SOBRECORRENTE TEMPORIZADA COM RESTRIÇÃO DE TENSÃO FASE B                \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
            }
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
        }
    }
    if (act_51V_C > 0)
    {
        if (enable_51V_C == true)
        {
            gettimeofday(&start_51V_C, NULL);
            enable_51V_C = false;
        }
        gettimeofday(&stop_51V_C, NULL);
        time_51V_C = (float)(stop_51V_C.tv_sec - start_51V_C.tv_sec);
        time_51V_C += (stop_51V_C.tv_usec - start_51V_C.tv_usec) / (float)MICRO_PER_SECOND;
        if ((time_51V_C) >= act_51V_C)
        {
            if (funcao == 1)
            {
                // IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_CTPTOC22_Op_general, true);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind22_stVal, true);
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                              ATUAR FUNÇÃO 51: SOBRECORRENTE TEMPORIZADA FASE C                              \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
            }
            else
            {
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind03_stVal, true);
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                   ATUAR FUNÇÃO 51V: SOBRECORRENTE TEMPORIZADA COM RESTRIÇÃO DE TENSÃO FASE C                \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
            }
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
        }
    }Thread_sleep(17);}
}

void funcao_51N()
{
    while(1){
    M1_51N = corrente_primarioN / (pick_up_51N);
    t = (dial_51N * (K_51N / ((pow(M1_51N, a)) - B)));

    if (act_51N > 0)
    {
        if (enable_51N == true)
        {
            gettimeofday(&start_51N, NULL);
            enable_51N = false;
        }
        gettimeofday(&stop_51N, NULL);
        time_51N = (float)(stop_51N.tv_sec - start_51N.tv_sec);
        time_51N += (stop_51N.tv_usec - start_51N.tv_usec) / (float)MICRO_PER_SECOND;
        if ((time_51N) >= act_51N)
        {
            printf("-------------------------------------------------------------------------------------------------------------\n");
            printf("                       ATUAR FUNÇÃO 51N: SOBRECORRENTE TEMPORIZADA DE NEUTRO                                 \n");
            printf("-------------------------------------------------------------------------------------------------------------\n");
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind13_stVal, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
        }
    }Thread_sleep(17);}
}

void funcao_67()
{
    while(1){
    ar[0] = max_tensao_a * cos(an[0]);
    br[0] = max_tensao_a * sin(an[0]);
    ar[1] = max_tensao_b * cos(an[1]);
    br[1] = max_tensao_b * sin(an[1]);
    ar[2] = max_tensao_c * cos(an[2]);
    br[2] = max_tensao_c * sin(an[2]);
    ar[3] = max_corrente_a * cos(an[3]);
    br[3] = max_corrente_a * sin(an[3]);
    ar[4] = max_corrente_b * cos(an[4]);
    br[4] = max_corrente_b * sin(an[4]);
    ar[5] = max_corrente_c * cos(an[5]);
    br[5] = max_corrente_c * sin(an[5]);
    yr[0] = ar[0] + br[0] * I;
    yr[1] = ar[1] + br[1] * I;
    yr[2] = ar[2] + br[2] * I;
    yr[3] = ar[3] + br[3] * I;
    yr[4] = ar[4] + br[4] * I;
    yr[5] = ar[5] + br[5] * I;

    Vbc = yr[1] - yr[2];
    Vca = yr[2] - yr[0];
    Vab = yr[0] - yr[1];
    aVbc = atan(cimag(Vbc) / creal(Vbc));
    aVca = atan(cimag(Vca) / creal(Vca));
    aVab = atan(cimag(Vab) / creal(Vab));
    mVbc = sqrt(pow(creal(Vbc), 2) + pow(cimag(Vbc), 2));
    mVca = sqrt(pow(creal(Vca), 2) + pow(cimag(Vca), 2));
    mVab = sqrt(pow(creal(Vab), 2) + pow(cimag(Vab), 2));
    teta = an[3] - aVbc;
    teta1 = an[4] - aVca;
    teta2 = an[5] - aVab;
    torque = max_corrente_a * mVbc * cos((atm_67 - teta) * M_PI / 180);
    torque1 = max_corrente_b * mVca * cos((atm_67 - teta1) * M_PI / 180);
    torque2 = max_corrente_c * mVab * cos((atm_67 - teta2) * M_PI / 180);

    if (torque > 0)
    {
        M1_67A = corrente_primarioA / (pick_up_67N);
        act_67A = (dial_67 * (K_67 / ((pow(M1_67A, alfa_67)) - B)));
        if (act_67A > 0)
        {
            if (enable_67A == true)
            {
                gettimeofday(&start_67A, NULL);
                enable_67A = false;
            }
            gettimeofday(&stop_67A, NULL);
            time_67A = (float)(stop_67A.tv_sec - start_67A.tv_sec);
            time_67A += (stop_67A.tv_usec - start_67A.tv_usec) / (float)MICRO_PER_SECOND;
            if ((time_67A) >= act_67A)
            {
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                         ATUAR FUNÇÃO 67: SOBRECORRENTE DIRECIONAL TEMPORIZADA FASE A                        \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind07_stVal, true);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
            }
        }
    }

    if (torque1 > 0)
    {
        M2_67B = corrente_primarioB / (pick_up_67);
        act_67B = (dial_67 * (K_67 / ((pow(M2_67B, alfa_67)) - B)));
        if (act_67B > 0)
        {
            if (enable_67B == true)
            {
                gettimeofday(&start_67B, NULL);
                enable_67B = false;
            }
            gettimeofday(&stop_67B, NULL);
            time_67B = (float)(stop_67B.tv_sec - start_67B.tv_sec);
            time_67B += (stop_67B.tv_usec - start_67B.tv_usec) / (float)MICRO_PER_SECOND;
            if ((time_67B) >= act_67B)
            {
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                         ATUAR FUNÇÃO 67: SOBRECORRENTE DIRECIONAL TEMPORIZADA FASE B                        \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind08_stVal, true);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
            }
        }
    }

    if (torque2 > 0)
    {
        M3_67C = corrente_primarioC / (pick_up_67);
        act_67C = (dial_67 * (K_67 / ((pow(M3_67C, alfa_67)) - B)));
        if (act_67C > 0)
        {
            if (enable_67C == true)
            {
                gettimeofday(&start_67C, NULL);
                enable_67C = false;
            }
            gettimeofday(&stop_67C, NULL);
            time_67C = (float)(stop_67C.tv_sec - start_67C.tv_sec);
            time_67C += (stop_67C.tv_usec - start_67C.tv_usec) / (float)MICRO_PER_SECOND;
            if ((time_67C) >= act_67C)
            {
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                         ATUAR FUNÇÃO 67: SOBRECORRENTE DIRECIONAL TEMPORIZADA FASE C                        \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind09_stVal, true);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
            }
        }
    }Thread_sleep(17);}
}

void funcao_67N()
{
    while(1){
    if (tensao_primarioN > (tensao_51V / 10))
    {
        tetaN = an[7] - (an[6]);
        torqueN = max_tensao_n * max_corrente_n * cos((atm_67N - tetaN) * pi / 180);
    }

    if (torqueN > 0)
    {
        M1_67N = corrente_primarioN / (pick_up_67N);
        act_67N = (dial_67N * (K_67N / ((pow(M1_67N, alfa_67N)) - B)));
        if (act_67N > 0)
        {
            if (enable_67N == true)
            {
                gettimeofday(&start_67N, NULL);
                enable_67N = false;
            }
            gettimeofday(&stop_67N, NULL);
            time_67N = (float)(stop_67N.tv_sec - start_67N.tv_sec);
            time_67N += (stop_67N.tv_usec - start_67N.tv_usec) / (float)MICRO_PER_SECOND;
            if ((time_67N) >= act_67N)
            {
                printf("-------------------------------------------------------------------------------------------------------------\n");
                printf("                       ATUAR FUNÇÃO 67N: DIRECIONAL DE SOBRECORRENTE TEMPORIZADA DE NEUTRO                   \n");
                printf("-------------------------------------------------------------------------------------------------------------\n");
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind12_stVal, true);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
            }
        }
    }Thread_sleep(17);}
}


/* Callback handler for received SV messages */
static void
svUpdateListener (SVSubscriber subscriber, void* parameter, SVSubscriber_ASDU asdu)
{   
    int i;
    const char* svID = SVSubscriber_ASDU_getSvId(asdu);
    if(resposta == true){
        resposta = false;
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind10_stVal, false);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind11_stVal, false);
    }

    if ((strcmp(svID,"VMU08"))== 0){      
        
        
        SVrms_deltaA = (SVrms_deltaA + pow((SVSubscriber_ASDU_getINT32 (asdu, 0)*0.001),2));
        SVrms_deltaA1 = (SVrms_deltaA1 + pow((SVSubscriber_ASDU_getINT32 (asdu, 32)*0.01),2));
        SVrms_deltaB = (SVrms_deltaB + pow((SVSubscriber_ASDU_getINT32 (asdu, 8)*0.001),2));
        SVrms_deltaB1 = (SVrms_deltaB1 + pow((SVSubscriber_ASDU_getINT32 (asdu, 40)*0.01),2));
        SVrms_deltaC = (SVrms_deltaC + pow((SVSubscriber_ASDU_getINT32 (asdu, 16)*0.001),2));
        SVrms_deltaC1 = (SVrms_deltaC1 + pow((SVSubscriber_ASDU_getINT32 (asdu, 48)*0.01),2));
        SVrms_deltaN = (SVrms_deltaN + pow((SVSubscriber_ASDU_getINT32 (asdu, 24)*0.001),2));
        SVrms_deltaN1 = (SVrms_deltaN1 + pow((SVSubscriber_ASDU_getINT32 (asdu, 56)*0.01),2));

        
        if((SVSubscriber_ASDU_getINT32 (asdu, 32)*0.01)>=0){
            teste[contadorSV1] = (SVSubscriber_ASDU_getINT32 (asdu, 32)*0.01);
        }

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

        if((contador12 == true)&&((SVSubscriber_ASDU_getINT32 (asdu, 56)*0.01)>=0)){
            max_tensao_n = (SVSubscriber_ASDU_getINT32 (asdu, 56)*0.01);
            contador12 = false;
        }

        if((contador13 == true)&&((SVSubscriber_ASDU_getINT32 (asdu, 24)*0.001)>=0)){
            max_corrente_n = (SVSubscriber_ASDU_getINT32 (asdu, 24)*0.001);
            contador13 = false;
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

        if(((SVSubscriber_ASDU_getINT32 (asdu, 56)*0.01)>=0)){
                if((SVSubscriber_ASDU_getINT32 (asdu, 56)*0.01)>max_tensao_n){
                    max_tensao_n = (SVSubscriber_ASDU_getINT32 (asdu, 56)*0.01);
                    ang8 = SVSubscriber_ASDU_getSmpCnt(asdu);
                }   
        }

        if(((SVSubscriber_ASDU_getINT32 (asdu, 24)*0.001)>=0)){
                if((SVSubscriber_ASDU_getINT32 (asdu, 24)*0.001)>max_corrente_n){
                    max_corrente_n = (SVSubscriber_ASDU_getINT32 (asdu, 24)*0.001);
                    ang7 = SVSubscriber_ASDU_getSmpCnt(asdu);
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

        /*funcao_50();
        funcao_50N();
        funcao_51();
        funcao_51V();
        funcao_51N();
        funcao_67();
        funcao_67N();*/

        contadorSV3 ++;

        if(contadorSV3 == 60){
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
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_A_phsA_cVal_mag_f, corrente_primarioA);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_A_phsB_cVal_mag_f, corrente_primarioB);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_A_phsC_cVal_mag_f, corrente_primarioC);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_PPV_phsAB_cVal_mag_f, tensao_primarioA);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_PPV_phsBC_cVal_mag_f, tensao_primarioB);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_PPV_phsCA_cVal_mag_f, tensao_primarioC);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_A_phsA_cVal_ang_f, an[3]);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_A_phsB_cVal_ang_f, an[4]);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_A_phsC_cVal_ang_f, an[5]);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_PPV_phsAB_cVal_ang_f, an[0]);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_PPV_phsBC_cVal_ang_f, an[1]);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_PPV_phsCA_cVal_ang_f, an[2]);
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

void
controlHandlerForBinaryOutput(ControlAction action, void* parameter, MmsValue* value, bool test)
{

    if (MmsValue_getType(value) == MMS_BOOLEAN) {
        printf("received binary control command: ");

        if (MmsValue_getBoolean(value)){
            printf("on\n");

            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, false);
        }
        else{
            printf("off\n");

            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, false);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, true);
        }
            
    }
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

    if (parameter == IEDMODEL_CON_RBGGIO1_SPCSO05) {
        IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO05_t, timestamp);
        IedServer_updateAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO05_stVal, value);
    }

    if (parameter == IEDMODEL_CON_RBGGIO1_SPCSO06) {
        IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO06_t, timestamp);
        IedServer_updateAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO06_stVal, value);
    }

    if (parameter == IEDMODEL_CON_RBGGIO1_SPCSO07) {
        IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO07_t, timestamp);
        IedServer_updateAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO07_stVal, value);
    }

    if (parameter == IEDMODEL_PRO_BKR1CSWI1_Pos) {
        //IedServer_getAttributeValue(iedServer, IEDMODEL_PRO_BKR1CSWI1_Pos_Oper_ctlVal);
        
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
gooseListener6(GooseSubscriber subscriber, void* parameter)
{
    MmsValue* values = GooseSubscriber_getDataSetValues(subscriber);

    char buffer[50];

    MmsValue_printToBuffer(values, buffer, 50);

    trip_21l6 = buffer[1];
    estado_dj_21l6 = atoi(&buffer[7]);

    uint64_t y = Hal_getTimeInMs();

    /*printf("-------------------------------------------------------------------------------------------------------------\n");            
    printf("                               PRIMEIRA MENSAGEM GOOSE ASSINADA VIED 1                                       \n");
    printf("-------------------------------------------------------------------------------------------------------------\n");*/

}

static void
gooseListener7(GooseSubscriber subscriber, void* parameter)
{
    MmsValue* values = GooseSubscriber_getDataSetValues(subscriber);

    char buffer[50];

    MmsValue_printToBuffer(values, buffer, 50);

    trip_21l7 = buffer[1];
    estado_dj_21l7 = atoi(&buffer[7]);

    uint64_t y = Hal_getTimeInMs();

    /*printf("-------------------------------------------------------------------------------------------------------------\n");            
    printf("                               PRIMEIRA MENSAGEM GOOSE ASSINADA VIED 1                                       \n");
    printf("-------------------------------------------------------------------------------------------------------------\n");*/

}

static void
gooseListener(GooseSubscriber subscriber, void* parameter)
{
    MmsValue* values = GooseSubscriber_getDataSetValues(subscriber);

    char buffer[50];

    MmsValue_printToBuffer(values, buffer, 50);

    trip_21l9 = buffer[1];

    if (trip_21l9 == 116){
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, false);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, true);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind23_stVal, true);
    }

    estado_dj_21l9 = atoi(&buffer[7]);

    uint64_t y = Hal_getTimeInMs();

    /*printf("-------------------------------------------------------------------------------------------------------------\n");            
    printf("                               PRIMEIRA MENSAGEM GOOSE ASSINADA VIED 1                                       \n");
    printf("-------------------------------------------------------------------------------------------------------------\n");*/

}

static void
gooseListener1(GooseSubscriber subscriber, void* parameter)
{
    MmsValue* values1 = GooseSubscriber_getDataSetValues(subscriber);

    char buffer1[50];

    MmsValue_printToBuffer(values1, buffer1, 50);

    a = atof(&buffer1[1]);//21L1
    i = atof(&buffer1[11]);//21L9

    if (maximo1 == true){
        pMax_21l9 = i;
        maximo1 = false;
    }

    if (i > pMax_21l9){
        pMax_21l9 = i;
    }

    //printf("\n%f\n%f\n", a, i);
    uint64_t y = Hal_getTimeInMs();

    /*printf("-------------------------------------------------------------------------------------------------------------\n");            
    printf("                               PRIMEIRA MENSAGEM GOOSE ASSINADA VIED 2                                       \n");
    printf("-------------------------------------------------------------------------------------------------------------\n");*/

}

static void
gooseListener2(GooseSubscriber subscriber, void* parameter)
{
    MmsValue* values2 = GooseSubscriber_getDataSetValues(subscriber);

    char buffer2[100];

    MmsValue_printToBuffer(values2, buffer2, 100);

    b1 = atof(&buffer2[1]);//21L2
    d1 = atof(&buffer2[11]);//21L4
    e1 = atof(&buffer2[21]);//21L5
    f1 = atof(&buffer2[31]);//21L6
    uint64_t y = Hal_getTimeInMs();

    /*printf("-------------------------------------------------------------------------------------------------------------\n");            
    printf("                               PRIMEIRA MENSAGEM GOOSE ASSINADA VIED 3                                       \n");
    printf("-------------------------------------------------------------------------------------------------------------\n");*/

}

static void
gooseListener3(GooseSubscriber subscriber, void* parameter)
{
    MmsValue* values3 = GooseSubscriber_getDataSetValues(subscriber);

    char buffer3[100];

    MmsValue_printToBuffer(values3, buffer3, 100);

    c2 = atof(&buffer3[1]);//21L3
    g2 = atof(&buffer3[11]);//21L7
    uint64_t y = Hal_getTimeInMs();

    /*printf("-------------------------------------------------------------------------------------------------------------\n");            
    printf("                               PRIMEIRA MENSAGEM GOOSE ASSINADA VIED 4                                       \n");
    printf("-------------------------------------------------------------------------------------------------------------\n");*/

}

static void
gooseListener4(GooseSubscriber subscriber, void* parameter)
{
    MmsValue* values = GooseSubscriber_getDataSetValues(subscriber);

    char buffer[50];
    
    MmsValue_printToBuffer(values, buffer, 50);

    comando_received_21l6_A = buffer[11];
    comando_received_21l6_F = buffer[16];

    float tensao;
    tensao = atof(&buffer[20]);

    system("clear");
    printf("\nTESTE NOVA MENSAGEM CONTROLE RELIGAMENTO\nCOMANDO 1: %d\nCOMANDO 2: %d\nTENSÂO V = %f\n", comando_received_21l6_A, comando_received_21l6_F, tensao);

    if ((d1 == 0)&&(tensao==0))
    {
        if (comando_received_21l6_A == 116)
        {
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, false);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind24_stVal, true);
            /*
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, false);
            */
        }
        if (comando_received_21l6_F == 116)
        {
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, false);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind24_stVal, false);
            /*
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, false);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, true);
            */
        }
    }

    uint64_t y = Hal_getTimeInMs();

    /*printf("-------------------------------------------------------------------------------------------------------------\n");            
    printf("                               PRIMEIRA MENSAGEM GOOSE ASSINADA VIED 2                                       \n");
    printf("-------------------------------------------------------------------------------------------------------------\n");*/

}

static void
gooseListener8(GooseSubscriber subscriber, void* parameter)
{
    MmsValue* values = GooseSubscriber_getDataSetValues(subscriber);

    char buffer[50];

    

    MmsValue_printToBuffer(values, buffer, 50);

    comando_received_21l7_A = buffer[11];
    comando_received_21l7_F = buffer[16];

    float tensao;
    tensao = atof(&buffer[23]);

    if ((g2 == 0)&&(tensao==0))
        if (comando_received_21l7_A == 116)
        {
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, false);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind25_stVal, true);
            /*
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, false);
            */
        }
        if (comando_received_21l7_F == 116)
        {
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, false);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind25_stVal, false);
            /*
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, false);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, true);
            */
        }
    /*system("clear");
    printf("\n%d\n%d\n", buffer[1], buffer[6]);*/
    
    uint64_t y = Hal_getTimeInMs();

    /*printf("-------------------------------------------------------------------------------------------------------------\n");            
    printf("                               PRIMEIRA MENSAGEM GOOSE ASSINADA VIED 2                                       \n");
    printf("-------------------------------------------------------------------------------------------------------------\n");*/

}

static void
gooseListener5(GooseSubscriber subscriber, void* parameter)
{
    MmsValue* values = GooseSubscriber_getDataSetValues(subscriber);

    char buffer[50];

    MmsValue_printToBuffer(values, buffer, 50);


    char b; char c; char d;

    b = buffer[1];
    c = buffer[6];
    d = buffer[11];
    uint64_t y = Hal_getTimeInMs();

    if(b == 116){
        IedServer_updateDbposValue(iedServer, IEDMODEL_PRO_BK1XCBR1_Pos_stVal, DBPOS_ON);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, false);
    }else{
        IedServer_updateDbposValue(iedServer, IEDMODEL_PRO_BK1XCBR1_Pos_stVal, DBPOS_INTERMEDIATE_STATE);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, false);
    }
    /*
    printf("-------------------------------------------------------------------------------------------------------------\n");            
    printf("                               PRIMEIRA MENSAGEM GOOSE ASSINADA VIED 3                                       \n");
    printf("-------------------------------------------------------------------------------------------------------------\n");
    */

}

void self_h()
{
    while (1)
    {
        float x, y1;

        uint64_t y = Hal_getTimeInMs();
        /*
        system("clear");
        printf("\n%f\n",a);21l1
        printf("%f\n",b1);21l2
        printf("%f\n",c2);21l3
        printf("%f\n",d1);21l4
        printf("%f\n",e1);21l5
        printf("%f\n",f1);21l6
        printf("%f\n",g2);21l7
        printf("%f\n",228.00);21l8
        printf("%f\n",i);21l9
        */

        // DETECÇÃO DO TRECHO EM FALTA PARA ENNCONTRO 21L8
        if ((tensao_primarioA == 0) && (trip_21l9 == 102) && (estado_dj_21l9 == 0))
        {
            if (delay_1 == true)
            {
                gettimeofday(&delay1p, NULL);
                delay_1 = false;
            }
            gettimeofday(&delay1, NULL);
            time_delay1 = (float)(delay1.tv_sec - delay1p.tv_sec);
            time_delay1 += (delay1.tv_usec - delay1p.tv_usec) / (float)MICRO_PER_SECOND;
            if ((time_delay1) >= 1)
            {
                system("clear");
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind26_stVal, true);
                IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind26_t, y);
                printf("\n---------------------");
                printf("\n----T1 em Falta------");
                printf("\n---------------------");
                // Análise do Trecho em Recomposição
                x = (c2 + pMax_21l9) / pMaxS21l3;
                y = (b1 + pMax_21l9 + g2) / pMaxS21l2;
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind19_stVal, true);
                IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind19_t, y);
                if (delay_2 == true)
                {
                    gettimeofday(&delay2p, NULL);
                    delay_2 = false;
                }
                gettimeofday(&delay2, NULL);
                time_delay2 = (float)(delay2.tv_sec - delay2p.tv_sec);
                time_delay2 += (delay2.tv_usec - delay2p.tv_usec) / (float)MICRO_PER_SECOND;
                if ((time_delay2) >= 1)
                {
                    if (x > y1)
                    {
                        printf("\n---------------------------");
                        printf("\n----Reeligar por 21L7------");
                        printf("\n---------------------------");
                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind16_stVal, true);
                        IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind16_t, y);
                        if (delay_3 == true)
                        {
                            gettimeofday(&delay3p, NULL);
                            delay_3 = false;
                        }
                        gettimeofday(&delay3, NULL);
                        time_delay3 = (float)(delay3.tv_sec - delay3p.tv_sec);
                        time_delay3 += (delay3.tv_usec - delay3p.tv_usec) / (float)MICRO_PER_SECOND;
                        if ((time_delay3) >= 1)
                        {
                            if ((estado_dj_21l6 == 10) && (estado_dj_21l7 == 0))
                            {
                                printf("\n----Abrir 21L6------");
                                // ABRIR 21L6
                                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind27_stVal, true);
                                IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind27_t, y);
                            }
                            if (delay_4 == true)
                            {
                                gettimeofday(&delay4p, NULL);
                                delay_4 = false;
                            }
                            gettimeofday(&delay4, NULL);
                            time_delay4 = (float)(delay4.tv_sec - delay4p.tv_sec);
                            time_delay4 += (delay4.tv_usec - delay4p.tv_usec) / (float)MICRO_PER_SECOND;
                            if ((time_delay4) >= 1)
                            {
                                if ((estado_dj_21l6 == 0) && (estado_dj_21l7 == 0))
                                {
                                    printf("\n----Fechar 21L7------");
                                    // FECHAR 21L7
                                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind30_stVal, true);
                                    IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind28_t, y);
                                }
                                if (delay_5 == true)
                                {
                                    gettimeofday(&delay5p, NULL);
                                    delay_5 = false;
                                }
                                gettimeofday(&delay5, NULL);
                                time_delay5 = (float)(delay5.tv_sec - delay5p.tv_sec);
                                time_delay5 += (delay5.tv_usec - delay5p.tv_usec) / (float)MICRO_PER_SECOND;
                                if ((time_delay5) >= 1)
                                {
                                    if ((estado_dj_21l6 == 0) && (estado_dj_21l7 == 10))
                                    {
                                        printf("\n----Fechar 21L8------");
                                        // FECHAR 21L8
                                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, true);
                                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, false);
                                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind17_stVal, true);
                                        IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind17_t, y);
                                    }
                                }
                            }
                        }
                    }

                    if (y1 > x)
                    {
                        printf("\n---------------------------");
                        printf("\n----Reeligar por 21L6------");
                        printf("\n---------------------------");
                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind15_stVal, false);
                        IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind15_t, y);
                        if (delay_6 == true)
                        {
                            gettimeofday(&delay6p, NULL);
                            delay_6 = false;
                        }
                        gettimeofday(&delay6, NULL);
                        time_delay6 = (float)(delay6.tv_sec - delay6p.tv_sec);
                        time_delay6 += (delay6.tv_usec - delay6p.tv_usec) / (float)MICRO_PER_SECOND;
                        if ((time_delay6) >= 1)
                        {
                            if ((estado_dj_21l6 == 0) && (estado_dj_21l7 == 10))
                            {
                                printf("\n----Abrir 21L7------");
                                // ABRIR 21L7
                                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind29_stVal, true);
                                IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind28_t, y);
                            }
                            if (delay_7 == true)
                            {
                                gettimeofday(&delay7p, NULL);
                                delay_7 = false;
                            }
                            gettimeofday(&delay7, NULL);
                            time_delay7 = (float)(delay7.tv_sec - delay7p.tv_sec);
                            time_delay7 += (delay7.tv_usec - delay7p.tv_usec) / (float)MICRO_PER_SECOND;
                            if ((time_delay7) >= 1)
                            {
                                if ((estado_dj_21l6 == 0) && (estado_dj_21l7 == 0))
                                {
                                    printf("\n----Fechar 21L6------");
                                    // FECHAR 21L6
                                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind28_stVal, true);
                                    IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind27_t, y);
                                }
                                if (delay_8 == true)
                                {
                                    gettimeofday(&delay8p, NULL);
                                    delay_8 = false;
                                }
                                gettimeofday(&delay8, NULL);
                                time_delay8 = (float)(delay8.tv_sec - delay8p.tv_sec);
                                time_delay8 += (delay8.tv_usec - delay8p.tv_usec) / (float)MICRO_PER_SECOND;
                                if ((time_delay8) >= 1)
                                {
                                    if ((estado_dj_21l6 == 10) && (estado_dj_21l7 == 0))
                                    {
                                        printf("\n----Fechar 21L8------");
                                        // FECHAR 21L8
                                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, true);
                                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, false);
                                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind17_stVal, true);
                                        IedServer_updateUTCTimeAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind17_t, y);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        /*// DETECÇÃO DO TRECHO EM FALTA PARA ENNCONTRO 21L8
        if ((tensao_primarioA == 0) && (trip_21l9 == 102) && (estado_dj_21l9 == 0))
        {
            system("clear");
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind26_stVal, true);
            printf("\n---------------------");
            printf("\n----T1 em Falta------");
            printf("\n---------------------");
            // Análise do Trecho em Recomposição
            x = (c2 + pMax_21l9) / pMaxS21l3;
            y = (b1 + pMax_21l9 + g2) / pMaxS21l2;
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind19_stVal, true);

            // RELIGAR POR 21L7
            if (x > y)
            {
                printf("\n---------------------------");
                printf("\n----Reeligar por 21L7------");
                printf("\n---------------------------");
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind25_stVal, true);
                if ((estado_dj_21l6 == 10) && (estado_dj_21l7 == 0))
                {
                    // ABRIR 21L6
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind27_stVal, true);
                }
                if ((estado_dj_21l6 == 0) && (estado_dj_21l7 == 0))
                {
                    // FECHAR 21L7
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind28_stVal, false);
                }
                if ((estado_dj_21l6 == 0) && (estado_dj_21l7 == 10))
                {
                    // FECHAR 21L8
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, true);
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, false);
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind17_stVal, true);
                }
            }

            // RELIGAR POR 21L6
            if (y > x)
            {
                printf("\n---------------------------");
                printf("\n----Reeligar por 21L6------");
                printf("\n---------------------------");
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind24_stVal, true);
                if ((estado_dj_21l6 == 0) && (estado_dj_21l7 == 10))
                {
                    // ABRIR 21L7
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind28_stVal, true);
                }
                if ((estado_dj_21l6 == 0) && (estado_dj_21l7 == 0))
                {
                    // FECHAR 21L6
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind27_stVal, false);
                }
                if ((estado_dj_21l6 == 10) && (estado_dj_21l7 == 0))
                {
                    // FECHAR 21L8
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, true);
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, false);
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind17_stVal, true);
                }
            }
        }*/
        Thread_sleep(17);
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

    FILE *file;
    file = fopen("Ajustes_50.txt", "r");
    fscanf(file, "%f\n", &pick_up_50);
    fclose(file);

    FILE *file1;
    file1 = fopen("Ajustes_50N.txt", "r");
    fscanf(file1, "%f\n", &pick_up_50N);
    fclose(file1);

    FILE *file2;
    file2 = fopen("Ajustes_51.txt", "r");
    fscanf(file2, "%f\n", &pick_up_51);
    fscanf(file2, "%f\n", &K_51);
    fscanf(file2, "%f\n", &alfa_51);
    fscanf(file2, "%f\n", &dial_51);
    fclose(file2);

    FILE *file3;
    file3 = fopen("Ajustes_51V.txt", "r");
    fscanf(file3, "%f\n", &pick_up_51V);
    fscanf(file3,"%f\n", &tensao_51V);
    fscanf(file3, "%f\n", &K_51V);
    fscanf(file3, "%f\n", &alfa_51V);
    fscanf(file3, "%f\n", &dial_51V);
    fclose(file3);

    FILE *file4;
    file4 = fopen("Ajustes_51N.txt", "r");
    fscanf(file4, "%f\n", &pick_up_51N);
    fscanf(file4, "%f\n", &K_51N);
    fscanf(file4, "%f\n", &alfa_51N);
    fscanf(file4, "%f\n", &dial_51N);
    fclose(file4);

    FILE *file5;
    file5 = fopen("Ajustes_67.txt", "r");
    fscanf(file5, "%f\n", &pick_up_67);
    fscanf(file5, "%f\n", &K_67);
    fscanf(file5, "%f\n", &alfa_67);
    fscanf(file5, "%f\n", &dial_67);
    fscanf(file5, "%f\n", &atm_67);
    fclose(file5);

    FILE *file6;
    file6 = fopen("Ajustes_67N.txt", "r");
    fscanf(file6, "%f\n", &pick_up_67N);
    fscanf(file6, "%f\n", &K_67N);
    fscanf(file6, "%f\n", &alfa_67N);
    fscanf(file6, "%f\n", &dial_67N);
    fscanf(file6, "%f\n", &atm_67N);
    fclose(file6);

    /*Preparando o código para receber mensagens SV*/

    SVSubscriber subscriberSV = SVSubscriber_create(NULL, 0x4000);
    SVSubscriber_setListener(subscriberSV, svUpdateListener, NULL);
    SVReceiver_addSubscriber(receiverSV, subscriberSV);


    //Habilitando a publicação de mensagens GOOSE
    IedServer_enableGoosePublishing(iedServer);

    //Recepção e Assinatura de mensagens GOOSE
    GooseReceiver receiver = GooseReceiver_create();
    GooseReceiver_setInterfaceId(receiver, "eth0");
    GooseSubscriber subscriber6 = GooseSubscriber_create("VIED_21L6CFG/LLN0$GO$GOOSE_STATUS", NULL); 
    GooseSubscriber subscriber7 = GooseSubscriber_create("VIED_21L7CFG/LLN0$GO$GOOSE_STATUS", NULL); 
    GooseSubscriber subscriber  = GooseSubscriber_create("VIED_21L9CFG/LLN0$GO$GOOSE_STATUS", NULL); //Especificação de quem o ied irá receber as mensagens goose
    GooseSubscriber subscriber1 = GooseSubscriber_create("VIED_21L9CFG/LLN0$GO$GOOSE_POWER", NULL); //Especificação de quem o ied irá receber as mensagens goose
    GooseSubscriber subscriber2 = GooseSubscriber_create("VIED_21L6CFG/LLN0$GO$GOOSE_POWER", NULL); //Especificação de quem o ied irá receber as mensagens goose
    GooseSubscriber subscriber3 = GooseSubscriber_create("VIED_21L7CFG/LLN0$GO$GOOSE_POWER", NULL); //Especificação de quem o ied irá receber as mensagens goose
    GooseSubscriber subscriber4 = GooseSubscriber_create("VIED_21L6CFG/LLN0$GO$FEEDER_MEETING", NULL); //Especificação de quem o ied irá receber as mensagens goose
    GooseSubscriber subscriber8 = GooseSubscriber_create("VIED_21L7CFG/LLN0$GO$FEEDER_MEETING", NULL);
    GooseSubscriber subscriber5 = GooseSubscriber_create("MUBinIO_BinaryInputs/LLN0$GO$VMU_08_GOOSE", NULL); //Especificação de quem o ied irá receber as mensagens goose
    GooseSubscriber_setListener(subscriber, gooseListener, iedServer);
    GooseSubscriber_setListener(subscriber1, gooseListener1, iedServer);
    GooseSubscriber_setListener(subscriber2, gooseListener2, iedServer);
    GooseSubscriber_setListener(subscriber3, gooseListener3, iedServer);
    GooseSubscriber_setListener(subscriber4, gooseListener4, iedServer);
    GooseSubscriber_setListener(subscriber8, gooseListener8, iedServer);
    GooseSubscriber_setListener(subscriber5, gooseListener5, iedServer);
    GooseSubscriber_setListener(subscriber6, gooseListener6, iedServer);
    GooseSubscriber_setListener(subscriber7, gooseListener7, iedServer);
    GooseReceiver_addSubscriber(receiver, subscriber);
    GooseReceiver_addSubscriber(receiver, subscriber1);
    GooseReceiver_addSubscriber(receiver, subscriber2);
    GooseReceiver_addSubscriber(receiver, subscriber3);
    GooseReceiver_addSubscriber(receiver, subscriber4);
    GooseReceiver_addSubscriber(receiver, subscriber8);
    GooseReceiver_addSubscriber(receiver, subscriber5);
    GooseReceiver_addSubscriber(receiver, subscriber6);
    GooseReceiver_addSubscriber(receiver, subscriber7);

    //Ligação dos Servidores
    GooseReceiver_start(receiver);
    SVReceiver_start(receiverSV);

    //Ligação dos Servidores para cada função de proteção
    if (pick_up_50 > 0)
    {
        thread_50 = Thread_create((ThreadExecutionFunction)funcao_50, (void *) thread_50, false);
        Thread_start(thread_50);
    }
    if (pick_up_50N > 0)
    {
        thread_50N = Thread_create((ThreadExecutionFunction)funcao_50N, (void *) thread_50N, false);
        Thread_start(thread_50N);
    }
    if (pick_up_51 > 0)
    {
        thread_51 = Thread_create((ThreadExecutionFunction)funcao_51, (void *) thread_51, false);
        Thread_start(thread_51);
    }
    if (pick_up_51V > 0)
    {
        thread_51V = Thread_create((ThreadExecutionFunction)funcao_51V, (void *) thread_51V, false);
        Thread_start(thread_51V);
    }
    if (pick_up_51N > 0)
    {
        thread_51N = Thread_create((ThreadExecutionFunction)funcao_51N, (void *) thread_51N, false);
        Thread_start(thread_51N);
    }
    if (pick_up_67 > 0)
    {
        thread_67 = Thread_create((ThreadExecutionFunction)funcao_67, (void *) thread_67, false);
        Thread_start(thread_67);
    }
    if (pick_up_67N > 0)
    {
        thread_67N = Thread_create((ThreadExecutionFunction)funcao_67N, (void *) thread_67N, false);
        Thread_start(thread_67N);
    }

    self_healing = Thread_create((ThreadExecutionFunction)self_h, (void *) self_healing, false);
    Thread_start(self_healing);

    IedServer_setGoCBHandler(iedServer, goCbEventHandler, NULL);

    /* MMS server will be instructed to start listening to client connections. */
    IedServer_start(iedServer, 102);

    IedServer_setControlHandler(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01, (ControlHandler) controlHandlerForBinaryOutput,
    IEDMODEL_CON_RBGGIO1_SPCSO01);

    IedServer_setControlHandler(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02, (ControlHandler) controlHandlerForBinaryOutput,
    IEDMODEL_CON_RBGGIO1_SPCSO02);

    IedServer_setControlHandler(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO03, (ControlHandler) controlHandlerForBinaryOutput,
    IEDMODEL_CON_RBGGIO1_SPCSO03);

    IedServer_setControlHandler(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO04, (ControlHandler) controlHandlerForBinaryOutput,
    IEDMODEL_CON_RBGGIO1_SPCSO04);

    IedServer_setControlHandler(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO05, (ControlHandler) controlHandlerForBinaryOutput,
    IEDMODEL_CON_RBGGIO1_SPCSO05);

    IedServer_setControlHandler(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO06, (ControlHandler) controlHandlerForBinaryOutput,
    IEDMODEL_CON_RBGGIO1_SPCSO06);

    IedServer_setControlHandler(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO07, (ControlHandler) controlHandlerForBinaryOutput,
    IEDMODEL_CON_RBGGIO1_SPCSO07);

    IedServer_setControlHandler(iedServer, IEDMODEL_PRO_BKR1CSWI1_Pos, (ControlHandler) controlHandlerForBinaryOutput, IEDMODEL_PRO_BKR1CSWI1_Pos);

    if (!IedServer_isRunning(iedServer)) {
        printf("Starting server failed! Exit.\n");
        IedServer_destroy(iedServer);
        exit(-1);
    }

    running = 1;

    signal(SIGINT, sigint_handler);

    float teste4;

    while (running) {

        teste4 = corrente_primarioA;
        IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_ANN_MVGGIO12_AnIn01_mag_f, a);
        IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_ANN_MVGGIO12_AnIn08_mag_f, corrente_primarioA);
        IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_ANN_MVGGIO12_AnIn09_mag_f, i);

        if (maximo2 == true){
        pMax_21l8 = teste4;
        maximo2 = false;
        }

        if (teste4 > pMax_21l8){
        pMax_21l8 = teste4;
        }

        if ((IedServer_getBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general)) == true){
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO01_stVal, false);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_CON_RBGGIO1_SPCSO02_stVal, true);
        }

        
        /*
        system("clear");
        printf("\n%f\n",a);//21l1
        printf("%f\n",b1);//21l2
        printf("%f\n",c2);//21l3
        printf("%f\n",d1);//21l4
        printf("%f\n",e1);//21l5
        printf("%f\n",f1);//21l6
        printf("%f\n",g2);//21l7
        printf("%f\n",corrente_primarioA);//21l8
        printf("%f\n",i);//21l9*/
        
        Thread_sleep(17);

    }

    /* stop MMS server - close TCP server socket and all client sockets */
        IedServer_stop(iedServer);

        /* Cleanup - free all resources */
        SVReceiver_destroy(receiverSV);
        IedServer_destroy(iedServer);


    return 0;
} /* main() */