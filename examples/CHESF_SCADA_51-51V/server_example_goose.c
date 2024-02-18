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
float Vn = 0, atm = 0, j = 0;
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
static float pick_up;
static float M, M1, M2, K, a, t, t1, t2,T, B = 1;
struct timeval start_time, start1_time, start2_time;
struct timeval stop_time, stop1_time, stop2_time;
struct timeval tart_time, tart1_time, tart2_time;
struct timeval top_time, top1_time, top2_time;
static float ang1, ang2, ang3, ang4, ang5, ang6, ang7, ang8;
static float time_diff = 0, time_diff1 = 0, time_diff2 = 0;
static float ime_diff = 0, ime_diff1 = 0, ime_diff2 = 0;
double an[8], ar[6], br[6], teta, teta1, teta2, torque, torque1, torque2, tetaN, torqueN;
double complex yr[6], Vbc, aVbc, mVbc,Vca, aVca, mVca, Vab, aVab, mVab;


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
        printf("Parametrização de Função 51V\n");
        printf("Escolha uma das funções abaixo:\n- Função 51 [1]\n- Função 51V [2]\n");
        scanf("%d", &funcao);
        if (funcao == 2){
            printf("Defina a Tensão Nominal:\n");
            scanf("%f", &Vn);
        }
        printf("Escolha o dial:\n");
        scanf("%f", &T);
        printf("Defina a corrente de pick-up:\n");
        scanf("%f", &pick_up);
        printf("Escolha uma das curvas abaixo:\n- Curva Inversa [1]\n- Moderadamente Inversa [2]\n- Muito Inversa [3]\n- Extremamente Inversa [4]\n");
        scanf("%d", &curva);
        exit(0);
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
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind10_stVal, false);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind11_stVal, false);
    }

    if ((strcmp(svID,"IEDNameMU0101"))== 0){      
        SVrms_deltaA = (SVrms_deltaA + pow((SVSubscriber_ASDU_getINT32 (asdu, 0)*0.001),2));
        SVrms_deltaA1 = (SVrms_deltaA1 + pow((SVSubscriber_ASDU_getINT32 (asdu, 32)*0.01),2));
        SVrms_deltaB = (SVrms_deltaB + pow((SVSubscriber_ASDU_getINT32 (asdu, 8)*0.001),2));
        SVrms_deltaB1 = (SVrms_deltaB1 + pow((SVSubscriber_ASDU_getINT32 (asdu, 40)*0.01),2));
        SVrms_deltaC = (SVrms_deltaC + pow((SVSubscriber_ASDU_getINT32 (asdu, 16)*0.001),2));
        SVrms_deltaC1 = (SVrms_deltaC1 + pow((SVSubscriber_ASDU_getINT32 (asdu, 48)*0.01),2));

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

        if(funcao == 1){
            j = 1;
        }else{
            j = tensao_primarioA/Vn;
        }    
 
        M = corrente_primarioA/(j*pick_up);
        t=(T*(K/((pow(M,a))-B)));

        M1 = corrente_primarioB/(j*pick_up);
        t1=(T*(K/((pow(M1,a))-B)));

        M2 = corrente_primarioC/(j*pick_up);
        t2=(T*(K/((pow(M2,a))-B)));

        contadorSV3 ++;

        if ((t>0)&&(corrente_primarioA>(pick_up*50/100))){
                if (contador == true){
                    gettimeofday( &start_time, NULL );
                    contador = false;
                }
                gettimeofday( &stop_time, NULL );
                time_diff = (float)(stop_time.tv_sec - start_time.tv_sec);
                time_diff += (stop_time.tv_usec - start_time.tv_usec)/(float)MICRO_PER_SECOND;
                if ((time_diff)>=t){
                    
                    if(funcao == 1){
                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_ATPTOC20_Op_general, true);
                        printf("-------------------------------------------------------------------------------------------------------------\n");
                        printf("                              ATUAR FUNÇÃO 51: SOBRECORRENTE TEMPORIZADA FASE A                              \n");
                        printf("-------------------------------------------------------------------------------------------------------------\n");
                        
                    }else{
                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind01_stVal, true);
                        printf("-------------------------------------------------------------------------------------------------------------\n");
                        printf("                   ATUAR FUNÇÃO 51V: SOBRECORRENTE TEMPORIZADA COM RESTRIÇÃO DE TENSÃO FASE A                \n");
                        printf("-------------------------------------------------------------------------------------------------------------\n");
                    }    
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
                    if (contador9 == true){
                        gettimeofday( &tart_time, NULL );
                        contador9 = false;
                    }
                    gettimeofday( &top_time, NULL );
                    ime_diff = (float)(top_time.tv_sec - tart_time.tv_sec);
                    ime_diff += (top_time.tv_usec - tart_time.tv_usec)/(float)MICRO_PER_SECOND;
                    if ((ime_diff)>=0.250){
                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BFR1RBRF1_OpEx_general, true);
                    }
                }
        }
        if ((t1>0)&&(corrente_primarioB>(pick_up*50/100))){
                if (contador7 == true){
                    gettimeofday( &start1_time, NULL );
                    contador7 = false;
                }
                gettimeofday( &stop1_time, NULL );
                time_diff1 = (float)(stop1_time.tv_sec - start1_time.tv_sec);
                time_diff1 += (stop1_time.tv_usec - start1_time.tv_usec)/(float)MICRO_PER_SECOND;
                if ((time_diff1)>=t1){
                    if(funcao == 1){
                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BTPTOC21_Op_general, true);
                        printf("-------------------------------------------------------------------------------------------------------------\n");
                        printf("                              ATUAR FUNÇÃO 51: SOBRECORRENTE TEMPORIZADA FASE B                              \n");
                        printf("-------------------------------------------------------------------------------------------------------------\n");
                    }else{
                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind02_stVal, true);
                        printf("-------------------------------------------------------------------------------------------------------------\n");
                        printf("                   ATUAR FUNÇÃO 51V: SOBRECORRENTE TEMPORIZADA COM RESTRIÇÃO DE TENSÃO FASE B                \n");
                        printf("-------------------------------------------------------------------------------------------------------------\n");
                    }
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
                    if (contador10 == true){
                        gettimeofday( &tart1_time, NULL );
                        contador10 = false;
                    }
                    gettimeofday( &top1_time, NULL );
                    ime_diff1 = (float)(top1_time.tv_sec - tart1_time.tv_sec);
                    ime_diff1 += (top1_time.tv_usec - tart1_time.tv_usec)/(float)MICRO_PER_SECOND;
                    if ((ime_diff1)>=0.250){
                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BFR1RBRF1_OpEx_general, true);
                    }
                }
        }
        if ((t2>0)&&(corrente_primarioC>(pick_up*50/100))){
                if (contador8 == true){
                    gettimeofday( &start2_time, NULL );
                    contador8 = false;
                }
                gettimeofday( &stop2_time, NULL );
                time_diff2 = (float)(stop2_time.tv_sec - start2_time.tv_sec);
                time_diff2 += (stop2_time.tv_usec - start2_time.tv_usec)/(float)MICRO_PER_SECOND;
                if ((time_diff2)>=t2){
                    if(funcao == 1){
                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_CTPTOC22_Op_general, true);
                        printf("-------------------------------------------------------------------------------------------------------------\n");
                        printf("                              ATUAR FUNÇÃO 51: SOBRECORRENTE TEMPORIZADA FASE C                              \n");
                        printf("-------------------------------------------------------------------------------------------------------------\n");
                    }else{
                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind03_stVal, true);
                        printf("-------------------------------------------------------------------------------------------------------------\n");
                        printf("                   ATUAR FUNÇÃO 51V: SOBRECORRENTE TEMPORIZADA COM RESTRIÇÃO DE TENSÃO FASE C                \n");
                        printf("-------------------------------------------------------------------------------------------------------------\n");
                    }
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, true);
                    if (contador11 == true){
                        gettimeofday( &tart2_time, NULL );
                        contador11 = false;
                    }
                    gettimeofday( &top2_time, NULL );
                    ime_diff2 = (float)(top2_time.tv_sec - tart2_time.tv_sec);
                    ime_diff2 += (top2_time.tv_usec - tart2_time.tv_usec)/(float)MICRO_PER_SECOND;
                    if ((ime_diff2)>=0.250){
                        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BFR1RBRF1_OpEx_general, true);
                    }
                }
        }

        if ((corrente_primarioA<(pick_up*10/100))&&(corrente_primarioB<(pick_up*10/100))&&(corrente_primarioC<(pick_up*10/100))){
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_TRIPPTRC1_Tr_general, false);
                IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BFR1RBRF1_OpEx_general, false);
                if(funcao == 1){
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_ATPTOC20_Op_general, false);
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_BTPTOC21_Op_general, false);
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_PRO_CTPTOC22_Op_general, false);
                }else{
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind01_stVal, false);
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind02_stVal, false);
                    IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind03_stVal, false);
                }
                contador = true;
                contador7 = true;
                contador8 = true;
                contador9 = true;
                contador10 = true;
                contador11 = true;
        }

        


        if(contadorSV3 == 60){
            /*FILE *file;
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

            fclose(file);*/
             
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
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_A_phsA_instCVal_mag_f, corrente_primarioA);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_A_phsB_instCVal_mag_f, corrente_primarioB);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_A_phsC_instCVal_mag_f, corrente_primarioC);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_A_phsA_cVal_mag_f, corrente_primarioA);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_A_phsB_cVal_mag_f, corrente_primarioB);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_A_phsC_cVal_mag_f, corrente_primarioC);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_PPV_phsAB_cVal_mag_f, tensao_primarioA);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_PPV_phsBC_cVal_mag_f, tensao_primarioB);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_PPV_phsCA_cVal_mag_f, tensao_primarioC);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_PPV_phsAB_instCVal_ang_f, tensao_primarioA);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_PPV_phsBC_instCVal_ang_f, tensao_primarioB);
            IedServer_updateFloatAttributeValue(iedServer, IEDMODEL_MET_METMMXU1_PPV_phsCA_instCVal_ang_f, tensao_primarioC);
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
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind10_stVal, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind11_stVal, false);
        }
        else{
            printf("off\n");
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind11_stVal, true);
            IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind10_stVal, false);
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


    char b;

    b = buffer[1];


    if(b == 116){
        IedServer_updateDbposValue(iedServer, IEDMODEL_PRO_BK1XCBR1_Pos_stVal, DBPOS_OFF);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind11_stVal, false);
        
    }

    else{
        IedServer_updateDbposValue(iedServer, IEDMODEL_PRO_BK1XCBR1_Pos_stVal, DBPOS_ON);
        IedServer_updateBooleanAttributeValue(iedServer, IEDMODEL_ANN_SVGGIO3_Ind10_stVal, false);
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


    IedServer_enableGoosePublishing(iedServer);

    GooseReceiver receiver = GooseReceiver_create();

    GooseReceiver_setInterfaceId(receiver, "p5p1");
    
    GooseSubscriber subscriber = GooseSubscriber_create("MU320CTRL/LLN0$GO$FastGOOSE1", NULL); 

    GooseSubscriber_setListener(subscriber, gooseListener, iedServer); 

    GooseReceiver_addSubscriber(receiver, subscriber); 

    GooseReceiver_start(receiver);
    SVReceiver_start(receiverSV);

    //Thread_sleep(1000);

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

    /*GooseReceiver_start(receiver);
    SVReceiver_start(receiverSV);*/

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
