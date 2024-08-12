/*
 * sv_publisher_example.c
 *
 * Example program for Sampled Values (SV) publisher
 */

#include <signal.h>
#include <stdio.h>
#include "hal_thread.h"
#include "sv_publisher.h"
#include "math.h"


static bool running = true;

void sigint_handler(int signalId)
{
    running = 0;
}

int
main(int argc, char** argv)
{
    float r_transformacao = 2.8868;
    char* interface;
  
    if (argc > 1)
        interface = argv[1];
    else
        interface = "eth1";
  
    printf("Using interface %s\n", interface);

    signal(SIGINT, sigint_handler);

    SVPublisher svPublisher = SVPublisher_create(NULL, interface);

    if (svPublisher) {

        /* Create first ASDU and add data points */

        SVPublisher_ASDU asdu1 = SVPublisher_addASDU(svPublisher, "IEDNameMU0201", NULL, 1);

        /*float float1 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float11 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float2 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float21 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float3 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float31 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float4 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float41 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float5 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float51 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float6 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float61 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float7 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float71 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float8 = SVPublisher_ASDU_addFLOAT(asdu1);
        float float81 = SVPublisher_ASDU_addFLOAT(asdu1);*/

        /*int float1 = SVPublisher_ASDU_addINT32(asdu1);
        int float11 = SVPublisher_ASDU_addINT32(asdu1);
        int float2 = SVPublisher_ASDU_addINT32(asdu1);
        int float21 = SVPublisher_ASDU_addINT32(asdu1);
        int float3 = SVPublisher_ASDU_addINT32(asdu1);
        int float31 = SVPublisher_ASDU_addINT32(asdu1);
        int float4 = SVPublisher_ASDU_addINT32(asdu1);
        int float41 = SVPublisher_ASDU_addINT32(asdu1);
        int float5 = SVPublisher_ASDU_addINT32(asdu1);
        int float51 = SVPublisher_ASDU_addINT32(asdu1);
        int float6 = SVPublisher_ASDU_addINT32(asdu1);
        int float61 = SVPublisher_ASDU_addINT32(asdu1);
        int float7 = SVPublisher_ASDU_addINT32(asdu1);
        int float71 = SVPublisher_ASDU_addINT32(asdu1);
        int float8 = SVPublisher_ASDU_addINT32(asdu1);
        int float81 = SVPublisher_ASDU_addINT32(asdu1);*/

        int float1 = SVPublisher_ASDU_addINT64(asdu1);
        //int float11 = SVPublisher_ASDU_addINT64(asdu1);
        int float2 = SVPublisher_ASDU_addINT64(asdu1);
        //int float21 = SVPublisher_ASDU_addINT64(asdu1);
        int float3 = SVPublisher_ASDU_addINT64(asdu1);
        //int float31 = SVPublisher_ASDU_addINT64(asdu1);
        int float7 = SVPublisher_ASDU_addINT64(asdu1);
        //int float71 = SVPublisher_ASDU_addINT64(asdu1);
        int float4 = SVPublisher_ASDU_addINT64(asdu1);
        //int float41 = SVPublisher_ASDU_addINT64(asdu1);
        int float5 = SVPublisher_ASDU_addINT64(asdu1);
        //int float51 = SVPublisher_ASDU_addINT64(asdu1);
        int float6 = SVPublisher_ASDU_addINT64(asdu1);
        //int float61 = SVPublisher_ASDU_addINT64(asdu1);
        int float8 = SVPublisher_ASDU_addINT64(asdu1);
        //int float81 = SVPublisher_ASDU_addINT64(asdu1);
        

        int ts1 = SVPublisher_ASDU_addTimestamp(asdu1);

        SVPublisher_setupComplete(svPublisher);

        float fVal1 = 0;
        float fVal2 = 0;
        float fVal3 = 0;
        float fVal4 = 0;
        float fVal5 = 0;
        float fVal6 = 0;
        float fVal7 = 0;
        float fVal8 = 0;
        float freq = 60.00;
        float vpico = 1697.06;//falta com BF
        //float vpico = 1272.8;//falta sem BF
        //float vpico = 800;//pré-falta
        float tpico = 97580.7358;
        float tempo;
        double x;

        while (running) {
            Timestamp ts;
            Timestamp_clearFlags(&ts);
            Timestamp_setTimeInMilliseconds(&ts, Hal_getTimeInMs());

            /* update the values in the SV ASDUs */
            

            /*SVPublisher_ASDU_setFLOAT(asdu1, float1, fVal1);
            SVPublisher_ASDU_setFLOAT(asdu1, float11, fVal6);
            SVPublisher_ASDU_setFLOAT(asdu1, float2, fVal2);
            SVPublisher_ASDU_setFLOAT(asdu1, float21, fVal6);
            SVPublisher_ASDU_setFLOAT(asdu1, float3, fVal3);
            SVPublisher_ASDU_setFLOAT(asdu1, float31, fVal6);
            SVPublisher_ASDU_setFLOAT(asdu1, float7, fVal6);
            SVPublisher_ASDU_setFLOAT(asdu1, float71, fVal6);
            SVPublisher_ASDU_setFLOAT(asdu1, float4, fVal4);
            SVPublisher_ASDU_setFLOAT(asdu1, float41, fVal6);
            SVPublisher_ASDU_setFLOAT(asdu1, float5, fVal5);
            SVPublisher_ASDU_setFLOAT(asdu1, float51, fVal6);
            SVPublisher_ASDU_setFLOAT(asdu1, float6, fVal6);
            SVPublisher_ASDU_setFLOAT(asdu1, float61, fVal6);
            SVPublisher_ASDU_setFLOAT(asdu1, float8, fVal6);
            SVPublisher_ASDU_setFLOAT(asdu1, float81, fVal6);*/
            
            /*SVPublisher_ASDU_setINT32(asdu1, float1, fVal1);
            SVPublisher_ASDU_setINT32(asdu1, float11, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float2, fVal2);
            SVPublisher_ASDU_setINT32(asdu1, float21, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float3, fVal3);
            SVPublisher_ASDU_setINT32(asdu1, float31, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float7, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float71, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float4, fVal4);
            SVPublisher_ASDU_setINT32(asdu1, float41, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float5, fVal5);
            SVPublisher_ASDU_setINT32(asdu1, float51, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float6, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float61, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float8, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float81, fVal6);*/

            SVPublisher_ASDU_setINT32(asdu1, float1, fVal1);
            //SVPublisher_ASDU_setINT64(asdu1, float11, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float2, fVal2);
            //SVPublisher_ASDU_setINT64(asdu1, float21, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float3, fVal3);
            //SVPublisher_ASDU_setINT64(asdu1, float31, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float7, fVal7);
            //SVPublisher_ASDU_setINT64(asdu1, float71, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float4, fVal4);
            //SVPublisher_ASDU_setINT64(asdu1, float41, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float5, fVal5);
            //SVPublisher_ASDU_setINT64(asdu1, float51, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float6, fVal6);
            //SVPublisher_ASDU_setINT64(asdu1, float61, fVal6);
            SVPublisher_ASDU_setINT32(asdu1, float8, fVal8);
            //SVPublisher_ASDU_setINT64(asdu1, float81, fVal6);

            SVPublisher_ASDU_setTimestamp(asdu1, ts1, ts);

            

            /* update the sample counters */

            SVPublisher_ASDU_increaseSmpCnt(asdu1);

            //fVal1 += 1.1f;
            x = 2*M_PI*freq*tempo;
            tempo = (tempo + 0.0002083);

            fVal1 = vpico*sin(x)*1000;
            fVal2 = vpico*sin(x + 4.18879)*1000;
            fVal3 = vpico*sin(x + 2.0944)*1000;  
            /*fVal1 = vpico*sin(x+1.222)*1000;
            fVal2 = vpico*sin(x+1.222)*1000;
            fVal3 = vpico*sin(x+1.222)*1000;*/
            fVal4 = tpico*sin(x)*100;
            fVal5 = tpico*sin(x+4.18879)*100;
            fVal6 = tpico*sin(x+2.0944)*100;  

            /*fVal1 = vpico*sin(x)*1000;
            fVal2 = 0;
            fVal3 = 0;
            fVal4 = tpico*sin(x)*100;
            fVal5 = 0;
            fVal6 = 0;*/

            fVal7 = vpico*sin(x)*1000;
            fVal8 = tpico*sin(x)*100;
            

            /* send the SV message */
            SVPublisher_publish(svPublisher);
            //printf("%.6f\n", vpico*sin(x));
            /*printf("--------CORRENTES DE LINHA LADO DE ALTA--------\n");
            printf("Corrente da linha A %.6f\n", fVal1);
            printf("Corrente da linha B %.6f\n", fVal2);
            printf("Corrente da linha C %.6f\n", fVal3);
            printf("--------TENSÕES DE LINHA LADO DE BAIXA-------\n");
            printf("Tensão da linha A  %.6f\n", fVal4);
            printf("Tensão da linha B  %.6f\n", fVal5);
            printf("Tensão da linha C  %.6f\n", fVal6);*/
            /*
             * For real applications this sleep time has to be adjusted to match the SV sample rate!
             * Platform specific functions like usleep or timer interrupt service routines have to be used instead
             * to realize the required time accuracy for sending messages.
             */
            Thread_sleep(0.20833);
        }

        SVPublisher_destroy(svPublisher);
    }
    else {
        printf("Failed to create SV publisher\n");
    }
    return 0;
}
