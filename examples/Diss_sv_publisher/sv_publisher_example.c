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
        interface = "eth0";
  
    printf("Using interface %s\n", interface);

    signal(SIGINT, sigint_handler);

    SVPublisher svPublisher = SVPublisher_create(NULL, interface);

    if (svPublisher) {

        /* Create first ASDU and add data points */

        SVPublisher_ASDU asdu1 = SVPublisher_addASDU(svPublisher, "svpub1", NULL, 1);

        int float1 = SVPublisher_ASDU_addFLOAT(asdu1);
        int float2 = SVPublisher_ASDU_addFLOAT(asdu1);
        int float3 = SVPublisher_ASDU_addFLOAT(asdu1);
        int ts1 = SVPublisher_ASDU_addTimestamp(asdu1);

        /* Create second ASDU and add data points */

        SVPublisher_ASDU asdu2 = SVPublisher_addASDU(svPublisher, "svpub2", NULL, 1);

        int float4 = SVPublisher_ASDU_addFLOAT(asdu2);
        int float5 = SVPublisher_ASDU_addFLOAT(asdu2);
        int float6 = SVPublisher_ASDU_addFLOAT(asdu2);
        int ts2 = SVPublisher_ASDU_addTimestamp(asdu2);

        SVPublisher_setupComplete(svPublisher);

        float fVal1 = 0;
        float fVal2 = 0;
        float fVal3 = 0;
        float fVal4 = 0;
        float fVal5 = 0;
        float fVal6 = 0;
        float freq = 60.00f;
        float vpico = 141.4214f;
        float tempo;
        double x;

        while (running) {
            Timestamp ts;
            Timestamp_clearFlags(&ts);
            Timestamp_setTimeInMilliseconds(&ts, Hal_getTimeInMs());

            /* update the values in the SV ASDUs */
            

            SVPublisher_ASDU_setFLOAT(asdu1, float1, fVal1);
            SVPublisher_ASDU_setFLOAT(asdu1, float2, fVal2);
            SVPublisher_ASDU_setFLOAT(asdu1, float3, fVal3);
            SVPublisher_ASDU_setTimestamp(asdu1, ts1, ts);

            SVPublisher_ASDU_setFLOAT(asdu2, float4, fVal4);
            SVPublisher_ASDU_setFLOAT(asdu2, float5, fVal5);
            SVPublisher_ASDU_setFLOAT(asdu2, float6, fVal6);
            SVPublisher_ASDU_setTimestamp(asdu2, ts2, ts);

            /* update the sample counters */

            SVPublisher_ASDU_increaseSmpCnt(asdu1);
            SVPublisher_ASDU_increaseSmpCnt(asdu2);

            //fVal1 += 1.1f;
            x = 2*M_PI*freq*tempo;
            tempo = (tempo + 0.0002083);

            fVal1 = 1*vpico*sin(x);
            fVal2 = vpico*sin(x + 4.18879);
            fVal3 = vpico*sin(x + 2.0944);
            fVal4 = 1.35*r_transformacao*vpico*sin(x);
            fVal5 = 1.35*r_transformacao*vpico*sin(x+4.18879);
            fVal6 = 1.35*r_transformacao*vpico*sin(x+2.0944);
            

            /* send the SV message */
            SVPublisher_publish(svPublisher);
            //printf("%.6f\n", vpico*sin(x));
            printf("--------CORRENTES DE LINHA LADO DE ALTA--------\n");
            printf("Corrente da linha A %.6f\n", fVal1);
            printf("Corrente da linha B %.6f\n", fVal2);
            printf("Corrente da linha C %.6f\n", fVal3);
            printf("--------CORRENTES DE LINHA LADO DE BAIXA-------\n");
            printf("Corrente da linha A  %.6f\n", fVal4);
            printf("Corrente da linha B  %.6f\n", fVal5);
            printf("Corrente da linha C  %.6f\n", fVal6);
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
