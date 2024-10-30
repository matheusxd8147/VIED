#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

int main()
{

    int funcoes, curva_51, curva_51V, curva_51N, curva_67, curva_67N;
    float pick_up_50 = 0, pick_up_50N = 0, pick_up_51 = 0, pick_up_51V = 0, pick_up_51N = 0, pick_up_67 = 0, pick_up_67N = 0, atm_67, atm_67N;
    float dial_51, dial_51V, dial_51N, dial_67, dial_67N, tensao_51V;

    while (1)
    {
        system("clear");
        printf("Funções de Proteção:");
        printf("\n[1] - Sobrecorrente Instantânea (ANSI 50);");
        printf("\n[2] - Sobrecorrente Instantânea de Neutro (ANSI 50N);");
        printf("\n[3] - Sobrecorrente Temporizada (ANSI 51);");
        printf("\n[4] - Sobrecorrente Temporizada (ANSI 51V);");
        printf("\n[5] - Sobrecorrente Temporizada de Neutro (ANSI 51N);");
        printf("\n[6] - Direcional de Sobrecorrente Temporizada (ANSI 67);");
        printf("\n[7] - Direcional de Sobrecorrente Temporizada de Neutro;");
        printf("\n[8] - Sair.\n");
        scanf("%d", &funcoes);

        if (funcoes == 1)
        {
            printf("\nAjustes Função 50:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_50);

            FILE *file;
            file = fopen("Ajustes_50.txt","w");
            fprintf(file,"%f\n",pick_up_50);
            fclose(file);
        }
        if (funcoes == 2)
        {
            printf("\nAjustes Função 50N:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_50N);

            FILE *file1;
            file1 = fopen("Ajustes_50N.txt","w");
            fprintf(file1,"%f\n",pick_up_50N);
            fclose(file1);
        }
        if (funcoes == 3)
        {
            printf("\nAjustes Função 51:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_51);
            printf("\nEscolha uma das curvas abaixo:\n[1] - Curva Inversa\n[2] - Moderadamente Inversa\n[3] - Muito Inversa\n[4] - Extremamente Inversa\n");
            scanf("%d",&curva_51);
            printf("\nDial: ");
            scanf("%f",&dial_51);

            FILE *file2;
            file2 = fopen("Ajustes_51.txt","w");
            fprintf(file2,"%f\n",pick_up_51);
            fprintf(file2,"%d\n",curva_51);
            fprintf(file2,"%f\n",dial_51);
            fclose(file2);
        }
        if (funcoes == 4)
        {
            printf("\nAjustes Função 51V:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_51V);
            printf("\nTensão Norminal: ");
            scanf("%f",&tensao_51V);
            printf("\nEscolha uma das curvas abaixo:\n[1] - Curva Inversa\n[2] - Moderadamente Inversa\n[3] - Muito Inversa\n[4] - Extremamente Inversa\n");
            scanf("%d",&curva_51V);
            printf("\nDial: ");
            scanf("%f",&dial_51V);

            FILE *file3;
            file3 = fopen("Ajustes_51V.txt","w");
            fprintf(file3,"%f\n",pick_up_51V);
            fprintf(file3,"%d\n",curva_51V);
            fprintf(file3,"%f\n",dial_51V);
            fclose(file3);
        }
        if (funcoes == 5)
        {
            printf("\nAjustes Função 51N:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_51N);
            printf("\nEscolha uma das curvas abaixo:\n[1] - Curva Inversa\n[2] - Moderadamente Inversa\n[3] - Muito Inversa\n[4] - Extremamente Inversa\n");
            scanf("%d",&curva_51N);
            printf("\nDial: ");
            scanf("%f",&dial_51N);

            FILE *file4;
            file4 = fopen("Ajustes_51N.txt","w");
            fprintf(file4,"%f\n",pick_up_51N);
            fprintf(file4,"%d\n",curva_51N);
            fprintf(file4,"%f\n",dial_51N);
            fclose(file4);
        }
        if (funcoes == 6)
        {
            printf("\nAjustes Função 67:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_67);
            printf("\nEscolha uma das curvas abaixo:\n[1] - Curva Inversa\n[2] - Moderadamente Inversa\n[3] - Muito Inversa\n[4] - Extremamente Inversa\n");
            scanf("%d",&curva_67);
            printf("\nDial: ");
            scanf("%f",&dial_67);
            printf("\nÂngulo de Torque Máximo:\n");
            scanf("%f",&atm_67);

            FILE *file5;
            file5 = fopen("Ajustes_67.txt","w");
            fprintf(file5,"%f\n",pick_up_67);
            fprintf(file5,"%d\n",curva_67);
            fprintf(file5,"%f\n",dial_67);
            fprintf(file5,"%f\n",atm_67);
            fclose(file5);
        }
        if (funcoes == 7)
        {
            printf("\nAjustes Função 67N:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_67N);
            printf("\nEscolha uma das curvas abaixo:\n[1] - Curva Inversa\n[2] - Moderadamente Inversa\n[3] - Muito Inversa\n[4] - Extremamente Inversa\n");
            scanf("%d",&curva_67N);
            printf("\nDial: ");
            scanf("%f",&dial_67N);
            printf("\nÂngulo de Torque Máximo:\n");
            scanf("%f",&atm_67);

            FILE *file6;
            file6 = fopen("Ajustes_67N.txt","w");
            fprintf(file6,"%f\n",pick_up_67N);
            fprintf(file6,"%d\n",curva_67N);
            fprintf(file6,"%f\n",dial_67N);
            fprintf(file6,"%f\n",atm_67N);
            fclose(file6);
        }
        if (funcoes == 8)
        {
            exit(0);
        }
    }

    return 0;
}