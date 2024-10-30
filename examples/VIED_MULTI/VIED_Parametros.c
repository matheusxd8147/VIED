#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

int main()
{

    int funcoes;
    float pick_up_50, pick_up_50N, pick_up_51, pick_up_51V, pick_up_51N, pick_up_67, pick_up_67N, atm_67, atm_67N;
    float curva_51, curva_51V, tensao_51V, curva_51N, curva_67, curva_67N, dial_51, dial_51V, dial_51N, dial_67, dial_67N;

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
            printf("Ajustes Função 50:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_50);
        }
        if (funcoes == 2)
        {
            printf("Ajustes Função 50N:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_50N);
        }
        if (funcoes == 3)
        {
            printf("Ajustes Função 51:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_51);
        }
        if (funcoes == 4)
        {
            printf("Ajustes Função 51V:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_51V);
        }
        if (funcoes == 5)
        {
            printf("Ajustes Função 51N:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_51N);
        }
        if (funcoes == 6)
        {
            printf("Ajustes Função 67:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_67);
        }
        if (funcoes == 7)
        {
            printf("Ajustes Função 67N:");
            printf("\nPick-Up: ");
            scanf("%f",&pick_up_67+N);
        }
        if (funcoes == 8)
        {
            exit(0);
        }
    }

    return 0;
}