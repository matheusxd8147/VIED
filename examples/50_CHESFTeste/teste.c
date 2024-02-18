#include <stdio.h>
#include <stdlib.h>

int main(void) {
    int x[] = { 2015, 2016, 2017, 2018, 2019, 2020 };
    int y[] = { 344, 543, 433, 232, 212, 343 };
    int i;

    FILE *gnuplot = popen("gnuplot", "w");
    if (!gnuplot) {
        perror("popen");
        exit(EXIT_FAILURE);
    }

    fprintf(gnuplot, "plot '-' u 1:2 t 'Price' w lp\n");
    for (i = 0; i < 6; ++i) {
        fprintf(gnuplot,"%d %d\n", x[i], y[i]);
    }
    fprintf(gnuplot, "e\n");
    fprintf(stdout, "Click Ctrl+d to quit...\n");
    fflush(gnuplot);
    getchar();

    pclose(gnuplot);
    exit(EXIT_SUCCESS);
}