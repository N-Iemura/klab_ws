#include <stdio.h>
#include <stdlib.h>

double Q[4][4], Qm[4], mTQ[4], alpha[4];
double graph1[600], graph2[600], graph3[600];
double mu;

int main(int argc, char **argv) {
    int i, j;

    for(i = 1; i <= 3; i++) {
        for(j = 1; j <= 3; j++) {
            Q[i][j] = 0.0;
        }
    }

    for(i = 1; i <= 3; i++) Qm[i] = mTQ[i] = 0.0;

    mu = 0.99; Q[1][1] = Q[2][2] = 1.0e6; Q[3][3] = 1.0e6;
    alpha[1] = 20000; alpha[2] = 400.0; alpha[3] = 5.0;

    for(i = 0; i < 600; i++)
        graph1[i] = graph2[i] = graph3[i] = 0.0;

    // Simulate data (replace this with your actual simulation code)
    for(i = 0; i < 600; i++) {
        graph1[i] = i * 0.1;  // Example data
        graph2[i] = i * 0.2;  // Example data
        graph3[i] = i * 0.3;  // Example data
    }

    // Save data to file
    FILE *fp = fopen("data_no_pid.txt", "w");
    if (fp == NULL) {
        perror("Failed to open file");
        return 1;
    }
    for (i = 0; i < 600; i++) {
        fprintf(fp, "%f %f %f\n", graph1[i], graph2[i], graph3[i]);
    }
    fclose(fp);

    return 0;
}