#include "matrix.h"

void matrix_multiply_3x3(float **a, float **b, float **dest) {
	int i, j, k;
	for(i = 0; i < 3; i++) {
		for(j = 0; j < 3; j++) {
			for(k = 0; k < 3; k++) {
				dest[i][j] +=  a[i][k] *  b[k][j];
			}
		}
	}
}
