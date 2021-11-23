#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <cuda.h>
#include <cuda_runtime.h>
//#ifndef CUDATESTINCLUDE_H_
//#define CUDATESTINCLUDE_H_

__global__ void vector_add2(float *out, float *a, float *b, int n);

#include "cuda_test_include.cu"
