#ifndef VECMATH_H
#define VECMATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <xmmintrin.h>
#include <immintrin.h>

typedef struct vec3x8
{
    __m256 Xs;
    __m256 Ys;
    __m256 Zs;
} vec3x8;

static inline vec3x8
Cloned3x8(vec3 V)
{
    vec3x8 Out;

    Out.Xs = _mm256_set1_ps(V.X);
    Out.Ys = _mm256_set1_ps(V.Y);
    Out.Zs = _mm256_set1_ps(V.Z);

    return Out;
}







#ifdef __cplusplus
}
#endif

#endif

