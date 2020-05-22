#ifndef VECMATH_H
#define VECMATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "x86intrin.h"


#define F32_SGN_BIT 0x80000000U

typedef struct vec3x8
{
    __m256 Xs;
    __m256 Ys;
    __m256 Zs;
} vec3x8;

typedef struct vec3x4
{
    __m128 Xs;
    __m128 Ys;
    __m128 Zs;
} vec3x4;

static inline vec3x4
Broadcast3x4(vec3 V)
{
    vec3x4 Out;
    Out.Xs = _mm_set1_ps(V.X);
    Out.Ys = _mm_set1_ps(V.Y);
    Out.Zs = _mm_set1_ps(V.Z);

    return Out;
}

static inline vec3x8
Broadcast3x8(vec3 V)
{
    vec3x8 Out;

    Out.Xs = _mm256_broadcast_ss(&V.X);
    Out.Ys = _mm256_broadcast_ss(&V.Y);
    Out.Zs = _mm256_broadcast_ss(&V.Z);

    return Out;
}

static inline vec3x4
SetSelectedBits3x4(vec3x4 V, unsigned int BitValues, unsigned int SelectMsk)
{
    
}







#ifdef __cplusplus
}
#endif

#endif

