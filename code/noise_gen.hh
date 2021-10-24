#ifndef NOISE_GEN_H
#define NOISE_GEN_H

#include "vecmath.hh"

struct noise_sample
{
    f32  Value;
    vm::vec3 Deriv;
};

extern svo*
BuildNoiseSvo(u32 MaxDepth, u32 ScaleExp);

extern noise_sample
Perlin3Noise(vm::vec3 P, f32 Freq, f32 Amp, f32 Falloff);

#endif
