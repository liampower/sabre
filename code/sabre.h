#ifndef SABRE_H
#define SABRE_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

typedef unsigned  uint;
typedef size_t    usize;
typedef unsigned char byte;

typedef uint8_t   u8;   
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef uint64_t  u64;

typedef int8_t   i8;   
typedef int16_t  i16;
typedef int32_t  i32;
typedef int64_t  i64;

typedef float    f32;
typedef double   f64;

#define packed_data __attribute__((packed))

extern "C" {
    __declspec(dllexport) extern unsigned int NvOptimusEnablement;
}

#endif

