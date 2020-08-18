#ifndef SABRE_H
#define SABRE_H

#include <cstdint>

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

template<typename t, size_t n> constexpr inline size_t
ArrayCount(t (&A)[n])
{
    return sizeof(A) / sizeof(A[0]);
}

extern "C" {
    __declspec(dllexport) extern unsigned int NvOptimusEnablement;
}

#endif

