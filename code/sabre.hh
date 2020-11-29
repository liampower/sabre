#ifndef SABRE_H
#define SABRE_H

#include <cstdint>

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

