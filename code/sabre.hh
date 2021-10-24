#ifndef SABRE_H
#define SABRE_H

#include <cstdint>
#include <cstdio>

#define packed_data __attribute__((packed))

#if defined(_WIN32)
  #define EXPORT_SYMBOL __declspec(dllexport)
#else
  #define EXPORT_SYMBOL
#endif

template<typename t, std::size_t n> constexpr inline std::size_t
ArrayCount(t (&A)[n])
{
    return sizeof(A) / sizeof(A[0]);
}

static inline void
LogInfo(const char* const Fmt, ...)
{
    va_list Args;
    va_start(Args, Fmt);

    printf("\033[1;34mINFO\033[0;0m    ");
    vprintf(Fmt, Args);
    printf("\n");
}

static inline void
LogError(const char* const Fmt, ...)
{
    va_list Args;
    va_start(Args, Fmt);

    printf("\033[1;31mFAIL\033[0;0m    ");
    vprintf(Fmt, Args);
    printf("\n");
}


static inline void
LogPlatform(const char* const Fmt, ...)
{
    va_list Args;
    va_start(Args, Fmt);

    printf("\033[1;35mPLAT\033[0;0m    ");
    vprintf(Fmt, Args);
    printf("\n");
}

// TODO(Liam): Stub out for release builds
static inline void
TraceOK(const char* const Fmt, ...)
{
    va_list Args;
    va_start(Args, Fmt);

    printf("\033[1;32mOKAY\033[0;0m    ");
    vprintf(Fmt, Args);
    printf("\n");
}

extern "C" {
   EXPORT_SYMBOL extern unsigned int NvOptimusEnablement;
}

#endif

