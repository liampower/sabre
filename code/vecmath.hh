#ifndef VECMATH_H
#define VECMATH_H

#include <cmath>
#include <cstdint>
#include <cfloat>

// {{{ New Types

// Primitive typedefs aren't namespaced
using uint  = unsigned int;
using usize = size_t;
using u8    = uint8_t;
using u16   = uint16_t;
using u32   = uint32_t;
using u64   = uint64_t;
using i8    = int8_t;
using i16   = int16_t;
using i32   = int32_t;
using i64   = int64_t;
using f32   = float;
using f64   = double;
using byte  = unsigned char;

// }}}

namespace vm
{

constexpr float PI_32  = 3.14159265f;
constexpr float SQRT_2 = 1.41421569f;
constexpr float INV_SQRT_2 = 0.70710678f;
constexpr float EPSILON = FLT_EPSILON;
constexpr float PI_OVER_180_DG = 0.01745329f;
constexpr float F32_MAX = FLT_MAX;
constexpr float F32_MIN = FLT_MIN;

// {{{ Vectors
template <typename c>
struct gvec2
{
    c X, Y;

    constexpr explicit gvec2() = default;
    constexpr explicit gvec2(c All) : X(All), Y(All) {}
    constexpr explicit gvec2(c _X, c _Y) : X(_X), Y(_Y) {}

    template<typename conv>
    constexpr explicit gvec2(gvec2<conv> Other) : gvec2(static_cast<c>(Other.X),
                                                        static_cast<c>(Other.Y)) {}

    template<typename conv>
    constexpr explicit gvec2(conv Other) : gvec2(static_cast<c>(Other),
                                                 static_cast<c>(Other)) {}
};


template <typename c>
struct gvec3
{
    c X, Y, Z;

    constexpr explicit gvec3() = default;
    constexpr explicit gvec3(c All) : X(All), Y(All), Z(All) {}
    constexpr explicit gvec3(c _X, c _Y, c _Z) : X(_X), Y(_Y), Z(_Z) {}

    template<typename conv>
    constexpr explicit gvec3(gvec3<conv> Other) : gvec3(static_cast<c>(Other.X),
                                                        static_cast<c>(Other.Y),
                                                        static_cast<c>(Other.Z)) {}
    template<typename conv>
    constexpr explicit gvec3(conv Other) : gvec3(static_cast<c>(Other),
                                                 static_cast<c>(Other),
                                                 static_cast<c>(Other)) {}
};


template <typename c>
struct gvec4
{
    c X, Y, Z, W;

    constexpr explicit gvec4() = default;
    constexpr explicit gvec4(c All) : X(All), Y(All), Z(All), W(All) {}
    constexpr explicit gvec4(c _X, c _Y, c _Z, c _W) : X(_X), Y(_Y), Z(_Z), W(_W) {}

    template<typename conv>
    constexpr explicit gvec4(gvec2<conv> Other) : gvec4(static_cast<c>(Other.X),
                                                        static_cast<c>(Other.Y),
                                                        static_cast<c>(Other.Z),
                                                        static_cast<c>(Other.W)) {}
    template<typename conv>
    constexpr explicit gvec4(conv Other) : gvec4(static_cast<c>(Other),
                                                 static_cast<c>(Other),
                                                 static_cast<c>(Other),
                                                 static_cast<c>(Other)) {}
};

using vec2 = gvec2<float>;
using vec3 = gvec3<float>;
using vec4 = gvec4<float>;
using ivec2 = gvec2<int>;
using ivec3 = gvec3<int>;
using ivec4 = gvec4<int>;

using uvec2 = gvec2<unsigned int>;
using uvec3 = gvec3<unsigned int>;
using uvec4 = gvec4<unsigned int>;

using bvec2 = gvec2<bool>;
using bvec3 = gvec3<bool>;
using bvec4 = gvec4<bool>;


// }}}


// {{{ Misc utilities

template <typename t> constexpr t
Maximum(t A, t B)
{
    return (A > B) ? A : B;
}

template <typename t> constexpr inline t
Minimum(t A, t B)
{
    return (A < B) ? A : B;
}


template <typename t> constexpr t
Clamp(t X, t Lo, t Hi)
{
    if (X < Lo) return Lo;
    if (X > Hi) return Hi;
    else        return X;
}

inline float
SelectNearestElem(vec3 V, float U)
{
    float D0 = std::abs(V.X - U);
    float D1 = std::abs(V.Y - U);
    float D2 = std::abs(V.Z - U);

    if (D0 < D1 && D0 < D2) return V.X;
    if (D1 < D0 && D1 < D2) return V.Y;
    else                    return V.Z;
}

constexpr inline float
Sign(float X)
{
    if (X > 0.0f) return 1.0f;
    if (X < 0.0f) return -1.0f;
    else          return 0.0f;
}

constexpr inline vec2
Sign(vec2 V)
{
    return vec2{ Sign(V.X), Sign(V.Y) };
}

constexpr inline vec3
Sign(vec3 V)
{
    return vec3{ Sign(V.X), Sign(V.Y), Sign(V.Z) };
}

constexpr inline vec4
Sign(vec4 V)
{
    return vec4{ Sign(V.X), Sign(V.Y), Sign(V.Z), Sign(V.W) };
}


static inline int
Round(float X)
{
    return static_cast<int>(std::roundf(X));
}

static inline u32
SafeIntToU32(int X)
{
    return static_cast<u32>(Maximum(X, 0));
}

constexpr inline float
Rads(float Degrees)
{
    return (Degrees*PI_OVER_180_DG);
}



// Min/Max for vectors are always component-wise
//
// Since we can't partially specialise function templates, we just define
// min/max/clamp for float vectors.
template <> constexpr inline vec2
Minimum<vec2>(vec2 A, vec2 B)
{
    return vec2{
        Minimum(A.X, B.X),
        Minimum(A.Y, B.Y)
    };
}

template <> constexpr inline vec2
Maximum<vec2>(vec2 A, vec2 B)
{
    return vec2{
        Maximum(A.X, B.X),
        Maximum(A.Y, B.Y)
    };
}

template <> constexpr inline vec3
Minimum<vec3>(vec3 A, vec3 B)
{
    return vec3{
        Minimum(A.X, B.X),
        Minimum(A.Y, B.Y),
        Minimum(A.Z, B.Z)
    };
}

template <> constexpr inline vec3
Maximum<vec3>(vec3 A, vec3 B)
{
    return vec3{
        Maximum(A.X, B.X),
        Maximum(A.Y, B.Y),
        Maximum(A.Z, B.Z)
    };
}

template <> constexpr inline vec4
Minimum<vec4>(vec4 A, vec4 B)
{
    return vec4{
        Minimum(A.X, B.X),
        Minimum(A.Y, B.Y),
        Minimum(A.Z, B.Z),
        Minimum(A.W, B.W)
    };
}

template <> constexpr inline vec4
Maximum<vec4>(vec4 A, vec4 B)
{
    return vec4{
        Maximum(A.X, B.X),
        Maximum(A.Y, B.Y),
        Maximum(A.Z, B.Z),
        Maximum(A.W, B.W)
    };
}

template <typename c> constexpr inline c
HorzMax(gvec2<c> V)
{
    return Maximum(V.X, V.Y);
}

template <typename c> constexpr inline c
HorzMin(gvec2<c> V)
{
    return Minimum(V.X, V.Y);
}

template <typename c> constexpr inline c
HorzMax(gvec3<c> V)
{
    return Maximum(V.X, Maximum(V.Y, V.Z));
}

template <typename c> constexpr inline c
HorzMin(gvec3<c> V)
{
    return Minimum(V.X, Minimum(V.Y, V.Z));
}

template <typename c> constexpr inline c
HorzMax(gvec4<c> V)
{
    return Maximum(V.X, Maximum(V.Y, Maximum(V.Z, V.W)));
}

template <typename c> constexpr inline c
HorzMin(gvec4<c> V)
{
    return Minimum(V.X, Minimum(V.Y, Minimum(V.Z, V.W)));
}

template <typename c> constexpr inline gvec2<c>
Abs(gvec2<c> V)
{
    return gvec2<c>{std::abs(V.X), std::abs(V.Y) };
}

template <typename c> constexpr inline gvec3<c>
Abs(gvec3<c> V)
{
    return gvec3<c>{std::abs(V.X), std::abs(V.Y), std::abs(V.Z) };
}

template <typename c> constexpr inline gvec4<c>
Abs(gvec4<c> V)
{
    return gvec4<c>{std::abs(V.X), std::abs(V.Y), std::abs(V.Z), std::abs(V.W) };
}

// }}}


// {{{ Boolean horizontal functions (vec2)

constexpr bool
All(bvec2 A)
{
    return A.X && A.Y;
}

constexpr bool
Any(bvec2 A)
{
    return A.X || A.Y;
}

// }}}


// {{{ Boolean horizontal functions (vec3)

constexpr bool
All(bvec3 A)
{
    return A.X && A.Y && A.Z;
}

constexpr bool
Any(bvec3 A)
{
    return A.X || A.Y || A.Z;
}

// }}}


// {{{ Boolean horizontal functions (vec4)

constexpr bool
All(bvec4 A)
{
    return A.X && A.Y && A.Z && A.W;
}

constexpr bool
Any(bvec4 A)
{
    return A.X || A.Y || A.Z || A.W;
}

// }}}


// {{{ Special boolean functions (vec2)
constexpr bvec2
operator&&(bvec2 L, bvec2 R)
{
    return bvec2{ L.X && R.X, L.Y && R.Y };
}

constexpr bvec2
operator||(bvec2 L, bvec2 R)
{
    return bvec2{ L.X || R.X, L.Y || R.Y };
}

constexpr bvec2
operator!(bvec2 L)
{
    return bvec2{ !L.X, !L.Y};
}
// }}}


// {{{ Special boolean functions (vec3)
constexpr bvec3
operator&&(bvec3 L, bvec3 R)
{
    return bvec3{ L.X && R.X, L.Y && R.Y, L.Z && R.Z };
}

constexpr bvec3
operator||(bvec3 L, bvec3 R)
{
    return bvec3{ L.X || R.X, L.Y || R.Y, L.Z || R.Z };
}

constexpr bvec3
operator!(bvec3 L)
{
    return bvec3{ !L.X, !L.Y, !L.Z};
}
// }}}


// {{{ Special boolean functions (vec3)
constexpr bvec4
operator&&(bvec4 L, bvec4 R)
{
    return bvec4{ L.X && R.X, L.Y && R.Y, L.Z && R.Z, L.W && R.W };
}

constexpr bvec4
operator||(bvec4 L, bvec4 R)
{
    return bvec4{ L.X || R.X, L.Y || R.Y, L.Z || R.Z, L.W || R.W };
}

constexpr bvec4
operator!(bvec4 L)
{
    return bvec4{ !L.X, !L.Y, !L.Z, !L.W};
}
// }}}


// {{{ Comparison operators (vec2)
template<typename c>
constexpr bvec2 Equal(gvec2<c> L, gvec2<c> R)
{
    return bvec2{ L.X == R.X, L.Y == R.Y };
}

template<typename c>
constexpr bvec2 NotEqual(gvec2<c> L, gvec2<c> R)
{
    return bvec2{ L.X != R.X, L.Y != R.Y };
}

template<>
inline bvec2 Equal(vec2 L, vec2 R)
{
    return bvec2{ fabsf(L.X - R.X) < EPSILON, fabsf(L.Y - R.Y) < EPSILON };
}

template<>
inline bvec2 NotEqual(vec2 L, vec2 R)
{
    return bvec2{ fabsf(L.X - R.X) > EPSILON, fabsf(L.Y - R.Y) > EPSILON };
}


template<typename c>
constexpr bvec2 LessThan(gvec2<c> L, gvec2<c> R)
{
    return bvec2{ L.X < R.X, L.Y < R.Y };
}

template<typename c>
constexpr bvec2 LessThanEqual(gvec2<c> L, gvec2<c> R)
{
    return bvec2{ L.X <= R.X, L.Y <= R.Y };
}

template<typename c>
constexpr bvec2 GreaterThan(gvec2<c> L, gvec2<c> R)
{
    return bvec2{ L.X > R.X, L.Y > R.Y };
}

template<typename c>
constexpr bvec2 GreaterThanEqual(gvec2<c> L, gvec2<c> R)
{
    return bvec2{ L.X >= R.X, L.Y >= R.Y };
}

// }}}


// {{{ Comparison operators (vec3)
template<typename c>
constexpr bvec3 Equal(gvec3<c> L, gvec3<c> R)
{
    return bvec3{ L.X == R.X, L.Y == R.Y, L.Z == R.Z };
}

template<typename c>
constexpr bvec3 NotEqual(gvec3<c> L, gvec3<c> R)
{
    return bvec3{ L.X != R.X, L.Y != R.Y, L.Z != R.Z };
}

template<>
inline bvec3 Equal(vec3 L, vec3 R)
{
    return bvec3{ 
        fabsf(L.X - R.X) < EPSILON,
        fabsf(L.Y - R.Y) < EPSILON,
        fabsf(L.Z - R.Z) < EPSILON,
    };
}

template<>
inline bvec3 NotEqual(vec3 L, vec3 R)
{
    return bvec3{ 
        fabsf(L.X - R.X) > EPSILON,
        fabsf(L.Y - R.Y) > EPSILON,
        fabsf(L.Z - R.Z) > EPSILON,
    };
}

template<typename c>
constexpr bvec3 LessThan(gvec3<c> L, gvec3<c> R)
{
    return bvec3{ L.X < R.X, L.Y < R.Y, L.Z < R.Z };
}

template<typename c>
constexpr bvec3 LessThanEqual(gvec3<c> L, gvec3<c> R)
{
    return bvec3{ L.X <= R.X, L.Y <= R.Y, L.Z <= R.Z };
}

template<typename c>
constexpr bvec3 GreaterThan(gvec3<c> L, gvec3<c> R)
{
    return bvec3{ L.X > R.X, L.Y > R.Y, L.Z > R.Z };
}

template<typename c>
constexpr bvec3 GreaterThanEqual(gvec3<c> L, gvec3<c> R)
{
    return bvec3{ L.X >= R.X, L.Y >= R.Y, L.Z >= R.Z };
}

// }}}


// {{{ Comparison operators (vec4)
template<typename c>
constexpr bvec4 Equal(gvec4<c> L, gvec4<c> R)
{
    return bvec4{ L.X == R.X, L.Y == R.Y, L.Z == R.Z, L.W == R.W };
}

template<typename c>
constexpr bvec4 NotEqual(gvec4<c> L, gvec4<c> R)
{
    return bvec4{ L.X != R.X, L.Y != R.Y, L.Z != R.Z, L.W != R.W };
}

template<typename c>
constexpr bvec4 LessThan(gvec4<c> L, gvec4<c> R)
{
    return bvec4{ L.X < R.X, L.Y < R.Y, L.Z < R.Z, L.W < R.W };
}

template<typename c>
constexpr bvec4 LessThanEqual(gvec4<c> L, gvec4<c> R)
{
    return bvec4{ L.X <= R.X, L.Y <= R.Y, L.Z <= R.Z, L.W <= R.W };
}

template<typename c>
constexpr bvec4 GreaterThan(gvec4<c> L, gvec4<c> R)
{
    return bvec4{ L.X > R.X, L.Y > R.Y, L.Z > R.Z, L.W > R.W };
}

template<typename c>
constexpr bvec4 GreaterThanEqual(gvec4<c> L, gvec4<c> R)
{
    return bvec4{ L.X >= R.X, L.Y >= R.Y, L.Z >= R.Z, L.W >= R.W };
}

// }}}


// {{{ Arithmetic operators (vec2)
template <typename c> constexpr gvec2<c>
operator+(gvec2<c> L, gvec2<c> R)
{
    return gvec2<c>{ L.X+R.X, L.Y+R.Y };
}

template <typename c> constexpr gvec2<c>
operator-(gvec2<c> L, gvec2<c> R)
{
    return gvec2<c>{ L.X-R.X, L.Y-R.Y };
}

template <typename c> constexpr gvec2<c>
operator*(gvec2<c> L, gvec2<c> R)
{
    return gvec2<c>{ L.X*R.X, L.Y*R.Y };
}

template <typename c> constexpr gvec2<c>
operator/(gvec2<c> L, gvec2<c> R)
{
    return gvec2<c>{ L.X/R.X, L.Y/R.Y };
}

template <typename c> constexpr gvec2<c>
operator%(gvec2<c> L, gvec2<c> R)
{
    return gvec2<c>{ L.X%R.X, L.Y%R.Y };
}

// Assignment operators
template <typename c> constexpr gvec2<c>& 
operator+=(gvec2<c>& L, gvec2<c> R)
{
    L = L + R;
    return L;
}

template <typename c> constexpr gvec2<c>&
operator-=(gvec2<c>& L, gvec2<c> R)
{
    L = L - R;
    return L;
}

template <typename c> constexpr gvec2<c>&
operator*=(gvec2<c>& L, gvec2<c> R)
{
    L = L * R;
    return L;
}

template <typename c> constexpr gvec2<c>&
operator/=(gvec2<c>& L, gvec2<c> R)
{
    L = L / R;
    return L;
}

template <typename c> constexpr gvec2<c>&
operator%=(gvec2<c> L, gvec2<c> R)
{
    L = L % R;
    return L;
}

// Scalars always come on the right

template <typename c> constexpr gvec2<c>
operator+(gvec2<c> L, c R)
{
    return gvec2<c>{ L.X + R, L.Y + R };
}

template <typename c> constexpr gvec2<c>
operator-(gvec2<c> L, c R)
{
    return gvec2<c>{ L.X - R, L.Y - R };
}

template <typename c> constexpr gvec2<c>
operator*(gvec2<c> L, c R)
{
    return gvec2<c>{ L.X*R, L.Y*R };
}

template <typename c> constexpr gvec2<c>
operator/(gvec2<c> L, c R)
{
    return gvec2<c>{ L.X/R, L.Y/R };
}

template <typename c> constexpr gvec2<c>
operator%(gvec2<c> L, c R)
{
    return gvec2<c>{ L.X%R, L.Y%R };
}
// }}}


// {{{ Arithmetic operators (vec3)
template <typename c> constexpr gvec3<c>
operator+(gvec3<c> L, gvec3<c> R)
{
    return gvec3<c>{ L.X+R.X, L.Y+R.Y, L.Z+R.Z };
}

template <typename c> constexpr gvec3<c>
operator-(gvec3<c> L, gvec3<c> R)
{
    return gvec3<c>{ L.X-R.X, L.Y-R.Y, L.Z-R.Z };
}

template <typename c> constexpr gvec3<c>
operator*(gvec3<c> L, gvec3<c> R)
{
    return gvec3<c>{ L.X*R.X, L.Y*R.Y, L.Z*R.Z };
}

template <typename c> constexpr gvec3<c>
operator/(gvec3<c> L, gvec3<c> R)
{
    return gvec3<c>{ L.X/R.X, L.Y/R.Y, L.Z/R.Z };
}

template <typename c> constexpr gvec3<c>
operator%(gvec3<c> L, gvec3<c> R)
{
    return gvec3<c>{ L.X%R.X, L.Y%R.Y, L.Z%R.Z };
}

// Assignment operators
template <typename c> constexpr gvec3<c>& 
operator+=(gvec3<c>& L, gvec3<c> R)
{
    L = L + R;
    return L;
}

template <typename c> constexpr gvec3<c>&
operator-=(gvec3<c>& L, gvec3<c> R)
{
    L = L - R;
    return L;
}

template <typename c> constexpr gvec3<c>&
operator*=(gvec3<c>& L, gvec3<c> R)
{
    L = L * R;
    return L;
}

template <typename c> constexpr gvec3<c>&
operator/=(gvec3<c>& L, gvec3<c> R)
{
    L = L / R;
    return L;
}

template <typename c> constexpr gvec3<c>&
operator%=(gvec3<c> L, gvec3<c> R)
{
    L = L % R;
    return L;
}

// Scalars always come on the right

template <typename c> constexpr gvec3<c>
operator+(gvec3<c> L, c R)
{
    return gvec3<c>{ L.X + R, L.Y + R, L.Z + R };
}

template <typename c> constexpr gvec3<c>
operator-(gvec3<c> L, c R)
{
    return gvec3<c>{ L.X - R, L.Y - R, L.Z - R };
}

template <typename c> constexpr gvec3<c>
operator*(gvec3<c> L, c R)
{
    return gvec3<c>{ L.X*R, L.Y*R, L.Z*R };
}

template <typename c> constexpr gvec3<c>
operator/(gvec3<c> L, c R)
{
    return gvec3<c>{ L.X/R, L.Y/R, L.Z/R };
}

template <typename c> constexpr gvec3<c>
operator%(gvec3<c> L, c R)
{
    return gvec3<c>{ L.X%R, L.Y%R, L.Z%R };
}
// }}}


// {{{ Arithmetic operators (vec4)
template <typename c> constexpr gvec4<c>
inline operator+(gvec4<c> L, gvec4<c> R)
{
    return gvec4<c>{ L.X+R.X, L.Y+R.Y, L.Z+R.Z, L.W+R.W };
}

template <typename c> constexpr gvec4<c>
inline operator-(gvec4<c> L, gvec4<c> R)
{
    return gvec4<c>{ L.X-R.X, L.Y-R.Y, L.Z-R.Z, L.W-R.W };
}

template <typename c> constexpr gvec4<c>
inline operator*(gvec4<c> L, gvec4<c> R)
{
    return gvec4<c>{ L.X*R.X, L.Y*R.Y, L.Z*R.Z, L.W*R.W };
}

template <typename c> constexpr gvec4<c>
inline operator/(gvec4<c> L, gvec4<c> R)
{
    return gvec4<c>{ L.X/R.X, L.Y/R.Y, L.Z/R.Z, L.W/R.W };
}

template <typename c> constexpr gvec4<c>
inline operator%(gvec4<c> L, gvec4<c> R)
{
    return gvec4<c>{ L.X%R.X, L.Y%R.Y, L.Z%R.Z, L.W%R.W };
}

// Scalars always come on the right

template <typename c> constexpr gvec4<c>
operator+(gvec4<c> L, c R)
{
    return gvec4<c>{ L.X + R, L.Y + R, L.Z + R, L.W + R };
}

template <typename c> constexpr gvec4<c>
operator-(gvec4<c> L, c R)
{
    return gvec4<c>{ L.X - R, L.Y - R, L.Z - R, L.W - R };
}

template <typename c> constexpr gvec4<c>
operator*(gvec4<c> L, c R)
{
    return gvec4<c>{ L.X*R, L.Y*R, L.Z*R, L.W*R };
}

template <typename c> constexpr gvec4<c>
operator/(gvec4<c> L, c R)
{
    return gvec4<c>{ L.X/R, L.Y/R, L.Z/R, L.W/R };
}

template <typename c> constexpr gvec4<c>
operator%(gvec4<c> L, c R)
{
    return gvec4<c>{ L.X%R, L.Y%R, L.Z%R, L.W%R };
}
// }}}


// {{{ Bitwise operations for unsigned vectors
inline uvec2
operator&(uvec2 L, uvec2 R)
{
    return uvec2{ L.X&R.X, L.Y&R.Y };
}

inline uvec2
operator|(uvec2 L, uvec2 R)
{
    return uvec2{ L.X|R.X, L.Y|R.Y };
}

inline uvec2
operator^(uvec2 L, uvec2 R)
{
    return uvec2{ L.X^R.X, L.Y^R.Y };
}

inline uvec2
operator>>(uvec2 L, uvec2 R)
{
    return uvec2{ L.X>>R.X, L.Y>>R.Y };
}

inline uvec2
operator<<(uvec2 L, uvec2 R)
{
    return uvec2{ L.X<<R.X, L.Y<<R.Y };
}

inline uvec2
operator~(uvec2 L)
{
    return uvec2{ ~L.X, ~L.Y };
}

inline uvec2
operator&(uvec2 L, unsigned int R)
{
    return uvec2{ L.X&R, L.Y&R };
}

inline uvec2
operator|(uvec2 L, unsigned int R)
{
    return uvec2{ L.X|R, L.Y|R };
}

inline uvec2
operator^(uvec2 L, unsigned int R)
{
    return uvec2{ L.X^R, L.Y^R };
}

inline uvec2
operator>>(uvec2 L, unsigned int R)
{
    return uvec2{ L.X>>R, L.Y>>R };
}

inline uvec2
operator<<(uvec2 L, unsigned int R)
{
    return uvec2{ L.X<<R, L.Y<<R };
}


// Vec3
inline uvec3
operator&(uvec3 L, uvec3 R)
{
    return uvec3{ L.X&R.X, L.Y&R.Y, L.Z&R.Z };
}

inline uvec3
operator|(uvec3 L, uvec3 R)
{
    return uvec3{ L.X|R.X, L.Y|R.Y, L.Z|R.Z };
}

inline uvec3
operator^(uvec3 L, uvec3 R)
{
    return uvec3{ L.X^R.X, L.Y^R.Y, L.Z^R.Z };
}

inline uvec3
operator>>(uvec3 L, uvec3 R)
{
    return uvec3{ L.X>>R.X, L.Y>>R.Y, L.Z>>R.Z };
}

inline uvec3
operator<<(uvec3 L, uvec3 R)
{
    return uvec3{ L.X<<R.X, L.Y<<R.Y, L.Z<<R.Z };
}

inline uvec3
operator~(uvec3 L)
{
    return uvec3{ ~L.X, ~L.Y, ~L.Z };
}

inline uvec3
operator&(uvec3 L, unsigned int R)
{
    return uvec3{ L.X&R, L.Y&R, L.Z&R };
}

inline uvec3
operator|(uvec3 L, unsigned int R)
{
    return uvec3{ L.X|R, L.Y|R, L.Z|R };
}

inline uvec3
operator^(uvec3 L, unsigned int R)
{
    return uvec3{ L.X^R, L.Y^R, L.Z^R };
}

inline uvec3
operator>>(uvec3 L, unsigned int R)
{
    return uvec3{ L.X>>R, L.Y>>R, L.Z>>R };
}

inline uvec3
operator<<(uvec3 L, unsigned int R)
{
    return uvec3{ L.X<<R, L.Y<<R, L.Z<<R };
}

// Vec4
constexpr inline uvec4
operator&(uvec4 L, uvec4 R)
{
    return uvec4{ L.X&R.X, L.Y&R.Y, L.Z&R.Z, L.W&R.W };
}

constexpr inline uvec4
operator|(uvec4 L, uvec4 R)
{
    return uvec4{ L.X|R.X, L.Y|R.Y, L.Z|R.Z, L.W|R.W };
}

constexpr inline uvec4
operator^(uvec4 L, uvec4 R)
{
    return uvec4{ L.X^R.X, L.Y^R.Y, L.Z^R.Z, L.W^R.W };
}

constexpr inline uvec4
operator>>(uvec4 L, uvec4 R)
{
    return uvec4{ L.X>>R.X, L.Y>>R.Y, L.Z>>R.Z, L.W>>R.W };
}

constexpr inline uvec4
operator<<(uvec4 L, uvec4 R)
{
    return uvec4{ L.X<<R.X, L.Y<<R.Y, L.Z<<R.Z, L.W<<R.W };
}

constexpr inline uvec4
operator~(uvec4 L)
{
    return uvec4{ ~L.X, ~L.Y, ~L.Z, ~L.W };
}

constexpr inline uvec4
operator&(uvec4 L, unsigned int R)
{
    return uvec4{ L.X&R, L.Y&R, L.Z&R, L.W&R };
}

constexpr inline uvec4
operator|(uvec4 L, unsigned int R)
{
    return uvec4{ L.X|R, L.Y|R, L.Z|R, L.W|R };
}

constexpr inline uvec4
operator^(uvec4 L, unsigned int R)
{
    return uvec4{ L.X^R, L.Y^R, L.Z^R, L.W^R };
}

constexpr inline uvec4
operator>>(uvec4 L, unsigned int R)
{
    return uvec4{ L.X>>R, L.Y>>R, L.Z>>R, L.W>>R };
}

constexpr inline uvec4
operator<<(uvec4 L, unsigned int R)
{
    return uvec4{ L.X<<R, L.Y<<R, L.Z<<R, L.W<<R };
}
// }}}


// {{{ Vector/Linear Algebra functions
template <typename c> constexpr c
Dot(gvec2<c> L, gvec2<c> R)
{
    return L.X*R.X + L.Y*R.Y;
}

template <typename c> constexpr c
Dot(gvec3<c> L, gvec3<c> R)
{
    return L.X*R.X + L.Y*R.Y + L.Z*R.Z;
}

template <typename c> constexpr c
Dot(gvec4<c> L, gvec4<c> R)
{
    return L.X*R.X + L.Y*R.Y + L.Z*R.Z + L.W*R.W;
}

inline float
Length(vec2 V)
{
    return sqrtf(V.X*V.X + V.Y*V.Y);
}

inline float
Length(vec3 V)
{
    return sqrtf(V.X*V.X + V.Y*V.Y + V.Z*V.Z);
}

inline float
Length(vec4 V)
{
    return sqrtf(V.X*V.X + V.Y*V.Y + V.Z*V.Z + V.W*V.W);
}

inline float
LengthSq(vec2 V)
{
    return (V.X*V.X + V.Y*V.Y);
}

inline float
LengthSq(vec3 V)
{
    return (V.X*V.X + V.Y*V.Y + V.Z*V.Z);
}

inline float
LengthSq(vec4 V)
{
    return (V.X*V.X + V.Y*V.Y + V.Z*V.Z + V.W*V.W);
}

inline vec2
Normalize(vec2 V)
{
    float L = Length(V);

    return (0.0f != L) ? (V / L) : V;
}

inline vec3
Normalize(vec3 V)
{
    float L = Length(V);

    return (0.0f != L) ? (V / L) : V;
}

inline vec4
Normalize(vec4 V)
{
    float L = Length(V);

    return (0.0f != L) ? (V / L) : V;
}

constexpr inline vec3
Cross(vec3 U, vec3 V)
{
    return vec3{
        (U.Y * V.Z) - (U.Z * V.Y),
        (U.Z * V.X) - (U.X * V.Z),
        (U.X * V.Y) - (U.Y * V.X)
    };
}

constexpr inline vec3
Reciprocal(vec3 V)
{
    return vec3{ 1.0f/V.X, 1.0f/V.Y, 1.0f/V.Z };
}

template <typename c> constexpr inline gvec2<c>
Select(gvec2<c> T, gvec2<c> F, bvec2 Msk)
{
    return gvec2<c>{ (Msk.X) ? T.X : F.X, (Msk.Y) ? T.Y : F.Y };
}

template <typename c> constexpr inline gvec3<c>
Select(gvec3<c> T, gvec3<c> F, bvec3 Msk)
{
    return gvec3<c>{
        (Msk.X) ? T.X : F.X,
        (Msk.Y) ? T.Y : F.Y,
        (Msk.Z) ? T.Z : F.Z
    };
}

template <typename c> constexpr inline gvec4<c>
Select(gvec4<c> T, gvec4<c> F, bvec4 Msk)
{
    return gvec3<c>{
        (Msk.X) ? T.X : F.X,
        (Msk.Y) ? T.Y : F.Y,
        (Msk.Z) ? T.Z : F.Z,
        (Msk.W) ? T.W : F.W
    };
}


// }}}


// {{{ Matrices


struct mat4x4
{
    float M[4][4];
};

struct mat3x3
{
    float M[3][3];
};

constexpr inline mat4x4
IdentityMatrix4()
{
    return mat4x4{{
        { 1.0f, 0.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f, 1.0f }
    }};
}

constexpr inline mat3x3
IdentityMatrix3()
{
    return mat3x3{{
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
    }};
}

constexpr inline mat4x4
Translation(vec3 Translation)
{
    mat4x4 T = IdentityMatrix4();
    
    T.M[0][3] = Translation.X;
    T.M[1][3] = Translation.Y;
    T.M[2][3] = Translation.Z;

    return T;
}

constexpr inline void 
Translate(mat4x4& Matrix, vec3 Translation)
{
    Matrix.M[0][3] = Translation.X;
    Matrix.M[1][3] = Translation.Y;
    Matrix.M[2][3] = Translation.Z;
}

inline vec3 
operator*(const vec3& V, const mat3x3& M)
{
    vec3 Out;

    // Matrix indexed as [row][col]
    Out.X = V.X*M.M[0][0] + V.Y*M.M[1][0] + V.Z*M.M[2][0];
    Out.Y = V.X*M.M[0][1] + V.Y*M.M[1][1] + V.Z*M.M[2][1];
    Out.Z = V.X*M.M[0][2] + V.Y*M.M[1][2] + V.Z*M.M[2][2];

    return Out;
}

inline mat4x4
operator*(mat4x4 L, mat4x4 R)
{
    mat4x4 O;
    O.M[0][0] = (L.M[0][0]*R.M[0][0]) + (L.M[0][1]*R.M[1][0]) + (L.M[0][2]*R.M[2][0]) + (L.M[0][3]*R.M[3][0]);
    O.M[0][1] = (L.M[0][0]*R.M[0][1]) + (L.M[0][1]*R.M[1][1]) + (L.M[0][2]*R.M[2][1]) + (L.M[0][3]*R.M[3][1]);
    O.M[0][2] = (L.M[0][0]*R.M[0][2]) + (L.M[0][1]*R.M[1][2]) + (L.M[0][2]*R.M[2][2]) + (L.M[0][3]*R.M[3][2]);
    O.M[0][3] = (L.M[0][0]*R.M[0][3]) + (L.M[0][1]*R.M[1][3]) + (L.M[0][2]*R.M[2][3]) + (L.M[0][3]*R.M[3][3]);

    O.M[1][0] = (L.M[1][0]*R.M[0][0]) + (L.M[1][1]*R.M[1][0]) + (L.M[1][2]*R.M[2][0]) + (L.M[1][3]*R.M[3][0]);
    O.M[1][1] = (L.M[1][0]*R.M[0][1]) + (L.M[1][1]*R.M[1][1]) + (L.M[1][2]*R.M[2][1]) + (L.M[1][3]*R.M[3][1]);
    O.M[1][2] = (L.M[1][0]*R.M[0][2]) + (L.M[1][1]*R.M[1][2]) + (L.M[1][2]*R.M[2][2]) + (L.M[1][3]*R.M[3][2]);
    O.M[1][3] = (L.M[1][0]*R.M[0][3]) + (L.M[1][1]*R.M[1][3]) + (L.M[1][2]*R.M[2][3]) + (L.M[1][3]*R.M[3][3]);

    O.M[2][0] = (L.M[2][0]*R.M[0][0]) + (L.M[2][1]*R.M[1][0]) + (L.M[2][2]*R.M[2][0]) + (L.M[2][3]*R.M[3][0]);
    O.M[2][1] = (L.M[2][0]*R.M[0][1]) + (L.M[2][1]*R.M[1][1]) + (L.M[2][2]*R.M[2][1]) + (L.M[2][3]*R.M[3][1]);
    O.M[2][2] = (L.M[2][0]*R.M[0][2]) + (L.M[2][1]*R.M[1][2]) + (L.M[2][2]*R.M[2][2]) + (L.M[2][3]*R.M[3][2]);
    O.M[2][3] = (L.M[2][0]*R.M[0][3]) + (L.M[2][1]*R.M[1][3]) + (L.M[2][2]*R.M[2][3]) + (L.M[2][3]*R.M[3][3]);

    O.M[3][0] = (L.M[3][0]*R.M[0][0]) + (L.M[3][1]*R.M[1][0]) + (L.M[3][2]*R.M[2][0]) + (L.M[3][3]*R.M[3][0]);
    O.M[3][1] = (L.M[3][0]*R.M[0][1]) + (L.M[3][1]*R.M[1][1]) + (L.M[3][2]*R.M[2][1]) + (L.M[3][3]*R.M[3][1]);
    O.M[3][2] = (L.M[3][0]*R.M[0][2]) + (L.M[3][1]*R.M[1][2]) + (L.M[3][2]*R.M[2][2]) + (L.M[3][3]*R.M[3][2]);
    O.M[3][3] = (L.M[3][0]*R.M[0][3]) + (L.M[3][1]*R.M[1][3]) + (L.M[3][2]*R.M[2][3]) + (L.M[3][3]*R.M[3][3]);

    return O;
}

constexpr inline mat4x4
PerspectiveProjection(float CotHalfFov, float AspectRatio, float Zn, float Zf)
{
    const float ScaleX = CotHalfFov;
    const float ScaleY = CotHalfFov * AspectRatio;
    const float ScaleZ = -(Zf + Zn) / (Zf - Zn);
    const float OffsetZ = -(2.0f * Zn * Zf) / (Zf - Zn);
    const float ScaleW = -1.0f;

    mat4x4 P = {{
        { ScaleX, 0.0f,   0.0f,   0.0f },
        { 0.0f,   ScaleY, 0.0f,   0.0f },
        { 0.0f,   0.0f,   ScaleZ, OffsetZ },
        { 0.0f,   0.0f,   ScaleW,   0.0f } 
    }};

    return P;
}

// }}}


// {{{ Quaternions
struct quat
{
    float X, Y, Z, W;
};

constexpr inline quat 
operator*(const quat& L, const quat& R)
{
    // NOTE: Quaternion Multiplication Derivation
    // RULES: 
    //   ij = k   ji = -k
    //   jk = i   kj = -i
    //   ki = j   ik = -j
    //
    //         left                right
    //   (a + bi + cj + dk) * (w + xi + yj + zk)
    //    LX  LY   LZ   LW     RX  RY   RZ   RW
    //
    //   = [a.w + a.xi + a.yj + a.zk]
    //   + [bi.w + bi.xi + bi.yj + bi.zk]
    //   + [cj.w + cj.xi + cj.yj + cj.zk]
    //   + [dk.w + dk.xi + dk.yj + dk.zk]
    //
    //   = [a.w + a.xi + a.yj + a.zk]
    //   + [bi.w - b.x + by.k - bz.j]
    //   + [cj.w - cx.k - c.y + cz.i]
    //   + [dk.w + dx.j - dy.i - d.z]
    //
    //   = [a.w  - b.x  - c.y  - d.z]
    //   + [ax.i + bw.i + cz.i - dy.i]
    //   + [ay.j - bz.j + cj.w + dx.j]
    //   + [az.k + by.k - cx.k + dw.k]
    //
    //   = (aw - bx - cy - dz,
    //      ax + bw + cz - dy,
    //      ay - bz + cw + dx,
    //      az + by - cx + dw)

    //         left                right
    //   (a + bi + cj + dk) * (w + xi + yj + zk)
    //    LX  LY   LZ   LW     RX  RY   RZ   RW
    return quat{
         (L.X * R.X) - (L.Y * R.Y) - (L.Z * R.Z) - (L.W * R.W),
         (L.X * R.Y) + (L.Y * R.X) + (L.Z * R.W) - (L.W * R.Z),
         (L.X * R.Z) - (L.Y * R.W) + (L.Z * R.X) + (L.W * R.Y),
         (L.X * R.W) + (L.Y * R.Z) - (L.Z * R.Y) + (L.W * R.X)
    };
}

inline quat
RotationQuaternion(const float Angle, vec3 Axis)
{
    quat Result;

    Result.X = cosf(Angle / 2.0f);

    Result.Y = Axis.X * sinf(Angle / 2.0f);
    Result.Z = Axis.Y * sinf(Angle / 2.0f);
    Result.W = Axis.Z * sinf(Angle / 2.0f);

    return Result;
}

constexpr inline quat
Conjugate(quat Q)
{
    return quat{ Q.X, -Q.Y, -Q.Z, -Q.W };
}

inline float
QuatLength(quat Q)
{
    return sqrtf(Q.X*Q.X + Q.Y*Q.Y + Q.Z*Q.Z + Q.W*Q.W);
}

inline quat
Normalize(quat Q)
{
    quat Result;
    float Length = QuatLength(Q);

    Result.X = Q.X / Length;
    Result.Y = Q.Y / Length;
    Result.Z = Q.Z / Length;
    Result.W = Q.W / Length;

    return Result;
}

constexpr inline vec3
ImaginaryPart(quat Q)
{
    return vec3{ Q.Y, Q.Z, Q.W };
}

constexpr inline float
RealPart(quat Q)
{
    return Q.X;
}

constexpr inline vec3
Rotate(quat Rotation, vec3 V)
{
    vec3 T =  Cross(ImaginaryPart(Rotation), V) * 2.0f;
    vec3 R = (T * RealPart(Rotation)) + Cross(ImaginaryPart(Rotation), T) + V;

    return R;

}
// }}}


// {{{ Morton key computation
static inline u32
HashVec3(uvec3 V)
{
    return (V.X*73856093U) ^ (V.Y*19349663U) ^ (V.Z*83492791U);
}

using morton_key = u64;
// Morton encoding by Fabien 'ryg' Giesen
// Giesen 2009, "Decoding Morton Codes"
// https://fgiesen.wordpress.com/2009/12/13/decoding-morton-codes/
// Accessed Mar. 2020

static inline u64
Part1By2_64(u64 X)
{
    // Select low 21 bits
    // Since we're interleaving the bits of three coordinates,
    // we can only fit 3*21 = 63 bits in a 64-bit number.
    X &= 0x1FFFFF;

    X = (X ^ (X << 32ULL)) & 0x001F00000000FFFF;
    X = (X ^ (X << 16ULL)) & 0x001F0000FF0000FF;
    X = (X ^ (X <<  8ULL)) & 0x100F00F00F00F00F;
    X = (X ^ (X <<  4ULL)) & 0x10C30C30C30C30C3;
    X = (X ^ (X <<  2ULL)) & 0x1249249249249249;

    return X;

}


static inline morton_key
EncodeMorton3(uvec3 V)
{
    return (Part1By2_64((u64)V.Z) << 2ULL) + (Part1By2_64((u64)V.Y) << 1ULL) + Part1By2_64((u64)V.X);
}


// }}}


inline bool
IsInsideSphericalRegion(vec3 P, vec3 Centre, float Radius)
{
    return (Length(Centre - P) <= Radius);
}

template <typename t> constexpr inline t
Remap01(t X)
{
    return ((X + 1.0f) / 2.0f);
}



}

#endif

