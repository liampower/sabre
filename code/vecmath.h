#ifndef VECMATH_H
#define VECMATH_H

#include <cmath>

namespace vm
{

constexpr float Pi32  = 3.14159265f;
constexpr float Sqrt2 = 1.41421569f;
constexpr float InvSqrt2 = 0.70710678f;
constexpr float PiOver180Dg = 0.01745329f;


// {{{ Misc utilities
template <typename t> constexpr t
Maximum(t A, t B)
{
    return (A > B) ? A : B;
}

template <typename t> constexpr t
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

template <typename t> constexpr t
Sign(t X)
{
    if (X > 0) return t(1.0f);
    if (X < 0) return t(-1.0f);
    else       return t(0.0f);
}

static inline int
Round(float X)
{
    return (int)std::roundf(X);
}

static inline u32
SafeIntToU32(int X)
{
    return (u32)(Maximum(X, 0));
}

constexpr inline float
Rads(float Degrees)
{
    return (Degrees*PiOver180Dg);
}

// }}}


// {{{ Vectors
template <typename ctype>
struct gvec2
{
    ctype X, Y;

    constexpr explicit gvec2() = default;
    constexpr explicit gvec2(ctype Uniform) : X(Uniform), Y(Uniform) {}
    constexpr explicit gvec2(ctype _X, ctype _Y) : X(_X), Y(_Y) {}
};


template <typename ctype>
struct gvec3
{
    ctype X, Y, Z;

    constexpr explicit gvec3() = default;
    constexpr explicit gvec3(ctype All) : X(All), Y(All), Z(All) {}
    constexpr explicit gvec3(ctype _X, ctype _Y, ctype _Z) : X(_X), Y(_Y), Z(_Z) {}
};


template <typename ctype>
struct gvec4
{
    ctype X, Y, Z, W;

    constexpr explicit gvec4() = default;
    constexpr explicit gvec4(ctype All) : X(All), Y(All), Z(All), W(All) {}
    constexpr explicit gvec4(ctype _X, ctype _Y, ctype _Z, ctype _W) : X(_X), Y(_Y), Z(_Z), W(_W) {}
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
    return gvec3<c>{ L.X-R.X, L.Y-R.Y, L.Y-R.Y };
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
    return gvec4<c>{ L.X+R.X, L.Y+R.Y, L.Z+R.Z };
}

template <typename c> constexpr gvec4<c>
inline operator-(gvec4<c> L, gvec4<c> R)
{
    return gvec4<c>{ L.X-R.X, L.Y-R.Y, L.Y-R.Y };
}

template <typename c> constexpr gvec4<c>
inline operator*(gvec4<c> L, gvec4<c> R)
{
    return gvec4<c>{ L.X*R.X, L.Y*R.Y, L.Z*R.Z };
}

template <typename c> constexpr gvec4<c>
inline operator/(gvec4<c> L, gvec4<c> R)
{
    return gvec4<c>{ L.X/R.X, L.Y/R.Y, L.Z/R.Z, L.W/R.w };
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

// Vec4
inline uvec4
operator&(uvec4 L, uvec4 R)
{
    return uvec4{ L.X&R.X, L.Y&R.Y, L.Z&R.Z, L.W&R.W };
}

inline uvec4
operator|(uvec4 L, uvec4 R)
{
    return uvec4{ L.X|R.X, L.Y|R.Y, L.Z|R.Z, L.W|R.W };
}

inline uvec4
operator^(uvec4 L, uvec4 R)
{
    return uvec4{ L.X^R.X, L.Y^R.Y, L.Z^R.Z, L.W^R.W };
}

inline uvec4
operator>>(uvec4 L, uvec4 R)
{
    return uvec4{ L.X>>R.X, L.Y>>R.Y, L.Z>>R.Z, L.W>>R.W };
}

inline uvec4
operator<<(uvec4 L, uvec4 R)
{
    return uvec4{ L.X<<R.X, L.Y<<R.Y, L.Z<<R.Z, L.W<<R.W };
}

inline uvec4
operator~(uvec4 L)
{
    return uvec4{ ~L.X, ~L.Y, ~L.Z, ~L.W };
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
    return std::sqrtf(V.X*V.X + V.Y*V.Y);
}

inline float
Length(vec3 V)
{
    return std::sqrtf(V.X*V.X + V.Y*V.Y + V.Z*V.Z);
}

inline float
Length(vec4 V)
{
    return std::sqrtf(V.X*V.X + V.Y*V.Y + V.Z*V.Z + V.W*V.W);
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
    O.M[0][0] = (L.M[0][0] * R.M[0][0]) + (L.M[0][1] * R.M[1][0]) + (L.M[0][2] * R.M[2][0]) + (L.M[0][3] * R.M[3][0]);
    O.M[0][1] = (L.M[0][0] * R.M[0][1]) + (L.M[0][1] * R.M[1][1]) + (L.M[0][2] * R.M[2][1]) + (L.M[0][3] * R.M[3][1]);
    O.M[0][2] = (L.M[0][0] * R.M[0][2]) + (L.M[0][1] * R.M[1][2]) + (L.M[0][2] * R.M[2][2]) + (L.M[0][3] * R.M[3][2]);
    O.M[0][3] = (L.M[0][0] * R.M[0][3]) + (L.M[0][1] * R.M[1][3]) + (L.M[0][2] * R.M[2][3]) + (L.M[0][3] * R.M[3][3]);

    O.M[1][0] = (L.M[1][0] * R.M[0][0]) + (L.M[1][1] * R.M[1][0]) + (L.M[1][2] * R.M[2][0]) + (L.M[1][3] * R.M[3][0]);
    O.M[1][1] = (L.M[1][0] * R.M[0][1]) + (L.M[1][1] * R.M[1][1]) + (L.M[1][2] * R.M[2][1]) + (L.M[1][3] * R.M[3][1]);
    O.M[1][2] = (L.M[1][0] * R.M[0][2]) + (L.M[1][1] * R.M[1][2]) + (L.M[1][2] * R.M[2][2]) + (L.M[1][3] * R.M[3][2]);
    O.M[1][3] = (L.M[1][0] * R.M[0][3]) + (L.M[1][1] * R.M[1][3]) + (L.M[1][2] * R.M[2][3]) + (L.M[1][3] * R.M[3][3]);

    O.M[2][0] = (L.M[2][0] * R.M[0][0]) + (L.M[2][1] * R.M[1][0]) + (L.M[2][2] * R.M[2][0]) + (L.M[2][3] * R.M[3][0]);
    O.M[2][1] = (L.M[2][0] * R.M[0][1]) + (L.M[2][1] * R.M[1][1]) + (L.M[2][2] * R.M[2][1]) + (L.M[2][3] * R.M[3][1]);
    O.M[2][2] = (L.M[2][0] * R.M[0][2]) + (L.M[2][1] * R.M[1][2]) + (L.M[2][2] * R.M[2][2]) + (L.M[2][3] * R.M[3][2]);
    O.M[2][3] = (L.M[2][0] * R.M[0][3]) + (L.M[2][1] * R.M[1][3]) + (L.M[2][2] * R.M[2][3]) + (L.M[2][3] * R.M[3][3]);

    O.M[3][0] = (L.M[3][0] * R.M[0][0]) + (L.M[3][1] * R.M[1][0]) + (L.M[3][2] * R.M[2][0]) + (L.M[3][3] * R.M[3][0]);
    O.M[3][1] = (L.M[3][0] * R.M[0][1]) + (L.M[3][1] * R.M[1][1]) + (L.M[3][2] * R.M[2][1]) + (L.M[3][3] * R.M[3][1]);
    O.M[3][2] = (L.M[3][0] * R.M[0][2]) + (L.M[3][1] * R.M[1][2]) + (L.M[3][2] * R.M[2][2]) + (L.M[3][3] * R.M[3][2]);
    O.M[3][3] = (L.M[3][0] * R.M[0][3]) + (L.M[3][1] * R.M[1][3]) + (L.M[3][2] * R.M[2][3]) + (L.M[3][3] * R.M[3][3]);

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

    Result.X = std::cosf(Angle / 2.0f);

    Result.Y = Axis.X * std::sinf(Angle / 2.0f);
    Result.Z = Axis.Y * std::sinf(Angle / 2.0f);
    Result.W = Axis.Z * std::sinf(Angle / 2.0f);

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
    return std::sqrtf(Q.X*Q.X + Q.Y*Q.Y + Q.Z*Q.Z + Q.W*Q.W);
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

inline vec3
Rotate(quat Rotation, vec3 V)
{
    vec3 T =  Cross(ImaginaryPart(Rotation), V) * 2.0f;
    vec3 Rotated = (T * RealPart(Rotation)) + Cross(ImaginaryPart(Rotation), T) + V;

    return Rotated;

}
// }}}

}

#endif

