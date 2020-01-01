#ifndef SABRE_MATH
#define SABRE_MATH

// For trig functions
#include <math.h>

#define Pi32  3.14159265f

#define Sqrt2       1.41421569f
#define InvSqrt2    0.70710678f
#define PiOver180Dg 0.01745329f

#define Rads(Deg) ((Deg)*PiOver180Dg)


#define V3(X, Y, Z) { (float)(X), (float)(Y), (float)(Z) }

// {{{ Vectors
// NOTE: COLUMN vector.
//

struct vec3
{
    float X;
    float Y;
    float Z;

    inline vec3() {}

    inline explicit
    vec3(float InX, float InY, float InZ)
    {
        X = InX;
        Y = InY;
        Z = InZ;
    }

    inline explicit
    vec3(int Uniform)
    {
        X = (float) Uniform;
        Y = (float) Uniform;
        Z = (float) Uniform;
    }
};

struct uvec3
{
    u32 X;
    u32 Y;
    u32 Z;

    inline uvec3() {}

    inline explicit
    uvec3(u32 InX, u32 InY, u32 InZ)
    {
        X = InX;
        Y = InY;
        Z = InZ;
    }

    inline 
    operator vec3()
    {
        vec3 Result;

        Result.X = (float) X;
        Result.Y = (float) Y;
        Result.Z = (float) Z;

        return Result;
    }
};

inline vec3 
operator+(vec3 L, vec3 R)
{
    vec3 Result;

    Result.X = L.X + R.X;
    Result.Y = L.Y + R.Y;
    Result.Z = L.Z + R.Z;

    return Result;
}

inline vec3 
operator-(vec3 L, vec3 R)
{
    vec3 Result;

    Result.X = L.X - R.X;
    Result.Y = L.Y - R.Y;
    Result.Z = L.Z - R.Z;

    return Result;
}

inline vec3 
operator/(vec3 V, f32 Scalar)
{
    vec3 Result;

    Result.X = V.X / Scalar;
    Result.Y = V.Y / Scalar;
    Result.Z = V.Z / Scalar;

    return Result;
}

inline vec3
operator*(vec3 L, vec3 R)
{
    vec3 Result;

    Result.X = L.X * R.X;
    Result.Y = L.Y * R.Y;
    Result.Z = L.Z * R.Z;

    return Result;
}

inline bool
operator<(vec3 L, vec3 R)
{
    return (L.X < R.X) && (L.Y < R.Y) && (L.Z < R.Z);
}

inline bool
operator>(vec3 L, vec3 R)
{
    return (L.X > R.X) && (L.Y > R.Y) && (L.Z > R.Z);
}

inline vec3 
operator-(vec3 A)
{
    return vec3(-A.X, -A.Y, -A.Z);
}


inline vec3& 
operator+=(vec3& L, vec3 R)
{
    L = L + R;

    return L;
}


inline vec3& 
operator-=(vec3& L, vec3 R)
{
    L = L - R;

    return L;
}


inline vec3 
operator*(float Scalar, vec3 Vec)
{
    vec3 Result;

    Result.X = Scalar * Vec.X;
    Result.Y = Scalar * Vec.Y;
    Result.Z = Scalar * Vec.Z;

    return Result;
}

inline vec3
operator*(vec3 V, int Scalar)
{
    vec3 Result;

    Result.X = Scalar * V.X;
    Result.Y = Scalar * V.Y;
    Result.Z = Scalar * V.Z;

    return Result;
}


static inline vec3 
Cross(vec3 U, vec3 V)
{
    vec3 Result;

    Result.X = (U.Y * V.Z) - (U.Z * V.Y);
    Result.Y = (U.Z * V.X) - (U.X * V.Z);
    Result.Z = (U.X * V.Y) - (U.Y * V.X);

    return Result;
}

static inline float
Dot(vec3 U, vec3 V)
{
    float Result = (U.X * V.X) + (U.Y * V.Y) + (U.Z * V.Z);
    return Result;
}

static inline vec3 
Normalize(vec3 V)
{
    const float L = sqrtf(V.X*V.X + V.Y*V.Y + V.Z*V.Z);

    if (L == 0.0f)
    { 
        return vec3(0.0f, 0.0f, 0.0f);
    }
    else
    {
        vec3 Result;

        Result.X = V.X / L;
        Result.Y = V.Y / L;
        Result.Z = V.Z / L;

        return Result;
    }
}

static inline float 
Length(vec3 V)
{
    return sqrtf(V.X*V.X + V.Y*V.Y + V.Z*V.Z);
}

// }}}


// {{{ Integer vectors

inline uvec3 
operator+(uvec3 Left, uvec3 Right)
{
    uvec3 Result;

    Result.X = Left.X + Right.X;
    Result.Y = Left.Y + Right.Y;
    Result.Z = Left.Z + Right.Z;

    return Result;
}

inline uvec3 
operator*(uvec3 L, u32 R)
{
    uvec3 Result;

    Result.X = L.X * R;
    Result.Y = L.Y * R;
    Result.Z = L.Z * R;

    return Result;
}


// }}}

#endif

