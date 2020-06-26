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


#if defined(__LLVM__) || defined(_MSC_VER)
  #define vcall __vectorcall
#endif

#if defined(__LLVM__)
  #define inline __attribute__((always_inline))
  #define aligned(N) __attribute__((aligned(N)))
#endif




static inline float
Clamp(float X, float Lo, float Hi)
{
    if (X < Lo) return Lo;
    if (X > Hi) return Hi;
    else        return X;
}

static inline int
Round(float X)
{
    return (int)roundf(X);
}

struct bvec3
{
    bool X;
    bool Y;
    bool Z;
};

struct mat3
{
    float M[3][3];
};


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

    inline explicit
    vec3(u32 Uniform)
    {
        X = (float) Uniform;
        Y = (float) Uniform;
        Z = (float) Uniform;
    }

    inline explicit
    vec3(float Uniform)
    {
        X = Uniform;
        Y = Uniform;
        Z = Uniform;
    }

    inline
    vec3(const vec3& Copy)
    {
        X = Copy.X;
        Y = Copy.Y;
        Z = Copy.Z;
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

    inline explicit
    uvec3(bvec3 B)
    {
        X = (u32)B.X;
        Y = (u32)B.Y;
        Z = (u32)B.Z;
    }

    inline explicit
    uvec3(vec3 V)
    {
        X = (u32)V.X;
        Y = (u32)V.Y;
        Z = (u32)V.Z;
    }

    inline explicit
    uvec3(u32 Uniform)
    {
        X = Uniform;
        Y = Uniform;
        Z = Uniform;
    }

    inline explicit
    operator vec3()
    {
        vec3 Result;

        Result.X = (float) X;
        Result.Y = (float) Y;
        Result.Z = (float) Z;

        return Result;
    }

    inline explicit
    operator bvec3()
    {
        bvec3 Result;
        
        Result.X = (X != 0);
        Result.Y = (Y != 0);
        Result.Z = (Z != 0);

        return Result;
    }
};

static inline uvec3
operator%(uvec3 L, uvec3 R)
{
    uvec3 Result;

    Result.X = L.X % R.X;
    Result.Y = L.Y % R.Y;
    Result.Z = L.Z % R.Z;

    return Result;
}

static inline uvec3
operator/(uvec3 L, uvec3 R)
{
    uvec3 Result;

    Result.X = L.X / R.X;
    Result.Y = L.Y / R.Y;
    Result.Z = L.Z / R.Z;

    return Result;
}

static inline bool
operator==(uvec3 L, uvec3 R)
{
    return L.X == R.X && L.Y == R.Y && L.Z == R.Z;
}

static inline uvec3
operator-(uvec3 L, uvec3 R)
{
    uvec3 Result;

    Result.X = L.X - R.X;
    Result.Y = L.Y - R.Y;
    Result.Z = L.Z - R.Z;

    return Result;
}

static inline uvec3
operator*(uvec3 L, uvec3 R)
{
    uvec3 Result;

    Result.X = L.X * R.X;
    Result.Y = L.Y * R.Y;
    Result.Z = L.Z * R.Z;

    return Result;
}

struct ivec3
{
    i32 X;
    i32 Y;
    i32 Z;

    inline explicit
    ivec3(i32 InX, i32 InY, i32 InZ)
    {
        X = InX;
        Y = InY;
        Z = InZ;
    }

    inline explicit
    ivec3(i32 Uniform)
    {
        X = Uniform;
        Y = Uniform;
        Z = Uniform;
    }

    inline explicit
    ivec3(bvec3 B)
    {
        X = (i32)B.X;
        Y = (i32)B.Y;
        Z = (i32)B.Z;
    }

    inline explicit
    ivec3(vec3 B)
    {
        X = (i32)B.X;
        Y = (i32)B.Y;
        Z = (i32)B.Z;
    }

    inline
    operator bvec3()
    {
        bvec3 Result;
        
        Result.X = (X != 0);
        Result.Y = (Y != 0);
        Result.Z = (Z != 0);

        return Result;
    }
};


inline ivec3
operator-(ivec3 L, ivec3 R)
{
    ivec3 Result = ivec3(0);

    Result.X = L.X - R.X;
    Result.Y = L.Y - R.Y;
    Result.Z = L.Z - R.Z;

    return Result;
}


static inline u32
Dot(uvec3 A, uvec3 B)
{
    return A.X*B.X + A.Y*B.Y + A.Z*B.Z;
}


static inline vec3
Invert(vec3 V)
{
    vec3 Out = vec3(0.0f);

    if (V.X != 0.0f) Out.X = 1.0f / V.X;
    if (V.Y != 0.0f) Out.Y = 1.0f / V.Y;
    if (V.Z != 0.0f) Out.Z = 1.0f / V.Z;

    return Out;
}


static inline uvec3
Clamp(uvec3 Val, u32 Min, u32 Max)
{
    uvec3 Out = Val;

    if (Out.X > Max) Out.X = Max;
    if (Out.X < Min) Out.X = Min;

    if (Out.Y > Max) Out.Y = Max;
    if (Out.Y < Min) Out.Y = Min;

    if (Out.Z > Max) Out.Z = Max;
    if (Out.Z < Min) Out.Z = Min;

    return Out;
}

static inline float
Min(float A, float B)
{
    return (A > B) ? B : A;
}

static inline float
Max(float A, float B)
{
    return (A > B) ? A : B;
}

static inline uint32_t
Min(uint32_t A, uint32_t B)
{
    return (A > B) ? B : A;
}

static inline uint32_t
Max(uint32_t A, uint32_t B)
{
    return (A > B) ? A : B;
}


static inline vec3
Min(vec3 A, vec3 B)
{
    vec3 Out;

    Out.X = Min(A.X, B.X);
    Out.Y = Min(A.Y, B.Y);
    Out.Z = Min(A.Z, B.Z);

    return Out;
}

static inline vec3
Max(vec3 A, vec3 B)
{
    vec3 Out;

    Out.X = Max(A.X, B.X);
    Out.Y = Max(A.Y, B.Y);
    Out.Z = Max(A.Z, B.Z);

    return Out;
}

static inline uvec3
Min(uvec3 A, uvec3 B)
{
    uvec3 Out;

    Out.X = Min(A.X, B.X);
    Out.Y = Min(A.Y, B.Y);
    Out.Z = Min(A.Z, B.Z);

    return Out;
}

static inline uvec3
Max(uvec3 A, uvec3 B)
{
    uvec3 Out;

    Out.X = Max(A.X, B.X);
    Out.Y = Max(A.Y, B.Y);
    Out.Z = Max(A.Z, B.Z);

    return Out;
}

static inline bool
Any(bvec3 V)
{
    return (V.X || V.Y || V.Z);
}

static inline bool
All(bvec3 V)
{
    return (V.X && V.Y && V.Z);
}

static inline bvec3
GreaterThan(vec3 A, vec3 B)
{
    bvec3 Out;

    Out.X = A.X > B.X;
    Out.Y = A.Y > B.Y;
    Out.Z = A.Z > B.Z;

    return Out;
}

static inline bvec3
GreaterThan(uvec3 A, uvec3 B)
{
    bvec3 Out;

    Out.X = A.X > B.X;
    Out.Y = A.Y > B.Y;
    Out.Z = A.Z > B.Z;

    return Out;
}

static inline bvec3
GreaterThanEqual(uvec3 A, uvec3 B)
{
    bvec3 Out;

    Out.X = A.X >= B.X;
    Out.Y = A.Y >= B.Y;
    Out.Z = A.Z >= B.Z;

    return Out;
}

static inline bvec3
GreaterThanEqual(vec3 A, vec3 B)
{
    bvec3 Out;

    Out.X = A.X >= B.X;
    Out.Y = A.Y >= B.Y;
    Out.Z = A.Z >= B.Z;

    return Out;
}


static inline bvec3
LessThan(vec3 A, vec3 B)
{
    bvec3 Out;

    Out.X = A.X < B.X;
    Out.Y = A.Y < B.Y;
    Out.Z = A.Z < B.Z;

    return Out;
}

static inline bvec3
LessThan(uvec3 A, uvec3 B)
{
    bvec3 Out;

    Out.X = A.X < B.X;
    Out.Y = A.Y < B.Y;
    Out.Z = A.Z < B.Z;

    return Out;
}


static inline bvec3
LessThanEqual(vec3 A, vec3 B)
{
    bvec3 Out;

    Out.X = A.X < B.X;
    Out.Y = A.Y < B.Y;
    Out.Z = A.Z < B.Z;

    return Out;
}

static inline bvec3
operator&&(bvec3 L, bvec3 R)
{
    bvec3 Out;

    Out.X = L.X && R.X;
    Out.Y = L.Y && R.Y;
    Out.Z = L.Z && R.Z;

    return Out;
}

static inline bvec3
operator||(bvec3 L, bvec3 R)
{
    bvec3 Out;

    Out.X = L.X || R.X;
    Out.Y = L.Y || R.Y;
    Out.Z = L.Z || R.Z;

    return Out;
}

static inline bvec3
Equals(vec3 A, vec3 B, float Tolerance)
{
    bvec3 Out;

    Out.X = fabsf(B.X - A.X) <= Tolerance;
    Out.Y = fabsf(B.Y - A.Y) <= Tolerance;
    Out.Z = fabsf(B.Z - A.Z) <= Tolerance;

    return Out;
}

static inline bvec3
Equals(uvec3 A, uvec3 B)
{
    bvec3 Out;

    Out.X = A.X == B.X;
    Out.Y = A.Y == B.Y;
    Out.Z = A.Z == B.Z;

    return Out;
}

static inline float
Sign(float V)
{
    if (V > 0) return 1.0f;
    if (V < 0) return -1.0f;
    else       return 0.0f;
}


static inline ivec3
operator&(ivec3 L, ivec3 R)
{
    return ivec3(L.X & R.X, L.Y & R.Y, L.Z & R.Z);
}

static inline vec3
Sign(ivec3 V)
{
    vec3 Out;

    Out.X = Sign(V.X);
    Out.Y = Sign(V.Y);
    Out.Z = Sign(V.Z);

    return Out;
}

static inline vec3
Sign(vec3 V)
{
    vec3 Out;

    Out.X = Sign(V.X);
    Out.Y = Sign(V.Y);
    Out.Z = Sign(V.Z);

    return Out;
}

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
operator*(vec3 Vec, float Scalar)
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

inline vec3
operator*(vec3 V, unsigned int Scalar)
{
    vec3 Result;

    Result.X = Scalar * V.X;
    Result.Y = Scalar * V.Y;
    Result.Z = Scalar * V.Z;

    return Result;
}

inline vec3
operator*=(vec3& L, float R)
{
    L = L * R;

    return L;
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

static inline uvec3
Select(uvec3 U, uvec3 V, bvec3 Mask)
{
    uvec3 Out;
    Out.X = (Mask.X ? U.X : V.X);
    Out.Y = (Mask.Y ? U.Y : V.Y);
    Out.Z = (Mask.Z ? U.Z : V.Z);

    return Out;
}

static inline vec3
operator/(float K, vec3 V)
{
    vec3 Out;
    Out.X = K / V.X;
    Out.Y = K / V.Y;
    Out.Z = K / V.Z;

    return Out;
}

static inline u32
HorzMax(uvec3 V)
{
    return Max(Max(V.X, V.Y), V.Z);
}

static inline float
HorzMax(vec3 V)
{
    return Max(Max(V.X, V.Y), V.Z);
}

static inline u32
HorzMin(uvec3 V)
{
    return Min(Min(V.X, V.Y), V.Z);
}

static inline float
HorzMin(vec3 V)
{
    return Min(Min(V.X, V.Y), V.Z);
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

static inline float
LengthSq(vec3 V)
{
    return (V.X*V.X + V.Y*V.Y + V.Z*V.Z);
}

static inline float
MaxComponent(vec3 V)
{
    return Max(Max(V.X, V.Y), V.Z);
}

static inline float
MinComponent(vec3 V)
{
    return Min(Min(V.X, V.Y), V.Z);
}


// }}}

// {{{ Matrices
// NOTE: ROW-MAJOR matrix. This means that when passing these
// matrices to OpenGL via uniforms you will need to set the 
// 'transpose' parameter to GL_TRUE.
struct m4x4
{
    float M[4][4];
};



static inline m4x4
IdentityMatrix()
{
    m4x4 I = {{
        { 1.0f, 0.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f, 1.0f }
    }};

    return I;
}

static inline m4x4 vcall
TranslationMatrix(const vec3 Translation)
{
    m4x4 T = IdentityMatrix();
    
    T.M[0][3] = Translation.X;
    T.M[1][3] = Translation.Y;
    T.M[2][3] = Translation.Z;

    return T;
}

static inline m4x4 vcall
TranslationMatrix(const uvec3 Translation)
{
    m4x4 T = IdentityMatrix();
    
    T.M[0][3] = Translation.X;
    T.M[1][3] = Translation.Y;
    T.M[2][3] = Translation.Z;

    return T;
}

static inline m4x4 vcall
TranslationMatrix(u32 X, u32 Y, u32 Z)
{

    m4x4 T = IdentityMatrix();
    
    T.M[0][3] = X;
    T.M[1][3] = Y;
    T.M[2][3] = Z;

    return T;
}

static inline void vcall
Translate3D(m4x4* Matrix, const vec3 Translation)
{
    Matrix->M[0][3] = Translation.X;
    Matrix->M[1][3] = Translation.Y;
    Matrix->M[2][3] = Translation.Z;
}

static inline vec3 vcall
operator*(const vec3& V, const mat3& M)
{
    vec3 Out;

    // Matrix indexed as [row][col]
    Out.X = V.X*M.M[0][0] + V.Y*M.M[1][0] + V.Z*M.M[2][0];
    Out.Y = V.X*M.M[0][1] + V.Y*M.M[1][1] + V.Z*M.M[2][1];
    Out.Z = V.X*M.M[0][2] + V.Y*M.M[1][2] + V.Z*M.M[2][2];

    return Out;
}


// NOTE: The projection is specified to be in the NEGATIVE z direction.
// 'CotHalfFov' means '1.0 / tan(Fov/2)'
static inline m4x4
PerspectiveProjection(float CotHalfFov, float AspectRatio, float Zn, float Zf)
{
    const float ScaleX = CotHalfFov;
    const float ScaleY = CotHalfFov * AspectRatio;
    const float ScaleZ = -(Zf + Zn) / (Zf - Zn);
    const float OffsetZ = -(2.0f * Zn * Zf) / (Zf - Zn);
    const float ScaleW = -1.0f;

    m4x4 P = {{
        { ScaleX, 0.0f,   0.0f,   0.0f },
        { 0.0f,   ScaleY, 0.0f,   0.0f },
        { 0.0f,   0.0f,   ScaleZ, OffsetZ },
        { 0.0f,   0.0f,   ScaleW,   0.0f } 
    }};

    return P;
}

static inline m4x4
operator*(m4x4 L, m4x4 R)
{
    m4x4 O;
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
operator^(uvec3 L, uvec3 R)
{
    uvec3 Result;

    Result.X = L.X ^ R.X;
    Result.Y = L.Y ^ R.Y;
    Result.Z = L.Z ^ R.Z;

    return Result;
}

inline uvec3
operator&(uvec3 L, uvec3 R)
{
    uvec3 Result;

    Result.X = L.X & R.X;
    Result.Y = L.Y & R.Y;
    Result.Z = L.Z & R.Z;

    return Result;
}

inline uvec3
operator>>(uvec3 L, uvec3 R)
{
    uvec3 Result;

    Result.X = L.X >> R.X;
    Result.Y = L.Y >> R.Y;
    Result.Z = L.Z >> R.Z;

    return Result;
}

inline uvec3
operator&(uvec3 L, uint Scalar)
{
    uvec3 Result;

    Result.X = L.X & Scalar;
    Result.Y = L.Y & Scalar;
    Result.Z = L.Z & Scalar;

    return Result;
}

inline uvec3
operator<<(uvec3 L, uvec3 R)
{
    uvec3 Result;

    Result.X = L.X << R.X;
    Result.Y = L.Y << R.Y;
    Result.Z = L.Z << R.Z;

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

// {{{ Quaternion
// NOTE: Quaternion real component is stored in the X-value
struct quat
{
    float X, Y, Z, W;
};

inline quat 
operator*(quat L, quat R)
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
    quat Result;

    //         left                right
    //   (a + bi + cj + dk) * (w + xi + yj + zk)
    //    LX  LY   LZ   LW     RX  RY   RZ   RW
    Result.X = (L.X * R.X) - (L.Y * R.Y) - (L.Z * R.Z) - (L.W * R.W);
    Result.Y = (L.X * R.Y) + (L.Y * R.X) + (L.Z * R.W) - (L.W * R.Z); 
    Result.Z = (L.X * R.Z) - (L.Y * R.W) + (L.Z * R.X) + (L.W * R.Y);
    Result.W = (L.X * R.W) + (L.Y * R.Z) - (L.Z * R.Y) + (L.W * R.X);

    return Result;
}

static inline quat
RotationQuaternion(const float Angle, vec3 Axis)
{
    quat Result;

    Result.X = cosf(Angle / 2.0f);

    Result.Y = Axis.X * sinf(Angle / 2.0f);
    Result.Z = Axis.Y * sinf(Angle / 2.0f);
    Result.W = Axis.Z * sinf(Angle / 2.0f);

    return Result;
}

static inline quat
Conjugate(quat Q)
{
    quat Conj;

    Conj.X = Q.X;

    Conj.Y = -Q.Y;
    Conj.Z = -Q.Z;
    Conj.W = -Q.W;

    return Conj;
}

static inline float
QuatLength(quat Q)
{
    return sqrtf(Q.X*Q.X + Q.Y*Q.Y + Q.Z*Q.Z + Q.W*Q.W);
}

static inline quat
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

static inline vec3
ImaginaryPart(quat Q)
{
    return vec3(Q.Y, Q.Z, Q.W);
}

static inline float
RealPart(quat Q)
{
    return Q.X;
}

// TODO Needs optimization!
// There is a better way invented by Fabien 'ryg' Giesen.
static inline vec3
Rotate(quat Rotation, vec3 V)
{
    /*vec3 Result;
    Normalize(Rotation);
    const quat Conj = Conjugate(Rotation);
    const quat V = {{ 0.0f, P.X, P.Y, P.Z }};

    quat Rotated = Rotation * V * Conj;

    Result.X = Rotated.Y;
    Result.Y = Rotated.Z;
    Result.Z = Rotated.W;

    return Result;*/
    vec3 T = 2.0f * Cross(ImaginaryPart(Rotation), V);
    vec3 Rotated = V + RealPart(Rotation) * T + Cross(ImaginaryPart(Rotation), T);

    return Rotated;

}

// }}}

static inline u32
Part1By2_32(u32 X)
{
    X &= 0X000003ff;                  // X = ---- ---- ---- ---- ---- --98 7654 3210
    X = (X ^ (X << 16U)) & 0Xff0000ff; // X = ---- --98 ---- ---- ---- ---- 7654 3210
    X = (X ^ (X <<  8U)) & 0X0300f00f; // X = ---- --98 ---- ---- 7654 ---- ---- 3210
    X = (X ^ (X <<  4U)) & 0X030c30c3; // X = ---- --98 ---- 76-- --54 ---- 32-- --10
    X = (X ^ (X <<  2U)) & 0X09249249; // X = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0

    return X;
}

static inline u32
EncodeMorton3_32(uvec3 V)
{
    return (Part1By2_32(V.Z) << 2U) + (Part1By2_32(V.Y) << 1U) + Part1By2_32(V.X);
}

#endif

