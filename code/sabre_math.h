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
#endif

