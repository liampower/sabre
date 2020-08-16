#include "sabre.h"
#include "vecmath.h"
#include "svo.h"
#include "noise_gen.h"

using namespace vm;

// Threshold for noise value to be considered "solid"
static constexpr float NOISE_THRESHOLD = 0.4f;

static constexpr float SCALE_FACTOR = 256.0f;

// Randomly distributed permutation of numbers 0-255
static const unsigned char Pt[512] = {
    151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,
    225,140,36,103,30,69,142,8,99,37,240,21,10,23,190, 
    6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,
    35,11,32,57,177,33, 88,237,149,56,87,174,20,125,136,
    171,168, 68,175,74,165,71,134,139,48,27,166, 77,146,
    158,231,83,111,229,122,60,211,133,230,220,105,92,41,
    55,46,245,40,244, 102,143,54, 65,25,63,161, 1,216,80,
    73,209,76,132,187,208, 89,18,169,200,196,135,130,116,
    188,159,86,164,100,109,198,173,186, 3,64,52,217,226,
    250,124,123,5,202,38,147,118,126,255,82,85,212,207,
    206,59,227,47,16,58,17,182,189,28,42,223,183,170,
    213,119,248,152, 2,44,154,163, 70,221,153,101,155,
    167, 43,172,9,129,22,39,253, 19,98,108,110,79,113,224,
    232,178,185, 112,104,218,246,97,228, 251,34,242,193,
    238,210,144,12,191,179,162,241, 81,51,145,235,249,14,
    239,107,49,192,214, 31,181,199,106,157,184, 84,204,
    176,115,121,50,45,127, 4,150,254,138,236,205,93,222,
    114,67,29,24,72,243,141,128,195,78,66,215,61,156,180,
    151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,
    225,140,36,103,30,69,142,8,99,37,240,21,10,23,190, 
    6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,
    35,11,32,57,177,33, 88,237,149,56,87,174,20,125,136,
    171,168, 68,175,74,165,71,134,139,48,27,166, 77,146,
    158,231,83,111,229,122,60,211,133,230,220,105,92,41,
    55,46,245,40,244, 102,143,54, 65,25,63,161, 1,216,80,
    73,209,76,132,187,208, 89,18,169,200,196,135,130,116,
    188,159,86,164,100,109,198,173,186, 3,64,52,217,226,
    250,124,123,5,202,38,147,118,126,255,82,85,212,207,
    206,59,227,47,16,58,17,182,189,28,42,223,183,170,
    213,119,248,152, 2,44,154,163, 70,221,153,101,155,
    167, 43,172,9,129,22,39,253, 19,98,108,110,79,113,224,
    232,178,185, 112,104,218,246,97,228, 251,34,242,193,
    238,210,144,12,191,179,162,241, 81,51,145,235,249,14,
    239,107,49,192,214, 31,181,199,106,157,184, 84,204,
    176,115,121,50,45,127, 4,150,254,138,236,205,93,222,
    114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
};


static constexpr inline float
Smootherstep(float X)
{
    return X*X*X * (X*(X*6.0f - 15.0f) + 10.0f);
}

static constexpr inline float
Smootherstep1(float X)
{
    // First order derivative of the smootherstep function
    // S'(x) = 30x^4 - 60x^3 - 30x^2
    return X*X * (30*(X*X - 2*X + 1));
}


static constexpr inline float
Smoothstep(float X)
{
  return X * X * (3 - 2 * X);
}

static constexpr inline float
Smoothstep1(float X)
{
    return 6*X*(1 - X);
}

static constexpr inline float
Lerp(float Edge0, float Edge1, float X)
{
    return Edge0 + ((Edge1 - Edge0)*X);
}

static inline float
GradDotDst(int Hash, float X, float Y, float Z)
{
    switch (Hash & 0xF)
    {
        case 0x0: return  X + Y;
        case 0x1: return -X + Y;
        case 0x2: return  X - Y;
        case 0x3: return -X - Y;
        case 0x4: return  X + Z;
        case 0x5: return  X - Z;
        case 0x6: return -X + Z;
        case 0x7: return -X - Z;
        case 0x8: return  Y + Z;
        case 0x9: return -Y + Z;
        case 0xA: return  Y - Z;
        case 0xB: return -Y - Z;
        case 0xC: return  Y + X;
        case 0xD: return -Y + X;
        case 0xE: return  Y - X;
        case 0xF: return -Y - Z;
        default: return 0; // Shouldn't happen
    }
}

static constexpr inline float
Fastfloor(float X)
{
    return float(int(X));
}


static inline vec3
Perlin3Deriv(vec3 P)
{
    float X = P.X / SCALE_FACTOR;
    float Y = P.Y / SCALE_FACTOR;
    float Z = P.Z / SCALE_FACTOR;

    // Unit cube Left-Bottom-Back coordinates
    int Xi = (int(X) & 255);
    int Yi = (int(Y) & 255);
    int Zi = (int(Z) & 255);

    // Coords of our point inside its unit cube.
    float Xf = X - Fastfloor(X);
    float Yf = Y - Fastfloor(Y);
    float Zf = Z - Fastfloor(Z);

    // Smooth point coords towards integral values.
    // This produces smoother noise output.
    float U = Smoothstep(Xf);
    float V = Smoothstep(Yf);
    float W = Smoothstep(Zf);

    // Derivative smootherstep values
    float Du = Smoothstep1(Xf);
    float Dv = Smoothstep1(Yf);
    float Dw = Smoothstep1(Zf);

    // Hash values for the corners of the unit cube.
    // Named according to their positions - assume we start
    // at (0, 0, 0) and the cube sides are unit length.
    int C000, C010, C001, C011, C100, C110, C101, C111;

    // Hash values are calculted using the permutation
    // table.
    C000 = Pt[Pt[Pt[Xi] + Yi] + Zi];
    C010 = Pt[Pt[Pt[Xi] + Yi + 1] + Zi];
    C001 = Pt[Pt[Pt[Xi] + Yi] + Zi + 1];
    C011 = Pt[Pt[Pt[Xi] + Yi + 1] + Zi + 1];
    C100 = Pt[Pt[Pt[Xi + 1] + Yi] + Zi];
    C110 = Pt[Pt[Pt[Xi + 1] + Yi + 1] + Zi];
    C101 = Pt[Pt[Pt[Xi + 1] + Yi] + Zi + 1];
    C111 = Pt[Pt[Pt[Xi + 1] + Yi + 1] + Zi + 1];

    float A = GradDotDst(C000, Xf, Yf, Zf);
    float B = GradDotDst(C100, Xf - 1.0f, Yf, Zf);
    float C = GradDotDst(C010, Xf, Yf - 1.0f, Zf);
    float D = GradDotDst(C110, Xf - 1.0f, Yf - 1.0f, Zf);
    float E = GradDotDst(C001, Xf, Yf, Zf - 1.0f);
    float F = GradDotDst(C101, Xf - 1.0f, Yf, Zf - 1.0f);
    float G = GradDotDst(C011, Xf, Yf - 1.0f, Zf - 1.0f);
    float H = GradDotDst(C111, Xf - 1.0f, Yf - 1.0f, Zf - 1.0f);

    float K0 = (B - A);
    float K1 = (C - A);
    float K2 = (E - A);
    float K3 = (A + D - B - C);
    float K4 = (A + F - B - E);
    float K5 = (A + G - C - E);
    float K6 = (B + C + E + H - A - D - F - G);

    // Noise partial derivatives in X, Y, Z directions
    float Dx = Du * (K0 + V*K3 + W*K4 + V*W*K6);
    float Dy = Dv * (K1 + U*K3 + W*K5 + U*W*K6);
    float Dz = Dw * (K2 + U*K4 + V*K5 + U*V*K6);

    return vec3(Dx, Dy, Dz);
}

static inline bool
Perlin3(vec3 P, vec3 Min, vec3 Max)
{
    float X = P.X / SCALE_FACTOR;
    float Y = P.Y / SCALE_FACTOR;
    float Z = P.Z / SCALE_FACTOR;

    // Unit cube Left-Bottom-Back coordinates
    int Xi = (int(X) & 255);
    int Yi = (int(Y) & 255);
    int Zi = (int(Z) & 255);

    // Coords of our point inside its unit cube.
    float Xf = X - Fastfloor(X);
    float Yf = Y - Fastfloor(Y);
    float Zf = Z - Fastfloor(Z);

    // Smooth point coords towards integral values.
    // This produces smoother noise output.
    float U = Smoothstep(Xf);
    float V = Smoothstep(Yf);
    float W = Smoothstep(Zf);

    // Derivative smootherstep values
    float Du = Smoothstep1(Xf);
    float Dv = Smoothstep1(Yf);
    float Dw = Smoothstep1(Zf);

    // Hash values for the corners of the unit cube.
    // Named according to their positions - assume we start
    // at (0, 0, 0) and the cube sides are unit length.
    int C000, C010, C001, C011, C100, C110, C101, C111;

    // Hash values are calculted using the permutation
    // table.
    C000 = Pt[Pt[Pt[Xi] + Yi] + Zi];
    C010 = Pt[Pt[Pt[Xi] + Yi + 1] + Zi];
    C001 = Pt[Pt[Pt[Xi] + Yi] + Zi + 1];
    C011 = Pt[Pt[Pt[Xi] + Yi + 1] + Zi + 1];
    C100 = Pt[Pt[Pt[Xi + 1] + Yi] + Zi];
    C110 = Pt[Pt[Pt[Xi + 1] + Yi + 1] + Zi];
    C101 = Pt[Pt[Pt[Xi + 1] + Yi] + Zi + 1];
    C111 = Pt[Pt[Pt[Xi + 1] + Yi + 1] + Zi + 1];

    float A = GradDotDst(C000, Xf, Yf, Zf);
    float B = GradDotDst(C100, Xf - 1.0f, Yf, Zf);
    float C = GradDotDst(C010, Xf, Yf - 1.0f, Zf);
    float D = GradDotDst(C110, Xf - 1.0f, Yf - 1.0f, Zf);
    float E = GradDotDst(C001, Xf, Yf, Zf - 1.0f);
    float F = GradDotDst(C101, Xf - 1.0f, Yf, Zf - 1.0f);
    float G = GradDotDst(C011, Xf, Yf - 1.0f, Zf - 1.0f);
    float H = GradDotDst(C111, Xf - 1.0f, Yf - 1.0f, Zf - 1.0f);

    float K0 = (B - A);
    float K1 = (C - A);
    float K2 = (E - A);
    float K3 = (A + D - B - C);
    float K4 = (A + F - B - E);
    float K5 = (A + G - C - E);
    float K6 = (B + C + E + H - A - D - F - G);

    // Noise partial derivatives in X, Y, Z directions
    float Dx = Du * (K0 + V*K3 + W*K4 + V*W*K6);
    float Dy = Dv * (K1 + U*K3 + W*K5 + U*W*K6);
    float Dz = Dw * (K2 + U*K4 + V*K5 + U*V*K6);

    float N = A + U * K0 + V * K1 + W * K2 + U * V * K3 + U * W * K4 + V * W * K5 + U * V * W * K6; 

    /*float X1, X2, Y1, Y2;
    X1 = Lerp(GradDotDst(C000, Xf, Yf, Zf), GradDotDst(C100, Xf - 1.0f, Yf, Zf), U);
    X2 = Lerp(GradDotDst(C010, Xf, Yf - 1.0f, Zf), GradDotDst(C110, Xf - 1.0f, Yf - 1.0f, Zf), U);
    Y1 = Lerp(X1, X2, V);

    X1 = Lerp(GradDotDst(C001, Xf, Yf, Zf - 1.0f), GradDotDst(C101, Xf - 1.0f, Yf, Zf - 1.0f), U);
    X2 = Lerp(GradDotDst(C011, Xf, Yf - 1.0f, Zf - 1.0f), GradDotDst(C111, Xf -1.0f, Yf - 1.0f, Zf - 1.0f), U);
    Y2 = Lerp(X1, X2, V);

    return (Lerp(Y1, Y2, W) + 1) / 2.0f;*/


    vec3 CS = vec3((K0 + V*K3 + W*K4 + V*W*K6),
                   (K1 + U*K3 + W*K5 + U*W*K6),
                   (K2 + U*K4 + V*K5 + U*V*K6));


    vec3 Diff = (Max - Min) * 0.5f;
    vec3 MaxDN = Abs(vec3(Smoothstep1(0.5f))*CS)*Diff; // Maximum variance of noise

    vec3 MaxVal = vec3(N) + MaxDN;
    vec3 MinVal = vec3(N) - MaxDN;

    // Min <= T <= Max
    if (Any(GreaterThan(vec3(NOISE_THRESHOLD), MinVal) && LessThan(vec3(NOISE_THRESHOLD), MaxVal)))
    {
        return true;
    }
    else
    {
        return false;
    }

}

static inline bool
ShapeSamplerFn(vec3 Min, vec3 Max, const svo* const Tree, const void* const)
{
    vec3 H = (Max - Min) * 0.5f;
    vec3 C = ((Min + H) * f32(1 << Tree->Bias.Scale));
    //return Perlin3(C, Min, Max);

    return Perlin3(C, Min, Max);
}


static inline vec3
DummySampler(vec3 C, const svo* const, const void* const)
{
    vec3 N = Perlin3Deriv(C);
    vec3 Tan{1, N.X, 0};
    vec3 BiTan{0, N.Z, 1};

    return Normalize(N);
}


extern svo*
BuildNoiseSvo(u32 MaxDepth, u32 ScaleExp)
{
    shape_sampler S{ nullptr, &ShapeSamplerFn };
    data_sampler D{ nullptr, &DummySampler };

    svo* Svo = CreateScene(ScaleExp, MaxDepth, &S, &D, &D);


    return Svo;
}
