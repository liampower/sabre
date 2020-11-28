#include "sabre.hh"
#include "vecmath.hh"
#include "svo.hh"
#include "noise_gen.hh"

using namespace vm;

// Noise values >= this constant are considered to be "solid"
static constexpr f32 NOISE_THRESHOLD = 0.6f;
static constexpr f32 SCALE_FACTOR = 1.0f/257.0f;

// Randomly distributed permutation of numbers 0-255
static constexpr u8 Pt[512] = {
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


static constexpr inline f32
Smootherstep(f32 X)
{
    return X*X*X * (X*(X*6.0f - 15.0f) + 10.0f);
}

static constexpr inline f32
Smootherstep1(f32 X)
{
    // First order derivative of the smootherstep function
    // S'(x) = 30x^4 - 60x^3 - 30x^2
    return X*X * (30*(X*X - 2*X + 1));
}


static constexpr inline f32
Smoothstep(f32 X)
{
  return X * X * (3 - 2 * X);
}

static constexpr inline f32
Smoothstep1(f32 X)
{
    return 6*X*(1 - X);
}

static constexpr inline f32
Lerp(f32 Edge0, f32 Edge1, f32 X)
{
    return Edge0 + ((Edge1 - Edge0)*X);
}

static inline f32
GradDotDst(int Hash, f32 X, f32 Y, f32 Z)
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

static constexpr inline f32
Fastfloor(f32 X)
{
    return f32(int(X));
}


static inline vec3
Perlin3Deriv(vec3 P)
{
    f32 X = P.X * SCALE_FACTOR;
    f32 Y = P.Y * SCALE_FACTOR;
    f32 Z = P.Z * SCALE_FACTOR;

    // Unit cube Left-Bottom-Back coordinates
    int Xi = (int(X) & 255);
    int Yi = (int(Y) & 255);
    int Zi = (int(Z) & 255);

    // Coords of our point inside its unit cube.
    f32 Xf = X - Fastfloor(X);
    f32 Yf = Y - Fastfloor(Y);
    f32 Zf = Z - Fastfloor(Z);

    // Smooth point coords towards integral values.
    // This produces smoother noise output.
    f32 U = Smoothstep(Xf);
    f32 V = Smoothstep(Yf);
    f32 W = Smoothstep(Zf);

    // Derivative smootherstep values
    f32 Du = Smoothstep1(Xf);
    f32 Dv = Smoothstep1(Yf);
    f32 Dw = Smoothstep1(Zf);

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

    f32 A = GradDotDst(C000, Xf, Yf, Zf);
    f32 B = GradDotDst(C100, Xf - 1.0f, Yf, Zf);
    f32 C = GradDotDst(C010, Xf, Yf - 1.0f, Zf);
    f32 D = GradDotDst(C110, Xf - 1.0f, Yf - 1.0f, Zf);
    f32 E = GradDotDst(C001, Xf, Yf, Zf - 1.0f);
    f32 F = GradDotDst(C101, Xf - 1.0f, Yf, Zf - 1.0f);
    f32 G = GradDotDst(C011, Xf, Yf - 1.0f, Zf - 1.0f);
    f32 H = GradDotDst(C111, Xf - 1.0f, Yf - 1.0f, Zf - 1.0f);

    f32 K0 = (B - A);
    f32 K1 = (C - A);
    f32 K2 = (E - A);
    f32 K3 = (A + D - B - C);
    f32 K4 = (A + F - B - E);
    f32 K5 = (A + G - C - E);
    f32 K6 = (B + C + E + H - A - D - F - G);

    // Noise partial derivatives in X, Y, Z directions
    f32 Dx = Du * (K0 + V*K3 + W*K4 + V*W*K6);
    f32 Dy = Dv * (K1 + U*K3 + W*K5 + U*W*K6);
    f32 Dz = Dw * (K2 + U*K4 + V*K5 + U*V*K6);

    return vec3(Dx, Dy, Dz);
}


static inline bool
Perlin3(vec3 P, vec3 Min, vec3 Max)
{
    f32 X = P.X * SCALE_FACTOR;
    f32 Y = P.Y * SCALE_FACTOR;
    f32 Z = P.Z * SCALE_FACTOR;

    // Unit cube Left-Bottom-Back coordinates
    int Xi = (int(X) & 255);
    int Yi = (int(Y) & 255);
    int Zi = (int(Z) & 255);

    // Coords of our point inside its unit cube.
    f32 Xf = X - Fastfloor(X);
    f32 Yf = Y - Fastfloor(Y);
    f32 Zf = Z - Fastfloor(Z);

    // Smooth point coords towards integral values.
    // This produces smoother noise output.
    f32 U = Smoothstep(Xf);
    f32 V = Smoothstep(Yf);
    f32 W = Smoothstep(Zf);

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

    f32 A = GradDotDst(C000, Xf, Yf, Zf);
    f32 B = GradDotDst(C100, Xf - 1.0f, Yf, Zf);
    f32 C = GradDotDst(C010, Xf, Yf - 1.0f, Zf);
    f32 D = GradDotDst(C110, Xf - 1.0f, Yf - 1.0f, Zf);
    f32 E = GradDotDst(C001, Xf, Yf, Zf - 1.0f);
    f32 F = GradDotDst(C101, Xf - 1.0f, Yf, Zf - 1.0f);
    f32 G = GradDotDst(C011, Xf, Yf - 1.0f, Zf - 1.0f);
    f32 H = GradDotDst(C111, Xf - 1.0f, Yf - 1.0f, Zf - 1.0f);

    f32 K0 = (B - A);
    f32 K1 = (C - A);
    f32 K2 = (E - A);
    f32 K3 = D - C - K0;
    f32 K4 = F - E - K0;
    f32 K5 = G - E - K1;
    f32 K6 = (H - G) - (F - E) - K3;//(B + C + E + H - A - D - F - G);

    f32 N = A + U * K0 + V * K1 + W * K2 + U * V * K3 + U * W * K4 + V * W * K5 + U * V * W * K6; 
    N = ((N + 1.0f) / 2.0f);

    vec3 Diff = Abs(Max - Min);
    vec3 MaxDN = vec3(0.009884f)*1.5f*Diff;
    vec3 MaxVal = (vec3(N) + MaxDN);

    return Any(LessThan(vec3(NOISE_THRESHOLD), MaxVal));
}

static inline bool
ShapeSamplerFn(vec3 Min, vec3 Max, const svo* const Tree, const void* const)
{
    vec3 H = (Max - Min) * 0.5f;
    vec3 C = ((Min + H) * f32(1U << Tree->Bias.Scale));
    //return Perlin3(C, Min, Max);

    return Perlin3(C, Min, Max);
}


static inline vec3
DummySampler(vec3 C, const svo* const, const void* const)
{
    return vec3(0.72, 0.25, 0.05);
}


extern svo*
BuildNoiseSvo(u32 MaxDepth, u32 ScaleExp)
{
    shape_sampler S{ nullptr, &ShapeSamplerFn };
    data_sampler D{ nullptr, &DummySampler };

    svo* Svo = CreateScene(10, MaxDepth, &S, &D, &D);


    return Svo;
}
