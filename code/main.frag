#version 450 core

out vec4 FragCr;

uniform sampler2D OutputTextureUniform;

struct luma
{
    float N, S, E, W, M;
    float Min, Max, Contrast;

    float NE, NW, SE, SW;
};

struct edge
{
    bool Horz;
    float StepSize;
};


in vec2 UV;


float ComputeBlendFactor(luma L)
{
    float F = 2.0 * (L.N + L.E + L.S + L.W);
    F += L.NE + L.NW + L.SE + L.SW;
    F *= 1/12.0;
    F = abs(F - L.M);
    F = clamp(F / L.Contrast, 0, 1);
    float B = smoothstep(0, 1, F);
    
    return B*B*0.75;
}

edge ComputeEdge(in luma L)
{
    edge E;
    float HorzContrast =
        abs(L.N + L.S - 2 * L.M) * 2 +
        abs(L.NE + L.SE - 2 * L.E) +
        abs(L.NW + L.SW - 2 * L.W);
    float VertContrast =
        abs(L.E + L.W - 2 * L.M) * 2 +
        abs(L.NE + L.NW - 2 * L.N) +
        abs(L.SE + L.SW - 2 * L.S);

    E.Horz = HorzContrast >= VertContrast;
    E.StepSize = E.Horz ? 1/1024.0 : 1/1024.0;
    float pLuminance = E.Horz ? L.N : L.E;
    float nLuminance = E.Horz ? L.S : L.W;
    float pGradient = abs(pLuminance - L.M);
    float nGradient = abs(nLuminance - L.M);

    if (pGradient < nGradient) {
        E.StepSize = -E.StepSize;
    }

    return E;
}

luma SampleLuma(vec2 UV)
{
    const vec2 Offsets[8] = {
        vec2(0, 1),
        vec2(1, 0),
        vec2(0, -1),
        vec2(-1, 0),

        vec2(1, 1),
        vec2(-1, 1),
        vec2(1, -1),
        vec2(-1, -1),
    };
    const float Ts = 1/1024.0;

    float M = texture(OutputTextureUniform, UV).g;
    float N = texture(OutputTextureUniform, UV + Offsets[0]*Ts).g;
    float E = texture(OutputTextureUniform, UV + Offsets[1]*Ts).g;
    float S = texture(OutputTextureUniform, UV + Offsets[2]*Ts).g;
    float W = texture(OutputTextureUniform, UV + Offsets[3]*Ts).g;
    float NE = texture(OutputTextureUniform, UV + Offsets[4]*Ts).g;
    float NW = texture(OutputTextureUniform, UV + Offsets[5]*Ts).g;
    float SE = texture(OutputTextureUniform, UV + Offsets[6]*Ts).g;
    float SW = texture(OutputTextureUniform, UV + Offsets[7]*Ts).g;

    float Min = min(min(min(min(M, N), E), S), W);
    float Max = max(max(max(max(M, N), E), S), W);

    float Contrast = Max - Min;

    return luma(N, S, E, W, M, Min, Max, Contrast, NE, NW, SE, SW);
}

void main()
{
    FragCr = texture(OutputTextureUniform, UV);
    /*luma L = SampleLuma(UV);
    float Blend = ComputeBlendFactor(L);
    edge E = ComputeEdge(L);

    vec2 UV2 = UV;
    if (E.Horz) {
        UV2.y += E.StepSize * Blend;
    }
    else {
        UV2.x += E.StepSize * Blend;
    }
    

    FragCr = vec4(textureLod(OutputTextureUniform, UV2, 0).rgb, L.M);*/
    //FragCr = texture(OutputTextureUniform, UV);*/
}

