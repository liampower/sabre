#include "sabre_data.h"



extern const char* const MainVertexCode = R"GLSL(
#version 450 core

layout (location = 0) in vec2 Position;

out vec2 UV;

void main()
{
    gl_Position = vec4(Position, 0.0, 1.0);
    UV = Position.xy;
}

)GLSL";

extern const char* const MainFragmentCode = R"GLSL(
#version 450 core

out vec4 FragCr;

uniform sampler2D OutputTextureUniform;

in vec2 UV;

void main()
{
    FragCr = texture(OutputTextureUniform, UV);
}

)GLSL";


extern const char* const RaycasterComputeKernel = R"GLSL(
#version 450 core

struct ray
{
    vec3 Origin;
    vec3 Dir;
    vec3 InvDir;
};

layout (local_size_x = 2, local_size_y = 2) in;

layout (rgba32f, binding = 0) uniform image2D OutputImgUniform;

uniform uint MaxDepthUniform;

float MaxComponent(vec3 V)
{
    return max(max(V.x, V.y), V.z);
}

float MinComponent(vec3 V)
{
    return min(min(V.x, V.y), V.z);
}

vec2 ComputeRayBoxIntersection(in ray R, in vec3 Min, in vec3 Max)
{
    vec3 t0 = (Min - R.Origin) * R.InvDir;
    vec3 t1 = (Max - R.Origin) * R.InvDir;

    vec3 tMin = min(t0, t1);
    vec3 tMax = max(t0, t1);

    float ttMin = MaxComponent(tMin);
    float ttMax = MinComponent(tMax);

    return vec2(ttMin, ttMax);
}

uint GetOctant(in vec3 P, in vec3 ParentCentreP)
{
    uvec3 G = uvec3(greaterThan(P, ParentCentreP));

    // TODO(Liam): Can use a DP here 
    return G.x + G.y*2 + G.z*4;
}

vec3 GetNodeCentreP(in uint Oct, in uint Radius, in vec3 ParentP)
{
    // TODO(Liam): We can actually use a bitwise AND op on uvec3s
    // so we don't have to work on scalars.

    float X = 1.0;
    float Y = 1.0;
    float Z = 1.0;

    // TODO(Liam): More speed here with step intrinsic?
    
    //  Use the bits of the input octant to compute the
    //  X, Y and Z vectors. A zero-bit corresponds to -1
    //  in some D direction, and a 1-bit corresponds to
    //  1.0 in D.
    if (0 == (Oct & 1)) X = -1.0;
    if (0 == (Oct & 2)) Y = -1.0;
    if (0 == (Oct & 4)) Z = -1.0;

    return ParentP + (vec3(X, Y, Z) * Radius);
}


vec4 Raycast(in ray R)
{
    uint Depth = 1;
    uint Radius = 1 << (MaxDepthUniform - Depth);

    // Initialise current octant to child of root
    uint CurrentOct = GetOctant(R.Origin, vec3(0, 0, 0));
    vec3 NodeCentreP = GetNodeCentreP(CurrentOct, Radius, vec3(0, 0, 0));
    vec3 NodeMin = NodeCentreP - vec3(Radius);
    vec3 NodeMax = NodeCentreP + vec3(Radius);

    vec2 Intersection = ComputeRayBoxIntersection(R, NodeMin, NodeMax);

    return vec4(Intersection.xy, Intersection.yx);
}

void main()
{
    ivec2 PixelCoords = ivec2(gl_GlobalInvocationID.xy);

    vec3 RayP = vec3(PixelCoords.xy, -1.0);
    vec3 RayD = normalize(vec3(16, 16, 16));

    ray R = { RayP, RayD, 1.0 / RayD };

    vec4 OutCr = Raycast(R);

    imageStore(OutputImgUniform, PixelCoords, OutCr);
}
)GLSL";

