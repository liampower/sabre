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

#define SVO_NODE_OCCUPIED_MASK 0x0000FF00U
#define SVO_NODE_LEAF_MAKS     0x000000FFU

struct ray
{
    vec3 Origin;
    vec3 Dir;
    vec3 InvDir;
};

struct svo_block
{
    uint Nodes[4096];
};

layout (local_size_x = 2, local_size_y = 2) in;

layout (rgba32f, binding = 0) uniform image2D OutputImgUniform;

uniform uint MaxDepthUniform;
uniform uint BlockCountUniform;

layout (std430) buffer svo_input
{
    svo_block Blocks[];
} SvoInputBuffer;

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


bool IsNodeOccupied(in uint Node)
{
    return bool((Node & SVO_NODE_OCCUPIED_MASK) != 0);
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

uint GetSvoNode(in uint Parent, in uint Oct)
{
    return 0;
}

vec4 Raycast(in ray R)
{
    uint CurrentDepth = 1;
    uint Radius = 1 << (MaxDepthUniform - CurrentDepth);

    // Initialise current octant to child of root
    uint CurrentOctant = GetOctant(R.Origin, vec3(0, 0, 0));

    // Compute the node centre and bounds
    vec3 NodeCentreP = GetNodeCentreP(CurrentOctant, Radius, vec3(0, 0, 0));
    vec3 NodeMin = NodeCentreP - vec3(Radius);
    vec3 NodeMax = NodeCentreP + vec3(Radius);

    // Determine the intersection point of the ray and this node box
    vec2 Intersection = ComputeRayBoxIntersection(R, NodeMin, NodeMax);

    uint CurrentNode = GetSvoNode(CurrentOctant, CurrentDepth);

    return vec4(Intersection.xy, Intersection.yx);
}


void main()
{
    ivec2 PixelCoords = ivec2(gl_GlobalInvocationID.xy);

    vec3 RayP = vec3(PixelCoords.xy, -0.5);
    vec3 RayD = normalize(vec3(16, 16, 16));

    ray R = { RayP, RayD, 1.0 / RayD };

    vec4 OutCr = Raycast(R);

    imageStore(OutputImgUniform, PixelCoords, OutCr);
}
)GLSL";

