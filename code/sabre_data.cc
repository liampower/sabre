#include "sabre_data.h"

extern const char* const MainVertexCode = R"GLSL(
#version 450 core

layout (location = 0) in vec2 Position;

out vec2 UV;

void main()
{
    gl_Position = vec4(Position, 0.0, 1.0);
    UV = Position*0.5 + 0.5;
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

#define SVO_NODE_OCCUPIED_MASK  0x0000FF00U
#define SVO_NODE_LEAF_MASK      0x000000FFU
// TODO(Liam): Need to fix this up to support
// far pointers!
#define SVO_NODE_CHILD_PTR_MASK 0xFFFF0000U

#define MAX_STEPS 64
#define SCREEN_DIM 512

struct ray
{
    vec3 Origin;
    vec3 Dir;
    vec3 InvDir;
};

struct stack_frame
{
    uint Oct;
    uint Depth;
};

struct ray_intersection
{
    float tMin;
    float tMax;
    vec3  tValues;
};

layout (local_size_x = 1, local_size_y = 1) in;

layout (rgba32f, binding = 0) uniform image2D OutputImgUniform;

uniform uint MaxDepthUniform;
uniform uint BlockCountUniform;

uniform mat4x4 ViewMatrixUniform;
uniform vec3 ViewPosUniform;

layout (std430, binding = 3) readonly buffer svo_input
{
    uint Nodes[];
} SvoInputBuffer;


float MaxComponent(vec3 V)
{
    return max(max(V.x, V.y), V.z);
}

float MinComponent(vec3 V)
{
    return min(min(V.x, V.y), V.z);
}

uint GetNodeChild(in uint ParentNode, in uint Oct)
{
    uint ChildPtr = (ParentNode & SVO_NODE_CHILD_PTR_MASK) >> 16;
    
    return SvoInputBuffer.Nodes[ChildPtr + Oct];
}

ray_intersection ComputeRayBoxIntersection(in ray R, in vec3 Min, in vec3 Max)
{
    vec3 t0 = (Min - R.Origin) * R.InvDir;
    vec3 t1 = (Max - R.Origin) * R.InvDir;

    vec3 tMin = min(t0, t1);
    vec3 tMax = max(t0, t1);

    float ttMin = MaxComponent(tMin);
    float ttMax = MinComponent(tMax);


    ray_intersection Result = { ttMin, ttMax, tMax };
    return Result;
}


uint GetNextOctant(in float tMax, in vec3 tValues, in uint CurrentOct)
{
    uint NextOct = 0x00;

    if (tMax == tValues.x)
    {
        NextOct = CurrentOct ^ 1;
    }
    else if (tMax == tValues.y)
    {
        NextOct = CurrentOct ^ 2;
    }
    else
    {
        NextOct = CurrentOct ^ 4;
    }

    return NextOct;
}

uint GetOctant(in vec3 P, in vec3 ParentCentreP)
{
    uvec3 G = uvec3(greaterThan(P, ParentCentreP));

    // TODO(Liam): Can use a DP here 
    return G.x + G.y*2 + G.z*4;
}


bool IsOctantOccupied(in uint Node, in uint Oct)
{
    return (Node & (1 << (8 + Oct))) != 0;
}

bool IsOctantLeaf(in uint Node, in uint Oct)
{
    return (Node & (1 << Oct)) != 0;
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


// From 0 to Max
vec3 CrScale(in uint V, in uint Max)
{
    float S = 0.1 + (float(V) / float(Max));

    return S * vec3(0, 1, 0);
}

bool IsAdvanceValid(in uint NewOct, in uint OldOct, in vec3 RayDir)
{
    vec3 Sgn = sign(RayDir);
    uint Diff = NewOct - OldOct;
    
    // TODO(Liam): Step?
    //if (Sgn.x < 0) Sgn.x = 0;
    //if (Sgn.y < 0) Sgn.y = 0;
    //if (Sgn.z < 0) Sgn.z = 0;


    bool Increased = (NewOct > OldOct);

    if (Sgn.x < 0 && Increased) return false;
    if (Sgn.y < 0 && Increased) return false;
    if (Sgn.z < 0 && Increased) return false;

    return true;
}

vec3 Raycast(in ray R)
{
    vec3 ParentCentre = vec3(0, 0, 0);
    uint CurrentDepth = 0;
    uint CurrentNode = SvoInputBuffer.Nodes[0]; // Initialised to root

    stack_frame Stack[MAX_STEPS + 1];
    uint StackHead = 0;

    for (int Step = 0; Step < MAX_STEPS; ++Step)
    {
        // TODO(Liam): Can just shift each iteration instead of recalculating
        uint Radius = 1 << (MaxDepthUniform - CurrentDepth);
        uint CurrentOctant = GetOctant(R.Origin, ParentCentre);

        vec3 NodeCentre = GetNodeCentreP(CurrentOctant, Radius, ParentCentre);
        vec3 NodeMin = NodeCentre - vec3(Radius);
        vec3 NodeMax = NodeCentre + vec3(Radius);


        ray_intersection Intersection = ComputeRayBoxIntersection(R, NodeMin, NodeMax);

        // Ray intersects this node
        if (Intersection.tMin < Intersection.tMax)
        {
            // Check if there is geometry inside this node
            if (IsOctantOccupied(CurrentNode, CurrentOctant))
            {
                if (IsOctantLeaf(CurrentNode, CurrentOctant))
                {
                    //return vec3(1, 0, 0); // Leaf: return this octant
                    break;
                }
                else // Go deeper
                {
                    CurrentNode = GetNodeChild(CurrentNode, CurrentOctant);
                    ParentCentre = NodeCentre;

                    CurrentOctant = GetOctant(R.Origin, ParentCentre);
                    ++CurrentDepth;

                    /*stack_frame Frame = { CurrentOctant, CurrentDepth };
                    Stack[StackHead] = Frame;
                    ++StackHead;*/
                }
            }
            else // Nothing here... move on to sibling
            {
                return vec3(1, 0, 0);
                // TODO(Liam): Handle case where we need to push instead.
                uint NextOctant = GetNextOctant(Intersection.tMax, Intersection.tValues, CurrentOctant);
                if (IsAdvanceValid(NextOctant, CurrentOctant, R.Dir))
                {
                    CurrentOctant = NextOctant;
                }
                else // Pop
                {
                    CurrentOctant = Stack[StackHead].Oct;
                    CurrentDepth = Stack[StackHead].Depth;
                    --StackHead;
                }
            }
        }
        else
        {
            return vec3(0.16);
            break;
        }

    }

    return CrScale(CurrentDepth, MaxDepthUniform);
}


bool Trace2(in ray R)
{
    vec3 P = R.Origin + R.Dir;
    float D = 0.0;

    for (int i = 0; i < 16; ++i)
    {
        D = length(P) - 1;

        if (abs(D) < 0.01) return true;

        P += D*R.Dir;
    }

    return false;
}

void main()
{
    // Ray XY coordinates of the screen pixels; goes from 0-512
    // in each dimension.
    vec2 PixelCoords = ivec2(gl_GlobalInvocationID.xy);

    // Origin of the screen plane in world coords; the camera pos
    // is at the centre of the screen
    vec2 ScreenOrigin = vec2(-SCREEN_DIM/2); 

    // World coordinates of the current pixel on the camera view
    // plane
    /*vec3 ScreenCoords = vec3(ScreenOrigin + PixelCoords.xy, 0);
    
    vec3 O = vec3(0, 0, -512);
    vec3 RayP = ViewPosUniform;//vec3(0, 0, -512);
    vec3 RayD = normalize(ScreenCoords - O);*/
    //vec2 UV = (PixelCoords * vec2(2/512)) - vec2(1);

    // Maps into [-1, 1] range
    vec2 UV = (PixelCoords * (2.0 / 512.0)) - 1.0;

    vec3 RayP = ViewPosUniform;//vec3(0, 0, -4);//ViewPosUniform;

    vec3 RayD = normalize(vec3(UV, 1.0));

    ray R = { RayP, RayD, 1.0 / RayD };

    //vec4 OutCr = vec4(vec3(Trace2(R)), 1.0);//vec4(Raycast(R), 1.0);//mix(vec4(abs(R.Dir), 1.0), vec4(Raycast(R), 1.0), 0.5);
    vec4 OutCr = vec4(Raycast(R), 1.0);//Trace2(R) ? vec4(1.0, 0, 0, 1.0) : vec4(0);

    imageStore(OutputImgUniform, ivec2(PixelCoords), OutCr);
}
)GLSL";

