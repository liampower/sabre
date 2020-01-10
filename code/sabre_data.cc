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

// TODO(Liam): Theoretical f32 ops we can do in 16ms on a 620M: 3840000000 
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
    uint Node;
    vec3 Centre;
};

struct ray_intersection
{
    float tMin;
    float tMax;
    vec3  tValues;
    vec3  tMinValues;
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
    
    // TODO(Liam): Broken?
    return SvoInputBuffer.Nodes[ChildPtr];
}

ray_intersection ComputeRayBoxIntersection(in ray R, in vec3 Min, in vec3 Max)
{
    vec3 t0 = (Min - R.Origin) * R.InvDir;
    vec3 t1 = (Max - R.Origin) * R.InvDir;

    vec3 tMin = min(t0, t1);
    vec3 tMax = max(t0, t1);

    float ttMin = MaxComponent(tMin);
    float ttMax = MinComponent(tMax);


    ray_intersection Result = { ttMin, ttMax, tMax, tMin };
    return Result;
}


uint GetOct(in float tMin, in vec3 tValues)
{
    const uvec3 OctBits = uvec3(1, 2, 4);
    bvec3 E = equal(vec3(tMin), tValues);

    return uint(dot(uvec3(E), OctBits));
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

vec3 GetNodeCentreP(in uint Oct, in int Radius, in vec3 ParentP)
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

const vec3 DEBUGOctColours[8] = {
    vec3(0, 0, 0),
    vec3(0, 0, 1),
    vec3(0, 1, 0),
    vec3(0, 1, 1),
    vec3(1, 0, 0),
    vec3(1, 0, 1),
    vec3(1, 1, 0),
    vec3(1, 1, 1),
};

vec3 Raycast(in ray R)
{
    int DEBUGrad = 1 << MaxDepthUniform;
    ray_intersection DEBUGr = ComputeRayBoxIntersection(R, vec3(-DEBUGrad), vec3(DEBUGrad));
    if (DEBUGr.tMin > DEBUGr.tMax || DEBUGr.tMax <= 0) return vec3(0.992, 0.961, 0.902);

    vec3 RootCentre = vec3(0, 0, 0);
    uint RootDepth = 0;
    uint RootNode = SvoInputBuffer.Nodes[0]; // Initialised to root

    //uint RootOctant = GetOct(DEBUGr.tMin, DEBUGr.tMinValues);//GetOctant(R.Origin, RootCentre);
    uint RootOctant = GetOctant(R.Origin + DEBUGr.tMin*R.Dir, RootCentre);
    //return vec3(float(RootOctant) / 7.0);

    // Initialise stack top with root node.
    stack_frame Stack[MAX_STEPS + 1];
    uint StackTop = 0;
    stack_frame RootFrame = { RootOctant, RootDepth, RootNode, RootCentre };
    Stack[StackTop] = RootFrame;
    float Sh = float(RootOctant) / 7.0;

    for (int Step = 0; Step < MAX_STEPS; ++Step)
    {
        stack_frame CurrentContext = Stack[StackTop];

        if (CurrentContext.Depth > MaxDepthUniform) return vec3(1, 0, 0);

        // TODO(Liam): Can just shift each iteration instead of recalculating
        int Radius = 1 << (MaxDepthUniform - CurrentContext.Depth);

       // vec3 NodeCentre = GetNodeCentreP(CurrentContext.Oct, Radius, CurrentContext.Centre);
        vec3 NodeCentre = CurrentContext.Centre;
        vec3 NodeMin = NodeCentre - vec3(Radius);
        vec3 NodeMax = NodeCentre + vec3(Radius);
        uint CurrentNode = CurrentContext.Node;

        ray_intersection Intersection = ComputeRayBoxIntersection(R, NodeMin, NodeMax);

        // Ray intersects this node
        if (Intersection.tMin < Intersection.tMax && Intersection.tMax > 0)
        {
            Sh *= float(CurrentContext.Oct) / 7.0;
            // Check if there is geometry inside this node
            if (IsOctantOccupied(CurrentContext.Node, CurrentContext.Oct))
            {
                //return vec3(float(CurrentContext.Oct) / 7.0);
                if (IsOctantLeaf(CurrentContext.Node, CurrentContext.Oct))
                {
                    return vec3(0, 0, 1);
                }
                else // Go deeper (push)
                {
                    vec3 P = R.Origin + Intersection.tMin*R.Dir;
                    //R.Origin += Intersection.tMin;
                    //R.Origin = P;
                    //Out *= (P / vec3(1 << MaxDepthUniform));

                    uint ChildNode = GetNodeChild(CurrentContext.Node, CurrentContext.Oct);
                    uint ChildOct = GetOctant(P, NodeCentre);//GetOct(Intersection.tMin, Intersection.tMinValues);//GetOctant(P, CurrentContext.Centre);
                    int Rad = Radius << 1;

                    vec3 ChildCentre = GetNodeCentreP(ChildOct, Rad, CurrentContext.Centre);

                    stack_frame Frame = { ChildOct, CurrentContext.Depth + 1, ChildNode, ChildCentre };
                    ++StackTop;
                    Stack[StackTop] = Frame;
                }
            }
            else // Nothing here... move on to sibling
            {
                return vec3(1, 0, 0);
                // TODO(Liam): Handle case where we need to push instead.
                /*uint NextOctant = GetNextOctant(Intersection.tMax, Intersection.tValues, CurrentOctant);
                if (IsAdvanceValid(NextOctant, CurrentOctant, R.Dir))
                {
                    CurrentOctant = NextOctant;
                }
                else // Pop
                {
                }*/
            }
        }
        else
        {
            return vec3(Sh);//Out;//return vec3(0.12);
        }

    }

    return vec3(1);
    //return CrScale(CurrentDepth, MaxDepthUniform);
}


bool Trace2(in ray R)
{
    vec3 P = R.Origin + R.Dir;
    float D = 0.0;

    vec3 Min = vec3(-256);
    vec3 Max = vec3(256);

    ray_intersection I = ComputeRayBoxIntersection(R, Min, Max);


    return I.tMin <= I.tMax;
}

void main()
{
    // Ray XY coordinates of the screen pixels; goes from 0-512
    // in each dimension.
    vec2 PixelCoords = ivec2(gl_GlobalInvocationID.xy);
    vec3 ScreenOrigin = vec3(ViewPosUniform.x - 256, ViewPosUniform.y - 256, ViewPosUniform.z - 512);

	vec3 ScreenCoord = ScreenOrigin + vec3(PixelCoords, 0);

    vec3 RayP = ViewPosUniform;
    vec3 RayD = normalize(ScreenCoord - ViewPosUniform);

    ray R = { RayP, RayD, 1.0 / RayD };

    vec4 OutCr = vec4(Raycast(R), 1.0);//vec4(0, 1, 0, 1) * float(Trace2(R));

    imageStore(OutputImgUniform, ivec2(PixelCoords), OutCr);
}
)GLSL";

