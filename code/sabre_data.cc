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

#define MAX_STEPS 16
#define SCREEN_DIM 512

#define ASSERT(Expr) if (! (Expr)) { return vec3(1); }

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
    uint Scale;
    uint Node;
    vec3 Centre;
};

struct ray_intersection
{
    float tMin;
    float tMax;
    vec3  tMaxV;
    vec3  tMinV;
};

layout (local_size_x = 8, local_size_y = 8) in;

layout (rgba32f, binding = 0) uniform image2D OutputImgUniform;

uniform uint MaxDepthUniform;
uniform uint BlockCountUniform;
uniform uint ScaleExponentUniform;

uniform vec3 ViewPosUniform;
uniform mat3 ViewMatrixUniform;

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
    uint OccBits = (ParentNode & SVO_NODE_OCCUPIED_MASK) >> 8; 
    uint LeafBits = (ParentNode & SVO_NODE_LEAF_MASK);
    uint OccupiedNonLeafOcts = OccBits & (~LeafBits);
    uint SetBitsBehindOctIdx = (1 << Oct) - 1;

    uint ChildOffset = bitCount(OccupiedNonLeafOcts & SetBitsBehindOctIdx); 

    // TODO(Liam): Broken?
    return SvoInputBuffer.Nodes[ChildPtr + ChildOffset];
}

ray_intersection ComputeRayBoxIntersection(in ray R, in vec3 vMin, in vec3 vMax)
{
    vec3 t0 = (vMin - R.Origin) * R.InvDir;
    vec3 t1 = (vMax - R.Origin) * R.InvDir;

    vec3 tMin = min(t0, t1);
    vec3 tMax = max(t0, t1);

    float ttMin = MaxComponent(tMin);
    float ttMax = MinComponent(tMax);

    ray_intersection Result = { ttMin, ttMax, tMax, tMin };
    return Result;
}


bool IsAdvanceValid(in uint NewOct, in uint OldOct, in vec3 RayDir)
{
    ivec3 Sgn = ivec3(sign(RayDir));
    uvec3 OctBits = uvec3(1, 2, 4);
    
    ivec3 NewOctBits = ivec3(bvec3(uvec3(NewOct) & OctBits));
    ivec3 OldOctBits = ivec3(bvec3(uvec3(OldOct) & OctBits));

    ivec3 OctSgn = NewOctBits - OldOctBits;

    if (Sgn.x < 0 && OctSgn.x > 0) return false;
    if (Sgn.y < 0 && OctSgn.y > 0) return false;
    if (Sgn.z < 0 && OctSgn.z > 0) return false;

    if (Sgn.x > 0 && OctSgn.x < 0) return false;
    if (Sgn.y > 0 && OctSgn.y < 0) return false;
    if (Sgn.z > 0 && OctSgn.z < 0) return false;

    return true;
}


uint GetNextOctant(in float tMax, in vec3 tValues, in uint CurrentOct)
{
    uint NextOct = CurrentOct;

    if (tMax >= tValues.x)
    {
        NextOct ^= 1;//|= CurrentOct ^ 1;
    }

    if (tMax >= tValues.y)
    {
        NextOct ^= 2;//CurrentOct ^ 2;
    }

    if (tMax >= tValues.z)
    {
        NextOct ^= 4;//|= CurrentOct ^ 4;
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

vec3 GetNodeCentreP(in uint Oct, in uint Scale, in vec3 ParentP)
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

    uint Radius = Scale >> 1;

    return ParentP + (vec3(X, Y, Z) * Radius);
}

vec3 Oct2Cr(in uint Oct)
{
    return vec3( bvec3(uvec3(Oct) & uvec3(1, 2, 4))).bgr;
}


uvec3 HDB(uvec3 A, uvec3 B)
{
    uvec3 DB = (A ^ B);

    // Find highest set bits
    uvec3 HighestSetBits = findMSB(DB);

    return HighestSetBits;
}

struct st_frame
{
    uint Node;
    uint Depth;
    int Scale;
    float tMin;
    vec3 ParentCentre;
};

vec3 Raycast(in ray R)
{
    // Extant of the root cube
    int Scale = 1 << (ScaleExponentUniform);
    
    vec3 RootMin = vec3(0);
    vec3 RootMax = vec3(Scale);
    vec3 Sgn = sign(R.Dir);

    // Intersection of the ray with the root cube (i.e. entire tree)
    ray_intersection CurrentIntersection = ComputeRayBoxIntersection(R, RootMin, RootMax);

    int Step;

    // Check if the ray is within the octree at all
    if (CurrentIntersection.tMin <= CurrentIntersection.tMax && CurrentIntersection.tMax > 0)
    {
        // Ray enters octree --- begin processing

        // Initialise parent to root node
        uint ParentNode = SvoInputBuffer.Nodes[0];

        // Current position along the ray
        vec3 RayP = R.Origin + CurrentIntersection.tMin * R.Dir;

        vec3 ParentCentre = vec3(Scale >> 1);

        // Current octant the ray is in (confirmed good)
        uint CurrentOct = GetOctant(RayP, ParentCentre);
        
        // Initialise depth to 1
        uint CurrentDepth = 1;

        // Stack of previous voxels
        st_frame Stack[MAX_STEPS + 1];
        Scale >>= 1;
        Stack[CurrentDepth] = st_frame(ParentNode, CurrentDepth, Scale, CurrentIntersection.tMin, ParentCentre);

        // Begin stepping along the ray
        for (Step = 0; Step < MAX_STEPS; ++Step)
        {
            // Go down 1 level
            vec3 NodeCentre = GetNodeCentreP(CurrentOct, Scale, ParentCentre);
            vec3 NodeMin = NodeCentre - vec3(Scale >> 1);
            vec3 NodeMax = NodeCentre + vec3(Scale >> 1);

            CurrentIntersection = ComputeRayBoxIntersection(R, NodeMin, NodeMax);

            if (CurrentIntersection.tMin <= CurrentIntersection.tMax && CurrentIntersection.tMax > 0)
            {
                // Ray hit this voxel
                
                // Check if voxel occupied
                if (IsOctantOccupied(ParentNode, CurrentOct))
                {
                    // Octant is occupied, check if leaf
                    if (IsOctantLeaf(ParentNode, CurrentOct))
                    {
                        // Done - return leaf colour
                        float O = float(Step) / MAX_STEPS;
                        return O * vec3(0.4, 0, 0.3);
                    }
                    else
                    {
                        // Voxel has children --- execute push
                        ParentNode = GetNodeChild(ParentNode, CurrentOct);
                        CurrentOct = GetOctant(RayP, NodeCentre);
                        ParentCentre = NodeCentre;
                        Scale >>= 1;
                        ++CurrentDepth;

                        Stack[CurrentDepth] = st_frame(ParentNode, CurrentDepth, Scale, CurrentIntersection.tMin, ParentCentre);

                        continue;
                    }
                }

                // Octant not occupied, need to handle advance/pop
                uint NextOct = GetNextOctant(CurrentIntersection.tMax, CurrentIntersection.tMaxV, CurrentOct);

                RayP = R.Origin + (CurrentIntersection.tMax + 1) * R.Dir;

                if (IsAdvanceValid(NextOct, CurrentOct, R.Dir))
                {
                    CurrentOct = NextOct;
                }
                else
                {
                    // Determined that NodeCentre is never < 0
                    uvec3 NodeCentreBits = uvec3(NodeCentre);
                    uvec3 RayPBits = uvec3(RayP);

                    // NOTE(Liam): It is **okay** to have negative values here
                    // because the HDB will end up being equal to ScaleExponentUniform.
                    uvec3 HighestDiffBits = HDB(NodeCentreBits, RayPBits);

                    uint M = uint(MaxComponent(HighestDiffBits));

                    M = 0;
                    if (HighestDiffBits.x > M && HighestDiffBits.x < ScaleExponentUniform) M = HighestDiffBits.x;
                    if (HighestDiffBits.y > M && HighestDiffBits.y < ScaleExponentUniform) M = HighestDiffBits.y;
                    if (HighestDiffBits.z > M && HighestDiffBits.z < ScaleExponentUniform) M = HighestDiffBits.z;

                    uint NextDepth = (ScaleExponentUniform - M);

                    if (NextDepth >= 0 && NextDepth <= MAX_STEPS)
                    {
                        CurrentDepth = NextDepth;
                        Scale = Stack[CurrentDepth].Scale;
                        ParentCentre = Stack[CurrentDepth].ParentCentre;
                        ParentNode = Stack[CurrentDepth].Node;

                        CurrentOct = GetOctant(RayP, ParentCentre);
                    }
                    else
                    {
                        return vec3(1, 1, 0);
                    }
                }
            }
            else
            {
                return vec3(0.5, 0.2, 0.6);
            }
        }
    }
    else
    {
        // Ray doesn't hit octree --- output background colour
        return vec3(0.12);
    }

    return vec3(0, 0, 1);
}

bool Trace2(in ray R)
{
    vec3 Min = vec3(-4);
    vec3 Max = vec3(4);

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

    RayD = RayD * ViewMatrixUniform;

    ray R = { RayP, RayD, 1.0 / RayD };

    vec3 OutCr = Raycast(R);

    imageStore(OutputImgUniform, ivec2(PixelCoords), vec4(OutCr, 1.0));
}
)GLSL";

