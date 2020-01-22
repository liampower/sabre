#include <intrin.h>
#include <stdio.h>
#include <math.h>
#include "sabre.h"
#include "sabre_math.h"

#define SVO_NODE_OCCUPIED_MASK  0x0000FF00U
#define SVO_NODE_LEAF_MASK      0x000000FFU
// TODO(Liam): Need to fix this up to support
// far pointers!
#define SVO_NODE_CHILD_PTR_MASK 0xFFFF0000U

#define MAX_STEPS 64
#define SCREEN_DIM 512

#define ASSERT(Expr) if (! (Expr)) { return vec3(1); }

#define in
#define out
#define inout

typedef unsigned int uint;

static vec3 OutBuffer[512][512];

struct vec2
{
    float X;
    float Y;
};

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

//layout (local_size_x = 4, local_size_y = 4) in;

//layout (rgba32f, binding = 0) uniform image2D OutputImgUniform;

static uint MaxDepthUniform = 4;
static uint BlockCountUniform = 1;
static uint ScaleExponentUniform = 5;

static vec3 ViewPosUniform = vec3(-5, -5, -512);
static mat3 ViewMatrixUniform;


struct svo_input
{
    uint Nodes[];
};

#if 0
static svo_input SvoInputBuffer = {
    {
        130816,
        32896,
        16448,
        8224,
        4112,
        2056,
        1028,
        514,
        257,
    }
};
#endif

static svo_input SvoInputBuffer = {
    {
130816,
655104,
1179392,
1703680,
2227968,
2752256,
3276544,
3800832,
4325120,
32896,
64764,
64250,
65535,
61166,
65535,
65535,
65535,
64764,
16448,
65535,
62965,
65535,
56797,
65535,
65535,
64250,
65535,
8224,
62451,
65535,
65535,
48059,
65535,
65535,
62965,
62451,
4112,
65535,
65535,
65535,
30583,
61166,
65535,
65535,
65535,
2056,
53199,
44975,
65535,
65535,
56797,
65535,
65535,
53199,
1028,
65535,
24415,
65535,
65535,
48059,
65535,
44975,
65535,
514,
16191,
65535,
65535,
65535,
30583,
65535,
24415,
16191,
257,
    }
};

static uint PrevOct;

uvec3 Abs(vec3 V)
{
    uvec3 Out;

    Out.X = fabs(V.X);
    Out.Y = fabs(V.Y);
    Out.Z = fabs(V.Z);

    return Out;
}

uint BitCount(uint X)
{
    return __popcnt(X);
}


uvec3 FindMSB(uvec3 A)
{
    uvec3 Result;

    unsigned long MSB;
    _BitScanReverse(&MSB, A.X);
    Result.X = (u32)MSB;

     _BitScanReverse(&MSB, A.Y);
    Result.Y = (u32)MSB;

     _BitScanReverse(&MSB, A.Z);
    Result.Z = (u32)MSB;

    return Result;
}

uint GetNodeChild(in uint ParentNode, in uint Oct)
{
    uint ChildPtr = (ParentNode & SVO_NODE_CHILD_PTR_MASK) >> 16;
    uint OccBits = (ParentNode & SVO_NODE_OCCUPIED_MASK) >> 8; 
    uint LeafBits = (ParentNode & SVO_NODE_LEAF_MASK);
    uint Cmsk = OccBits & (~LeafBits);
    uint Smsk = (1 << Oct) - 1;

    uint ChildOffset = BitCount(Cmsk & Smsk); 


    // TODO(Liam): Broken?
    return SvoInputBuffer.Nodes[ChildPtr + ChildOffset];
}

ray_intersection ComputeRayBoxIntersection(in ray R, in vec3 vMin, in vec3 vMax)
{
    vec3 t0 = (vMin - R.Origin) * R.InvDir;
    vec3 t1 = (vMax - R.Origin) * R.InvDir;

    vec3 tMin = Min(t0, t1);
    vec3 tMax = Max(t0, t1);

    float ttMin = MaxComponent(tMin);
    float ttMax = MinComponent(tMax);

    ray_intersection Result = { ttMin, ttMax, tMax, tMin };
    return Result;
}


bool IsAdvanceValid(in uint NewOct, in uint OldOct, in vec3 RayDir)
{
    ivec3 Sgn = ivec3(Sign(RayDir));
    uvec3 OctBits = uvec3(1, 2, 4);
    
    ivec3 NewOctBits = ivec3(bvec3(uvec3(NewOct) & OctBits));
    ivec3 OldOctBits = ivec3(bvec3(uvec3(OldOct) & OctBits));

    ivec3 OctSgn = NewOctBits - OldOctBits;

    if (Sgn.X < 0 && OctSgn.X > 0) return false;
    if (Sgn.Y < 0 && OctSgn.Y > 0) return false;
    if (Sgn.Z < 0 && OctSgn.Z > 0) return false;

    if (Sgn.X > 0 && OctSgn.X < 0) return false;
    if (Sgn.Y > 0 && OctSgn.Y < 0) return false;
    if (Sgn.Z > 0 && OctSgn.Z < 0) return false;

    return true;
}


uint GetNextOctant(in float tMax, in vec3 tValues, in uint CurrentOct)
{
    uint NextOct = CurrentOct;

    if (tMax >= tValues.X)
    {
        NextOct ^= 1;//|= CurrentOct ^ 1;
    }

    if (tMax >= tValues.Y)
    {
        NextOct ^= 2;//CurrentOct ^ 2;
    }

    if (tMax >= tValues.Z)
    {
        NextOct ^= 4;//|= CurrentOct ^ 4;
    }

    return NextOct;
}

uint GetOctant(in vec3 P, in vec3 ParentCentreP)
{
    uvec3 G = uvec3(GreaterThan(P, ParentCentreP));

    // TODO(Liam): Can use a DP here 
    return G.X + G.Y*2 + G.Z*4;
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


// From 0 to Max
vec3 CrScale(in uint V, in uint Max)
{
    float S = 0.1 + (float(V) / float(Max));

    return S * vec3(0, 1, 0);
}


uvec3 HDB(uvec3 A, uvec3 B)
{
    uvec3 DB = (A ^ B);

    // Find highest set bits
    uvec3 HighestSetBits = FindMSB(DB);

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
    vec3 Sgn = Sign(R.Dir);

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
        st_frame Stack[MAX_STEPS + 1] = { 0 };
        Scale >>= 1;
        Stack[CurrentDepth] = { ParentNode, CurrentDepth, Scale, CurrentIntersection.tMin, ParentCentre };

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
                        return vec3(0.4, 0, 0.3);
                    }
                    else
                    {
                        // Voxel has children --- execute push
                        ParentNode = GetNodeChild(ParentNode, CurrentOct);
                        CurrentOct = GetOctant(RayP, NodeCentre);
                        ParentCentre = NodeCentre;
                        Scale >>= 1;
                        ++CurrentDepth;

                        Stack[CurrentDepth] = { ParentNode, CurrentDepth, Scale, CurrentIntersection.tMin, ParentCentre };

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
                    uvec3 NodeCentreBits = uvec3(NodeCentre);
                    uvec3 RayPBits = uvec3(RayP);

                    uvec3 HighestDiffBits = Clamp(HDB(NodeCentreBits, RayPBits), 0, ScaleExponentUniform);
                    uint M = uint(MaxComponent(HighestDiffBits));

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
                return vec3(1);
            }
        }
    }
    else
    {
        // Ray doesn't hit octree --- output background colour
        return vec3(0.12f);
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

int main()
{
    vec3 Right =  vec3(0.821647, -0.000000, 0.569997);
    vec3 Up = vec3(-0.095058, 0.985996, 0.137025);
    vec3 Forward = vec3(0.562015, 0.166769, -0.810140);
    vec3 Position = vec3(4.459143, 10.918401, 28.265924);

    mat3 View = {{
        { Right.X, Right.Y, Right.Z },
        { Up.X, Up.Y, Up.Z },
        { -Forward.X, -Forward.Y, -Forward.Z },
    }};

    // Ray XY coordinates of the screen pixels; goes from 0-512
    // in each dimension.
    for (int X = 0; X < 512; ++X)
    {
        for (int Y = 0; Y < 512; ++Y)
        {
            vec2 PixelCoords = { (float)Y, (float)X };
            vec3 ScreenOrigin = vec3(ViewPosUniform.X - 256, ViewPosUniform.Y - 256, ViewPosUniform.Z - 512);

            vec3 ScreenCoord = ScreenOrigin + vec3(PixelCoords.X, PixelCoords.Y, 0);

            vec3 RayP = Position;
            vec3 RayD = Normalize(ScreenCoord - Position);

            RayD = RayD * View;

            ray R = { RayP, RayD, Invert(RayD) };

            vec3 OutCr = Raycast(R);

            OutBuffer[X][Y] = OutCr;
        }
    }

    return 0;
}
