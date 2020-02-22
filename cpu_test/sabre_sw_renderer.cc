#include <intrin.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "sabre.h"
#include "sabre_math.h"
#include "sabre_svo.h"

#define SVO_NODE_OCCUPIED_MASK  0x0000FF00U
#define SVO_NODE_LEAF_MASK      0x000000FFU
#define SVO_NODE_CHILD_PTR_MASK 0x7FFF0000U
#define SVO_FAR_PTR_BIT_MASK    0x80000000U

#define SVO_FAR_PTRS_PER_BLOCK  2
//#define SVO_ENTRIES_PER_BLOCK   1

#define MAX_STEPS 16
#define SCREEN_DIM 512

#define SABRE_MAX_TREE_DEPTH 4
#define SABRE_SCALE_EXPONENT 5

#define ASSERT(Expr) if (! (Expr)) { return vec3(1); }

#define ArrayCount(A) ( sizeof((A)) / sizeof((A)[0]) )
#define Squared(A) ( (A)*(A) )

#define in
#define out
#define inout

typedef unsigned int uint;

static vec3 OutBuffer[512][512];

static inline bool
CubeSphereIntersection(vec3 Min, vec3 Max)
{
    const vec3 S = vec3(16);
    const f32 R = 8.0f;

    f32 DistanceSqToCube = R * R;

    //printf("MIN (%f, %f, %f), MAX (%f, %f, %f)", Min.X, Min.Y, Min.Z, Max.X, Max.Y, Max.Z);

    // STACKOVER
    if (S.X < Min.X) DistanceSqToCube -= Squared(S.X - Min.X);
    else if (S.X > Max.X) DistanceSqToCube -= Squared(S.X - Max.X);

    if (S.Y < Min.Y) DistanceSqToCube -= Squared(S.Y - Min.Y);
    else if (S.Y > Max.Y) DistanceSqToCube -= Squared(S.Y - Max.Y);

    if (S.Z < Min.Z) DistanceSqToCube -= Squared(S.Z - Min.Z);
    else if (S.Z > Max.Z) DistanceSqToCube -= Squared(S.Z - Max.Z);

    if (DistanceSqToCube > 0)
    {
        //printf("MIN (%f, %f, %f), MAX (%f, %f, %f)", Min.X, Min.Y, Min.Z, Max.X, Max.Y, Max.Z);
        //printf("        TRUE\n");
        return true;
    }
    else
    {
        //printf("        FALSE\n");
        return false;
    }
}
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

static uint MaxDepthUniform = 2;
static uint ScaleExponentUniform = 5;
static uint BiasUniform = (MaxDepthUniform > ScaleExponentUniform) ? (MaxDepthUniform - ScaleExponentUniform) : 0;
static float InvBiasUniform = BiasUniform > 0 ? 1.0f / float(BiasUniform) : 1.0f;
static uint EntriesPerBlockUniform = SVO_ENTRIES_PER_BLOCK;
static uint FarPtrsPerBlockUniform = SVO_FAR_PTRS_PER_BLOCK;

static vec3 ViewPosUniform = vec3(-5, -5, -512);
static mat3 ViewMatrixUniform;


struct svo_input
{
    uint Nodes[];
};

struct far_ptr_input
{
    far_ptr FarPtrs[];
};

static far_ptr_input SvoFarPtrBuffer = {
    {
    // {{{
{ 4,1 },
{ 0,0 },
{ 8,0 },
{ 12,1 },
{ 16,0 },
{ 20,1 },
{ 24,0 },
{ 28,1 },
{ 1,0,},
{ 32,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 1,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 1,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 1,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
{ 0,0 },
//}}}
    }
};

static svo* DEBUGSvo;

// BLKSZ 4096
static svo_input SvoInputBuffer4096 = {
    // {{{
130816,
622592,
671744,
729088,
790528,
854016,
918528,
983552,
1048832,
1179392,
1703680,
2227968,
2752256,
3276544,
3800832,
4325120,
4849408,
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
// }}}
};

// BLKSZ 2
static svo_input SvoInputBuffer = {
    // {{{
    130816,
    2147516416,
    2147500032,
    2147557376,
    2147487744,
    2147551232,
    2147484672,
    2147549696,
    2147549440,
    2147548928,
    32896,
    64764,
    64250,
    65535,
    61166,
    65535,
    65535,
    65535,
    130816,
    64764,
    16448,
    65535,
    62965,
    65535,
    56797,
    65535,
    65535,
    2147548928,
    64250,
    65535,
    8224,
    62451,
    65535,
    65535,
    48059,
    65535,
    130816,
    65535,
    62965,
    62451,
    4112,
    65535,
    65535,
    65535,
    30583,
    2147548928,
    61166,
    65535,
    65535,
    65535,
    2056,
    53199,
    44975,
    65535,
    130816,
    65535,
    56797,
    65535,
    65535,
    53199,
    1028,
    65535,
    24415,
    2147548928,
    65535,
    65535,
    48059,
    65535,
    44975,
    65535,
    514,
    16191,
    130816,
    65535,
    65535,
    65535,
    30583,
    65535,
    24415,
    16191,
    257,
    0,
    // }}}
};

static uint PrevOct;


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

uint GetNodeChild(in uint ParentNode, in uint Oct, inout uint& ParentBlkIndex)
{
    uint ChildPtr = (ParentNode & SVO_NODE_CHILD_PTR_MASK) >> 16;
    uint OccBits = (ParentNode & SVO_NODE_OCCUPIED_MASK) >> 8; 
    uint LeafBits = (ParentNode & SVO_NODE_LEAF_MASK);
    uint OccupiedNonLeafOcts = OccBits & (~LeafBits);
    uint SetBitsBehindOctIdx = (1 << Oct) - 1;

    uint ChildOffset = BitCount(OccupiedNonLeafOcts & SetBitsBehindOctIdx); 

    if (! bool(ParentNode & SVO_FAR_PTR_BIT_MASK))
    {
        uint Child = SvoInputBuffer.Nodes[ParentBlkIndex*EntriesPerBlockUniform + ChildPtr + ChildOffset];
        ParentBlkIndex += (ChildPtr + ChildOffset) / (EntriesPerBlockUniform);

        return Child;
    }
    else
    {
        // Find the far ptr associated with this node. To do this, we need to compute
        // the byte offset for this block, then index into that block's far ptr
        // list for this node.
        uint FarPtrIndex = (ParentNode & SVO_NODE_CHILD_PTR_MASK) >> 16;
        uint FarPtrBlkStart = ParentBlkIndex*FarPtrsPerBlockUniform;
        far_ptr FarPtr = SvoFarPtrBuffer.FarPtrs[FarPtrBlkStart + FarPtrIndex];

        // Skip to the block containing the first child
        ParentBlkIndex = FarPtr.BlkIndex;
        uint ChildBlkStart = ParentBlkIndex * EntriesPerBlockUniform;

        // Skip any blocks required to get to the actual child node
        ParentBlkIndex += (FarPtr.NodeOffset + ChildOffset) / EntriesPerBlockUniform;

        uint Child = SvoInputBuffer.Nodes[ChildBlkStart + FarPtr.NodeOffset + ChildOffset];

        return Child;
    }
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

    if (Sgn.X == 0 && OctSgn.X != 0) return false;
    if (Sgn.Y == 0 && OctSgn.Y != 0) return false;
    if (Sgn.Z == 0 && OctSgn.Z != 0) return false;

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
        NextOct ^= 1;
    }

    if (tMax >= tValues.Y)
    {
        NextOct ^= 2;
    }

    if (tMax >= tValues.Z)
    {
        NextOct ^= 4;
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
    return (Node & (256 << Oct)) != 0;
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
    return vec3(uvec3(Oct));
}

struct st_frame
{
    uint Node;
    uint Depth;
    int Scale;
    float tMin;
    vec3 ParentCentre;
    uint BlkIndex;
    uint Test;
};

vec3 Raycast(in ray R)
{
    // Scale up by the tree bias.
    int Scale = (1 << ScaleExponentUniform) << BiasUniform;

    uint BlkIndex = 0;
    
    vec3 RootMin = vec3(0);
    vec3 RootMax = vec3(Scale) * InvBiasUniform;
    vec3 Sgn = Sign(R.Dir);

    // Intersection of the ray with the root cube (i.e. entire tree)
    ray_intersection CurrentIntersection = ComputeRayBoxIntersection(R, RootMin, RootMax);

    int Step;
    uint CurrentOct;
    uint CurrentDepth;

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
        CurrentOct = GetOctant(RayP, ParentCentre*InvBiasUniform);
        
        // Initialise depth to 1
        CurrentDepth = 1;

        // Stack of previous voxels
        st_frame Stack[MAX_STEPS + 1] = { 0 };
        Scale >>= 1;
        Stack[CurrentDepth] = { ParentNode, 
                                CurrentDepth,
                                Scale,
                                CurrentIntersection.tMin,
                                ParentCentre,
                                BlkIndex,
                                88};

        // Begin stepping along the ray
        for (Step = 0; Step < MAX_STEPS; ++Step)
        {
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
                        return vec3(1, 0, 1);
                    }
                    else
                    {
                        // Voxel has children --- execute push
                        // NOTE(Liam): BlkIndex (potentially) updated here
                        ParentNode = GetNodeChild(ParentNode, CurrentOct, BlkIndex);

                        CurrentOct = GetOctant(RayP, NodeCentre*InvBiasUniform);
                        ParentCentre = NodeCentre;
                        Scale >>= 1;
                        ++CurrentDepth;

                        Stack[CurrentDepth] = { ParentNode, CurrentDepth, Scale, CurrentIntersection.tMin, ParentCentre, BlkIndex, 88 };

                        continue;
                    }
                }

                // Octant not occupied, need to handle advance/pop
                uint NextOct = GetNextOctant(CurrentIntersection.tMax, CurrentIntersection.tMaxV, CurrentOct);

                RayP = R.Origin + (CurrentIntersection.tMax + 1) * R.Dir;
                if (Stack[CurrentDepth].Test != 88) return vec3(1, 0, 0);

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
                    //
                    // Find the highest differing bit
                    uvec3 HDB = FindMSB(NodeCentreBits ^ RayPBits);

                    uint M = 0;
                    if (HDB.X > M && HDB.X < ScaleExponentUniform + BiasUniform) M = HDB.X;
                    if (HDB.Y > M && HDB.Y < ScaleExponentUniform + BiasUniform) M = HDB.Y;
                    if (HDB.Z > M && HDB.Z < ScaleExponentUniform + BiasUniform) M = HDB.Z;
                    if (M == 0) return vec3(0.5, 0, 0.5);

                    uint NextDepth = ((ScaleExponentUniform + BiasUniform) - M);

                    if (NextDepth > CurrentDepth) return vec3(0.6, 0, 0.16);

                    if (NextDepth >= 0 && NextDepth <= MAX_STEPS)
                    {
                        CurrentDepth = NextDepth;
                        Scale = Stack[CurrentDepth].Scale;
                        ParentCentre = Stack[CurrentDepth].ParentCentre;
                        ParentNode = Stack[CurrentDepth].Node;
                        BlkIndex = Stack[CurrentDepth].BlkIndex;

                        CurrentOct = GetOctant(RayP, ParentCentre*InvBiasUniform);
                    }
                    else
                    {
                        return vec3(1, 1, 0);
                    }
                }
            }
            else
            {
                return vec3(0.16);
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
    DEBUGSvo = CreateSparseVoxelOctree(SABRE_SCALE_EXPONENT, SABRE_MAX_TREE_DEPTH, &CubeSphereIntersection);

    vec3 Right = vec3(0.105397, 0.000000, -0.994430);
    vec3 Up = vec3(-0.110848, 0.993768, -0.011748);
    vec3 Forward = vec3(-0.988233, -0.111469, -0.104740);
    vec3 Position = vec3(73.017464, 11.058921, 15.149199);

    mat3 View = {{
        { Right.X, Right.Y, Right.Z },
        { Up.X, Up.Y, Up.Z },
        { -Forward.X, -Forward.Y, -Forward.Z },
    }};

    const uint Nmsk = ~(0xFFFF << 16);
    for (int I = 0; I < (83); ++I)
    {
        uint N = SvoInputBuffer.Nodes[I] & Nmsk;
        uint N2 = SvoInputBuffer4096.Nodes[I] & Nmsk;
        printf("Chk\n");
        //assert(N == N2);
    }

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

    DeleteSparseVoxelOctree(DEBUGSvo);

    return 0;
}
