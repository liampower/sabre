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

#define SVO_FAR_PTRS_PER_BLOCK  1
//#define SVO_ENTRIES_PER_BLOCK   1

#define MAX_STEPS 16
#define SCREEN_DIM 512

#define SABRE_MAX_TREE_DEPTH 3
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
static uint BlockCountUniform = 1;
static uint ScaleExponentUniform = 5;

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
{1,0},
{8,0},
{9,0},
{10,0},
{11,0},
{12,0},
{13,0},
{14,0},
{15,0},
    }
    // }}}
};

static svo* DEBUGSvo;

// BLKSZ 4096
static svo_input SvoInputBuffer4096 = {
130816,
622592,
671744,
729088,
790528,
854016,
918528,
983552,
1048832,
65535,
65535,
65535,
65535,
65535,
65535,
65535,
65535,
};

// BLKSZ 1
static svo_input SvoInputBuffer = {
    
2147548928,
2147516416,
2147500032,
2147491840,
2147487744,
2147485696,
2147484672,
2147484160,
2147483904,
65535,
65535,
65535,
65535,
65535,
65535,
65535,
65535,
/*
*/
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

uint GetNodeChild(in uint ParentNode, in uint Oct, inout int& BlkIndex)
{
    uint ChildPtr = (ParentNode & SVO_NODE_CHILD_PTR_MASK) >> 16;
    uint OccBits = (ParentNode & SVO_NODE_OCCUPIED_MASK) >> 8; 
    uint LeafBits = (ParentNode & SVO_NODE_LEAF_MASK);
    uint OccupiedNonLeafOcts = OccBits & (~LeafBits);
    uint SetBitsBehindOctIdx = (1 << Oct) - 1;

    uint ChildOffset = BitCount(OccupiedNonLeafOcts & SetBitsBehindOctIdx); 

    if (! bool(ParentNode & SVO_FAR_PTR_BIT_MASK))
    {
        BlkIndex = int(ChildPtr + ChildOffset) / SVO_ENTRIES_PER_BLOCK;

        return SvoInputBuffer.Nodes[ChildPtr + ChildOffset];
    }
    else
    {
        // Find the far ptr associated with this node. To do this, we need to compute
        // the byte offset for this block, then index into that block's far ptr
        // list for this node.
        uint FarPtrIndex = (ParentNode & SVO_NODE_CHILD_PTR_MASK) >> 16;
        far_ptr FarPtr = SvoFarPtrBuffer.FarPtrs[BlkIndex*SVO_FAR_PTRS_PER_BLOCK + FarPtrIndex];

        BlkIndex += FarPtr.BlkOffset;
        uint ChildBlkStart = BlkIndex * SVO_ENTRIES_PER_BLOCK;

        return SvoInputBuffer.Nodes[ChildBlkStart + FarPtr.NodeOffset + ChildOffset];
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

vec3 Oct2Cr(in uint Oct)
{
    return vec3(uvec3(Oct));
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
    int BlkIndex;
};

vec3 Raycast(in ray R)
{
    // Extant of the root cube
    int Scale = 1 << (ScaleExponentUniform);

    int BlkIndex = 0;
    
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
        Stack[CurrentDepth] = { ParentNode, CurrentDepth, Scale, CurrentIntersection.tMin, ParentCentre, BlkIndex };

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
                        ParentNode = GetNodeChild(ParentNode, CurrentOct, BlkIndex);
                        CurrentOct = GetOctant(RayP, NodeCentre);
                        ParentCentre = NodeCentre;
                        Scale >>= 1;
                        ++CurrentDepth;

                        Stack[CurrentDepth] = { ParentNode, CurrentDepth, Scale, CurrentIntersection.tMin, ParentCentre, BlkIndex };

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
                    if (HighestDiffBits.X > M && HighestDiffBits.X < ScaleExponentUniform) M = HighestDiffBits.X;
                    if (HighestDiffBits.Y > M && HighestDiffBits.Y < ScaleExponentUniform) M = HighestDiffBits.Y;
                    if (HighestDiffBits.Z > M && HighestDiffBits.Z < ScaleExponentUniform) M = HighestDiffBits.Z;

                    uint NextDepth = (ScaleExponentUniform - M);

                    if (NextDepth >= 0 && NextDepth <= MAX_STEPS)
                    {
                        CurrentDepth = NextDepth;
                        Scale = Stack[CurrentDepth].Scale;
                        ParentCentre = Stack[CurrentDepth].ParentCentre;
                        ParentNode = Stack[CurrentDepth].Node;
                        BlkIndex = Stack[CurrentDepth].BlkIndex;

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
    vec3 Right =  vec3(0.821647, -0.000000, 0.569997);
    vec3 Up = vec3(-0.095058, 0.985996, 0.137025);
    vec3 Forward = vec3(0.562015, 0.166769, -0.810140);
    vec3 Position = vec3(4.459143, 10.918401, 28.265924);

    mat3 View = {{
        { Right.X, Right.Y, Right.Z },
        { Up.X, Up.Y, Up.Z },
        { -Forward.X, -Forward.Y, -Forward.Z },
    }};

#if 0
    const uint Nmsk = ~(0xFFFF << 16);
    for (int I = 0; I < ArrayCount(A2); ++I)
    {
        uint N = SvoInputBuffer.Nodes[I] & Nmsk;
        uint N2 = A2[I] & Nmsk;
        assert(N == N2);
    }
#endif

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
