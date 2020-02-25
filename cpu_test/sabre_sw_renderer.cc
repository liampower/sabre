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

#define SVO_FAR_PTRS_PER_BLOCK  4096
//#define SVO_ENTRIES_PER_BLOCK   1

#define MAX_STEPS 64
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

static uint MaxDepthUniform = 4;
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
#if 0
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
#endif
    { 
        {0,0},
    }
};

static svo* DEBUGSvo;


// BLKSZ 2
static svo_input SvoInputBuffer = {
    // {{{
130816,
622592,
737280,
860160,
987136,
1116160,
1246208,
1376768,
1507584,
688128,
65535,
802816,
65535,
925696,
65535,
1052672,
65535,
1181696,
65535,
1311744,
65535,
1442304,
65535,
1573120,
65535,
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
    
    uvec3 NewOctBits = uvec3(NewOct) & OctBits;
    uvec3 OldOctBits = uvec3(OldOct) & OctBits;

    vec3 OctSgn = Sign(NewOctBits - OldOctBits);

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

    return ParentP + (vec3(X, Y, Z) * float(Radius));
}

vec3 Oct2Cr(in uint Oct)
{
    return vec3(uvec3(Oct));
}

struct st_frame
{
    uint Node;
    int Scale;
    vec3 ParentCentre;
    uint BlkIndex;
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

        float StkThreshold = CurrentIntersection.tMax;

        // Current octant the ray is in (confirmed good)
        CurrentOct = GetOctant(RayP, ParentCentre*InvBiasUniform);
        
        // Initialise depth to 1
        CurrentDepth = 1;

        // Stack of previous voxels
        st_frame Stack[MAX_STEPS + 1] = { 0 };

        // Drop down one scale value to the initial children of the
        // root node.
        Scale >>= 1;

        Stack[CurrentDepth] = { ParentNode, 
                                Scale,
                                ParentCentre,
                                BlkIndex };

        bool Skip = false;

        // Begin stepping along the ray
        for (Step = 0; Step < MAX_STEPS; ++Step)
        {
            // Get the centre position of this octant
            vec3 NodeCentre = GetNodeCentreP(CurrentOct, Scale, ParentCentre);
            vec3 NodeMin = (NodeCentre - vec3(Scale >> 1)) * InvBiasUniform;
            vec3 NodeMax = (NodeCentre + vec3(Scale >> 1)) * InvBiasUniform;

            CurrentIntersection = ComputeRayBoxIntersection(R, NodeMin, NodeMax);

            // TODO(Liam): There are some spots occurring due to (probably) error in
            // the intersection. Investigate an epsilon value that lets us eliminate
            // these spots. The epsilon should probably be the same as the step value
            // we use below.
            if (CurrentIntersection.tMin < CurrentIntersection.tMax && CurrentIntersection.tMax > 0)
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

                        if (CurrentIntersection.tMax < StkThreshold || true)
                        {
                            Stack[CurrentDepth] = { ParentNode,
                                                    Scale, 
                                                    ParentCentre,
                                                    BlkIndex };
                        }
                        Skip = true;
                        StkThreshold = CurrentIntersection.tMax;

                        continue;
                    }
                }

                // Octant not occupied, need to handle advance/pop
                uint NextOct = GetNextOctant(CurrentIntersection.tMax, CurrentIntersection.tMaxV, CurrentOct);

                RayP = R.Origin + (CurrentIntersection.tMax + 0.0001) * R.Dir;
                Skip = false;

                if (IsAdvanceValid(NextOct, CurrentOct, R.Dir))
                {
                    CurrentOct = NextOct;
                }
                else
                {
                    // Determined that NodeCentre is never < 0
                    uvec3 NodeCentreBits = uvec3(NodeCentre);
                    uvec3 RayPBits = uvec3(RayP * (1.0f / InvBiasUniform));

                    // NOTE(Liam): It is **okay** to have negative values here
                    // because the HDB will end up being equal to ScaleExponentUniform.
                    //
                    // Find the highest differing bit
                    uvec3 HDB = FindMSB(NodeCentreBits ^ RayPBits);

                    uint M = 0;
                    if (HDB.X > M && HDB.X < ScaleExponentUniform + BiasUniform) M = HDB.X;
                    if (HDB.Y > M && HDB.Y < ScaleExponentUniform + BiasUniform) M = HDB.Y;
                    if (HDB.Z > M && HDB.Z < ScaleExponentUniform + BiasUniform) M = HDB.Z;

                    uint NextDepth = ((ScaleExponentUniform + BiasUniform) - M);

                    if (NextDepth >= CurrentDepth) return vec3(1, 0, 0);

                    if (NextDepth <= MAX_STEPS)
                    {
                        CurrentDepth = NextDepth;
                        Scale = Stack[CurrentDepth].Scale;
                        ParentCentre = Stack[CurrentDepth].ParentCentre;
                        ParentNode = Stack[CurrentDepth].Node;
                        BlkIndex = Stack[CurrentDepth].BlkIndex;

                        CurrentOct = GetOctant(RayP, ParentCentre*InvBiasUniform);
                        StkThreshold = 0.0f;
                    }
                    else
                    {
                        return vec3(1, 1, 0);
                    }
                }
            }
            else
            {
                return vec3(0.16f);
            }
        }
    }
    else
    {
        // Ray doesn't hit octree --- output background colour
        return vec3(0.12f);
    }

    return vec3(0, 1, 0);
}

void main()
{
    //DEBUGSvo = CreateSparseVoxelOctree(SABRE_SCALE_EXPONENT, SABRE_MAX_TREE_DEPTH, &CubeSphereIntersection);

#if 1
    vec3 Right= vec3(0.944949f, -0.000000f, 0.327218f);
    vec3 Up= vec3(-0.026243f, 0.996779f, 0.075784f);
    vec3 Forward= vec3(0.326164f, 0.080199f, -0.941905f);
    vec3 Position= vec3(25.368835f, 27.123180f, 14.338293f);

#else
   vec3 Right = vec3(1.000000, -0.000000, 0.000000);
   vec3 Up = vec3(0.000000, 1.000000, 0.000000);
   vec3 Forward = vec3(0.000000, 0.000000, -1.000000);
   vec3 Position = vec3(19.399998, 17.999998, -35.600060);
#endif
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

    //DeleteSparseVoxelOctree(DEBUGSvo);
}
