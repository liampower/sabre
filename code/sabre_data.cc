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
#define SVO_NODE_CHILD_PTR_MASK 0x7FFF0000U
#define SVO_FAR_PTR_BIT_MASK    0x80000000U

#define MAX_STEPS 128
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

struct far_ptr
{
    uint BlkIndex;
    uint NodeOffset;
};

layout (local_size_x = 8, local_size_y = 8) in;

layout (rgba32f, binding = 0) uniform image2D OutputImgUniform;

uniform uint MaxDepthUniform;
uniform uint BlockCountUniform;
uniform uint ScaleExponentUniform;
uniform uint EntriesPerBlockUniform;
uniform uint FarPtrsPerBlockUniform;

uniform uint BiasUniform;
uniform float InvBiasUniform;

uniform vec3 ViewPosUniform;
uniform mat3 ViewMatrixUniform;

//uniform sampler1D NormalsDataUniform;
uniform sampler1DArray NormalsDataUniform;
uniform sampler3D MapDataUniform;

const uvec3 OCT_BITS = uvec3(1, 2, 4);
const vec3 OCT_BITS_F32 = vec3(1.0, 2.0, 4.0);

layout (std430, binding = 3) readonly buffer svo_input
{
    uint Nodes[];
} SvoInputBuffer;

layout (std430, binding = 4) readonly buffer svo_far_ptr_index
{
    far_ptr FarPtrs[];
} SvoFarPtrBuffer;


float MaxComponent(vec3 V)
{
    return max(max(V.x, V.y), V.z);
}

float MinComponent(vec3 V)
{
    return min(min(V.x, V.y), V.z);
}

uint MaxComponentU(uvec3 V)
{
    return max(max(V.x, V.y), V.z);
}

vec3 GammaCorrect(vec3 C)
{
    const vec3 Gamma = vec3(1.0 / 2.2);

    return pow(C, Gamma);
}

uint ComputeLeafHash(uvec3 LeafCentre)
{
    // Morton-encode
    const uvec3 Sh = uvec3(0, 1, 2);

    LeafCentre &= 0X000003ff;
    LeafCentre = (LeafCentre ^ (LeafCentre << 16)) & 0Xff0000ff;
    LeafCentre = (LeafCentre ^ (LeafCentre <<  8)) & 0X0300f00f;
    LeafCentre = (LeafCentre ^ (LeafCentre <<  4)) & 0X030c30c3;
    LeafCentre = (LeafCentre ^ (LeafCentre <<  2)) & 0X09249249;

    LeafCentre << Sh;

    uint Key = LeafCentre.x + LeafCentre.y + LeafCentre.z;

    // FNV hash
    const uint PRIME = 16777619;

    uint Hash = 2166136261;

    Hash ^= Key &  0x000000FF;
    Hash *= PRIME;

    Hash ^= (Key & 0x0000FF00) >> 8;
    Hash *= PRIME;

    Hash ^= (Key & 0x00FF0000) >> 16;
    Hash *= PRIME;

    Hash ^= (Key & 0xFF000000) >> 24;
    Hash *= PRIME;

    return Hash;
}

uint GetNodeChildIndex(in uint ParentNode, in uint Oct, inout uint ParentBlkIndex)
{
    uint ChildPtr = (ParentNode & SVO_NODE_CHILD_PTR_MASK) >> 16;
    uint OccBits = (ParentNode & SVO_NODE_OCCUPIED_MASK) >> 8; 
    uint LeafBits = (ParentNode & SVO_NODE_LEAF_MASK);
    uint OccupiedNonLeafOcts = OccBits & (~LeafBits);
    uint SetBitsBehindOctIdx = (1 << Oct) - 1;

    uint ChildOffset = bitCount(OccupiedNonLeafOcts & SetBitsBehindOctIdx); 

    if (! bool(ParentNode & SVO_FAR_PTR_BIT_MASK))
    {
        uint ChildIndex = ParentBlkIndex*EntriesPerBlockUniform + ChildPtr + ChildOffset;
        ParentBlkIndex += (ChildPtr + ChildOffset) / (EntriesPerBlockUniform);

        return ChildIndex;
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

        uint ChildIndex = ChildBlkStart + FarPtr.NodeOffset + ChildOffset;

        return ChildIndex;
    }
}

ray_intersection ComputeRayBoxIntersection(in ray R, in vec3 vMin, in vec3 vMax)
{
    vec3 t0 = (vMin - R.Origin) * R.InvDir; // Distance along ray to vmin planes
    vec3 t1 = (vMax - R.Origin) * R.InvDir; // Distance along ray to vmax planes

    vec3 tMin = min(t0, t1); // Minimums of all distances
    vec3 tMax = max(t0, t1); // Maximums of all distances

    float ttMin = MaxComponent(tMin); // Largest of the min distances (closest to box)
    float ttMax = MinComponent(tMax); // Smallest of max distances (closest to box)

    ray_intersection Result = { ttMin, ttMax, tMax, tMin };
    return Result;
}


bool IsAdvanceValid(in uint NewOct, in uint OldOct, in vec3 RaySgn)
{
    ivec3 NewOctBits = ivec3(NewOct) & ivec3(OCT_BITS);
    ivec3 OldOctBits = ivec3(OldOct) & ivec3(OCT_BITS);

    vec3 OctSgn = sign(NewOctBits - OldOctBits);

    return any(equal(RaySgn, OctSgn));
}


uint GetNextOctant(in float tMax, in vec3 tValues, in uint CurrentOct)
{
    // For every element of tValues that is equal to tMax, XOR the corresponding
    // bit in CurrentOct.
    // if tMax >= tValues.x then NextOct ^= 1;
    // if tMax >= tValues.y then NextOct ^= 2;
    // if tMax >= tValues.z then NextOct ^= 4;
    uvec3 XorMsk3 = uvec3(greaterThanEqual(vec3(tMax), tValues)) * OCT_BITS;
    uint XorMsk = XorMsk3.x + XorMsk3.y + XorMsk3.z;
    return CurrentOct ^ XorMsk;
}

uint GetOctant(in vec3 P, in vec3 ParentCentreP)
{
    uvec3 G = uvec3(greaterThan(P, ParentCentreP));

    // It is likely that a DP is actually *not* faster
    // here because the GLSL `dot` function can only return floats,
    // so we would need to convert this back into a uint. 
    return G.x + G.y*2 + G.z*4;
}

bool IsOctantOccupied(in uint Node, in uint Oct)
{
    return (Node & (256 << Oct)) != 0;
}

bool IsOctantLeaf(in uint Node, in uint Oct)
{
    return (Node & (1 << Oct)) != 0;
}

vec3 Oct2Cr(in uint Oct)
{
    return vec3(bvec3(uvec3(Oct) & uvec3(1, 2, 4))).bgr;
}

struct st_frame
{
    uint Node;
    uint LeafIndex;
    int Scale;
    vec3 ParentCentre;
    uint BlkIndex;
};


// NOTE(Liam): Lessons learned in GPU optimisation:
// * Functions are (mostly) fine. The compiler inlines nearly everything
// * Even very trivial ifs can cause bad asm. Nearly always worth optimising these out
// * Swizzling is free in h.w.
// * Integer/float conversions aren't free
// * It appears we can only do 32-bit loads/stores, so this means that for an 8-byte structure
//   we need two corresponding movs, etc.
// * No h.w. instruction for `sign` - deceptively slow, especially with conversions
// * Vector min/max map directly to asm instructions

uint ComputeFnvHash(uint Key)
{
    const uint PRIME = 16777619;

    uint Hash = 2166136261;

    Hash ^= Key &  0x000000FF;
    Hash *= PRIME;

    Hash ^= (Key & 0x0000FF00) >> 8;
    Hash *= PRIME;

    Hash ^= (Key & 0x00FF0000) >> 16;
    Hash *= PRIME;

    Hash ^= (Key & 0xFF000000) >> 24;
    Hash *= PRIME;

    return Hash;
}


vec3 Raycast(in ray R)
{
    // Scale up by the tree bias.
    int Scale = (1 << ScaleExponentUniform) << BiasUniform;

    const float BiasScale = (1.0 / InvBiasUniform);

    uint BlkIndex = 0;
    
    vec3 RootMin = vec3(0);
    vec3 RootMax = vec3(Scale) * InvBiasUniform;
    vec3 RaySgn = sign(R.Dir);

    // Intersection of the ray with the root cube (i.e. entire tree)
    ray_intersection CurrentIntersection = ComputeRayBoxIntersection(R, RootMin, RootMax);

    int Step;
    uint CurrentOct;
    uint CurrentDepth;
    uint LeafIndex = 0;

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
        st_frame Stack[MAX_STEPS + 1];

        // Drop down one scale value to the initial children of the
        // root node.
        Scale >>= 1;

        Stack[CurrentDepth] = st_frame(ParentNode, 
                                       LeafIndex,
                                       Scale,
                                       ParentCentre,
                                       BlkIndex);

        // Begin stepping along the ray
        for (Step = 0; Step < MAX_STEPS; ++Step)
        {
            // Radius of the current octant's cube (half the current scale);
            vec3 Rad = vec3(Scale >> 1);

            // Get the centre position of this octant
            vec3 OctSgn = mix(vec3(-1), vec3(1), bvec3(CurrentOct & OCT_BITS));
            vec3 NodeCentre = ParentCentre + (OctSgn * Rad);

            vec3 NodeMin = (NodeCentre - Rad) * InvBiasUniform;
            vec3 NodeMax = (NodeCentre + Rad) * InvBiasUniform;

            CurrentIntersection = ComputeRayBoxIntersection(R, NodeMin, NodeMax);

            // TODO(Liam): There are some spots occurring due to (probably) error in
            // the intersection. Investigate an epsilon value that lets us eliminate
            // these spots. The epsilon should probably be the same as the step value
            // we use below.

            if (CurrentIntersection.tMin <= CurrentIntersection.tMax)
            {
                // Ray hit this voxel
                
                // Check if voxel occupied
                if (IsOctantOccupied(ParentNode, CurrentOct))
                {
                    // Octant is occupied, check if leaf
                    if (IsOctantLeaf(ParentNode, CurrentOct))
                    {
                        return texelFetch(MapDataUniform, ivec3(NodeCentre.xyz), 0).rgb;

                        if (all(greaterThan(NodeCentre, vec3(12, 12, 12)))) return vec3(1,1,0);
                        //if (any(equal(NodeCentre, vec3(12, 12, 12)))) return vec3(0.5)*vec3(equal(NodeCentre, vec3(12)));
                        if (NodeCentre.z == 12 && NodeCentre.x == 28 && NodeCentre.y == 28) return vec3(1, 0, 1);
                        return vec3(1, 0, 0);
                        //uint Lmsk = SVO_NODE_LEAF_MASK & ~(1 << CurrentOct);

                        // PROBLEM: LeafIndex isn't absolute here; it's the offset from the root.
 
                        //LeafIndex += bitCount(ParentNode & Lmsk);

                        //uint Hash = ComputeLeafHash(uvec3(NodeCentre));
                        //return vec3(float(Hash) / -1U);
                       // return vec3(float(LeafIndex) / 64.0);

                        /*vec3 N = texelFetch(NormalsDataUniform, int(LeafIndex), 0).xyz;
                        vec3 Ldir = normalize((NodeCentre*InvBiasUniform) - vec3(32, 0, 0));*/



                        //return vec3(dot(N, Ldir));
                    }
                    else
                    {
                        // Voxel has children --- execute push
                        // NOTE(Liam): BlkIndex (potentially) updated here
                        ParentNode = SvoInputBuffer.Nodes[GetNodeChildIndex(ParentNode, CurrentOct, BlkIndex /*out*/)];
                        CurrentOct = GetOctant(RayP, NodeCentre*InvBiasUniform);
                        ParentCentre = NodeCentre;
                        Scale >>= 1;
                        ++CurrentDepth;
                        LeafIndex += bitCount(ParentNode & SVO_NODE_LEAF_MASK);

                        Stack[CurrentDepth] = st_frame(ParentNode,
                                                   LeafIndex,
                                                   Scale,
                                                   ParentCentre,
                                                   BlkIndex);


                        continue;
                    }
                }

                // Octant not occupied, need to handle advance/pop
                uint NextOct = GetNextOctant(CurrentIntersection.tMax, CurrentIntersection.tMaxV, CurrentOct);

                RayP = R.Origin + (CurrentIntersection.tMax + 0.015625) * R.Dir;

                if (IsAdvanceValid(NextOct, CurrentOct, RaySgn))
                {
                    CurrentOct = NextOct;
                }
                else
                {
                    // Determined that NodeCentre is never < 0
                    uvec3 NodeCentreBits = uvec3(NodeCentre);
                    uvec3 RayPBits = uvec3(RayP * BiasScale);

                    // NOTE(Liam): It is **okay** to have negative values here
                    // because the HDB will end up being equal to ScaleExponentUniform.
                    //
                    // Find the highest differing bit
                    uvec3 HDB = findMSB(NodeCentreBits ^ RayPBits);

                    uint M = MaxComponentU(mix(uvec3(0), HDB, lessThan(HDB, uvec3(ScaleExponentUniform + BiasUniform))));

                    uint NextDepth = ((ScaleExponentUniform + BiasUniform) - M);

                    if (NextDepth <= MAX_STEPS && NextDepth < CurrentDepth)
                    {
                        CurrentDepth = NextDepth;
                        Scale = Stack[CurrentDepth].Scale;
                        ParentCentre = Stack[CurrentDepth].ParentCentre;
                        ParentNode = Stack[CurrentDepth].Node;
                        BlkIndex = Stack[CurrentDepth].BlkIndex;
                        LeafIndex = Stack[CurrentDepth].LeafIndex;

                        CurrentOct = GetOctant(RayP, ParentCentre*InvBiasUniform);
                    }
                    else
                    {
                        break;
                    }
                }
            }
            else
            {
                break;
            }
        }
    }
    else
    {
        // Ray doesn't hit octree --- output background colour
        return vec3(0.12);
    }

    return vec3(0.12);
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

    RayD += 0.0001;
    ray R = { RayP, RayD, 1.0 / RayD };

    vec3 OutCr = Raycast(R);

    imageStore(OutputImgUniform, ivec2(PixelCoords), vec4(OutCr, 1.0));
}
)GLSL";

