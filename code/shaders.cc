#include "shaders.h"

// {{{ MainVertexCode
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
// }}}

// {{{ MainFragmentCode
extern const char* const MainFragmentCode = R"GLSL(
#version 450 core

out vec4 FragCr;

uniform sampler2D OutputTextureUniform;

struct luma
{
    float N, S, E, W, M;
    float Min, Max, Contrast;

    float NE, NW, SE, SW;
};

struct edge
{
    bool Horz;
    float StepSize;
};


in vec2 UV;


float ComputeBlendFactor(luma L)
{
    float F = 2.0 * (L.N + L.E + L.S + L.W);
    F += L.NE + L.NW + L.SE + L.SW;
    F *= 1/12.0;
    F = abs(F - L.M);
    F = clamp(F / L.Contrast, 0, 1);
    float B = smoothstep(0, 1, F);
    
    return B*B*0.75;
}

edge ComputeEdge(in luma L)
{
    edge E;
    float HorzContrast =
        abs(L.N + L.S - 2 * L.M) * 2 +
        abs(L.NE + L.SE - 2 * L.E) +
        abs(L.NW + L.SW - 2 * L.W);
    float VertContrast =
        abs(L.E + L.W - 2 * L.M) * 2 +
        abs(L.NE + L.NW - 2 * L.N) +
        abs(L.SE + L.SW - 2 * L.S);

    E.Horz = HorzContrast >= VertContrast;
    E.StepSize = E.Horz ? 1/512.0 : 1/512.0;
    float pLuminance = E.Horz ? L.N : L.E;
    float nLuminance = E.Horz ? L.S : L.W;
    float pGradient = abs(pLuminance - L.M);
    float nGradient = abs(nLuminance - L.M);

    if (pGradient < nGradient) {
        E.StepSize = -E.StepSize;
    }

    return E;
}

luma SampleLuma(vec2 UV)
{
    const vec2 Offsets[8] = {
        vec2(0, 1),
        vec2(1, 0),
        vec2(0, -1),
        vec2(-1, 0),

        vec2(1, 1),
        vec2(-1, 1),
        vec2(1, -1),
        vec2(-1, -1),
    };
    const float Ts = 1/512.0;

    float M = texture(OutputTextureUniform, UV).g;
    float N = texture(OutputTextureUniform, UV + Offsets[0]*Ts).g;
    float E = texture(OutputTextureUniform, UV + Offsets[1]*Ts).g;
    float S = texture(OutputTextureUniform, UV + Offsets[2]*Ts).g;
    float W = texture(OutputTextureUniform, UV + Offsets[3]*Ts).g;
    float NE = texture(OutputTextureUniform, UV + Offsets[4]*Ts).g;
    float NW = texture(OutputTextureUniform, UV + Offsets[5]*Ts).g;
    float SE = texture(OutputTextureUniform, UV + Offsets[6]*Ts).g;
    float SW = texture(OutputTextureUniform, UV + Offsets[7]*Ts).g;

    float Min = min(min(min(min(M, N), E), S), W);
    float Max = max(max(max(max(M, N), E), S), W);

    float Contrast = Max - Min;

    return luma(N, S, E, W, M, Min, Max, Contrast, NE, NW, SE, SW);
}

void main()
{
    FragCr = texture(OutputTextureUniform, UV);
    /*luma L = SampleLuma(UV);
    float Blend = ComputeBlendFactor(L);
    edge E = ComputeEdge(L);

    vec2 UV2 = UV;
    if (E.Horz) {
        UV2.y += E.StepSize * Blend;
    }
    else {
        UV2.x += E.StepSize * Blend;
    }
    

    FragCr = vec4(textureLod(OutputTextureUniform, UV2, 0).rgb, L.M);
    //FragCr = texture(OutputTextureUniform, UV);*/
}

)GLSL";
// }}}

// {{{ RaycasterComputeKernel
extern const char* const RaycasterComputeKernel = R"GLSL(
#version 450 core

// TODO(Liam): Theoretical f32 ops we can do in 16ms on a 620M: 3840000000 
#define SVO_NODE_OCCUPIED_MASK  0x0000FF00U
#define SVO_NODE_LEAF_MASK      0x000000FFU
#define SVO_NODE_CHILD_PTR_MASK 0x7FFF0000U
#define SVO_FAR_PTR_BIT_MASK    0x80000000U

#define MAX_STEPS 256
#define SCREEN_DIM 512
#define EMPTY_KEY 0xFFFFFFFF

#define ASSERT(Expr) if (! (Expr)) { return vec3(1); }

struct ray
{
    vec3 Origin;
    vec3 Dir;
    vec3 InvDir;
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

struct hmap_entry
{
    uint Key;
    uint PackedNormal;
    uint PackedColour;
};

layout (local_size_x = 8, local_size_y = 8) in;

layout (rgba8, binding = 0) writeonly restrict uniform image2D OutputImgUniform;
layout (r32f, binding = 1) restrict uniform image2D BeamImgUniform;

uniform uint MaxDepthUniform;
uniform uint BlockCountUniform;
uniform uint ScaleExponentUniform;
uniform uint EntriesPerBlockUniform;
uniform uint FarPtrsPerBlockUniform;

uniform uint BiasUniform;
uniform uint TableSizeUniform;
uniform float InvBiasUniform;

uniform vec3 ViewPosUniform;
uniform mat3 ViewMatrixUniform;
uniform bool IsCoarsePassUniform;


const uvec3 OCT_BITS = uvec3(1, 2, 4);
const vec3 OCT_BITS_F32 = vec3(1.0, 2.0, 4.0);

layout (std430, binding = 3) readonly restrict buffer svo_input
{
    uint Nodes[];
} SvoInputBuffer;

layout(std430, binding=5) readonly restrict buffer leaf_input
{
    hmap_entry Entries[];
} LeafInputBuffer;

layout (std430, binding = 4) readonly restrict buffer svo_far_ptr_index
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

uint GetNodeChild(in uint ParentNode, in uint Oct, inout uint ParentBlkIndex)
{
    uint ChildPtr = bitfieldExtract(ParentNode, 16, 15);
    uint OccBits = bitfieldExtract(ParentNode, 8, 8);
    uint LeafBits = bitfieldExtract(ParentNode, 0, 8);
    uint OccupiedNonLeafOcts = OccBits & (~LeafBits);
    uint SetBitsBehindOctIdx = (1 << Oct) - 1;

    uint ChildOffset = bitCount(OccupiedNonLeafOcts & SetBitsBehindOctIdx); 

    if ((ParentNode & SVO_FAR_PTR_BIT_MASK) == 0)
    {
        uint ChildIndex = ParentBlkIndex*EntriesPerBlockUniform + ChildPtr + ChildOffset;
        ParentBlkIndex += (ChildPtr + ChildOffset) >> 14;

        return SvoInputBuffer.Nodes[ChildIndex];
    }
    else
    {
        // Find the far ptr associated with this node. To do this, we need to compute
        // the byte offset for this block, then index into that block's far ptr
        // list for this node.
        uint FarPtrBlkStart = ParentBlkIndex*FarPtrsPerBlockUniform;
        far_ptr FarPtr = SvoFarPtrBuffer.FarPtrs[FarPtrBlkStart + ChildPtr];

        // Skip to the block containing the first child
        ParentBlkIndex = FarPtr.BlkIndex;
        uint ChildBlkStart = ParentBlkIndex * EntriesPerBlockUniform;

        // Skip any blocks required to get to the actual child node
        ParentBlkIndex += (FarPtr.NodeOffset + ChildOffset) >> 14;

        uint ChildIndex = ChildBlkStart + FarPtr.NodeOffset + ChildOffset;

        return SvoInputBuffer.Nodes[ChildIndex];
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
    G *= OCT_BITS;
    return G.x + G.y + G.z;
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


uint HashVec3(uvec3 V)
{
    V *= uvec3(73856093, 19349663, 83492791);
    return V.x ^ V.y ^ V.z;
}


uvec4 ComputeHashes(uint Key)
{
    const uvec4 S0 = uvec4(16, 18, 17, 16);
    const uvec4 S1 = uvec4(15, 16, 15, 16);
    const uvec4 S2 = uvec4(16, 17, 16, 15);

    const uvec4 C0 = uvec4(0x7feb352d, 0xa136aaad, 0x24f4d2cd, 0xe2d0d4cb);
    const uvec4 C1 = uvec4(0x846ca68b, 0x9f6d62d7, 0x1ba3b969, 0x3c6ad939);

    uvec4 K = uvec4(Key);
    K ^= K >> S0;
    K *= C0;
    K ^= K >> S1;
    K *= C1;
    K ^= K >> S2;
    return K % (TableSizeUniform - 1);
}

void LookupLeafVoxelData(uvec3 Pos, out vec3 NormalOut, out vec3 ColourOut)
{
    uint Key = HashVec3(Pos);
    uvec4 Hashes = ComputeHashes(Key);

    if (LeafInputBuffer.Entries[Hashes.x].Key == Key)
    {
        NormalOut = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.x].PackedNormal).xyz;
        ColourOut = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.x].PackedColour).xyz;
    }
    else if (LeafInputBuffer.Entries[Hashes.y].Key == Key)
    {
        NormalOut = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.y].PackedNormal).xyz;
        ColourOut = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.y].PackedColour).xyz;
    }
    else if (LeafInputBuffer.Entries[Hashes.z].Key == Key)
    {
        NormalOut = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.z].PackedNormal).xyz;
        ColourOut = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.z].PackedColour).xyz;
    }
    else if (LeafInputBuffer.Entries[Hashes.w].Key == Key)
    {
        NormalOut = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.w].PackedNormal).xyz;
        ColourOut = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.w].PackedColour).xyz;
    }
}

float RaycastDst(in ray R)
{
    // Scale up by the tree bias.
    uint Scale = (1 << ScaleExponentUniform) << BiasUniform;

    const float BiasScale = (1.0 / InvBiasUniform);

    uint BlkIndex = 0;
    
    vec3 RootMin = vec3(0);
    vec3 RootMax = vec3(Scale) * InvBiasUniform;
    vec3 RaySgn = sign(R.Dir);

    // Intersection of the ray with the root cube (i.e. entire tree)
    ray_intersection RaySpan = ComputeRayBoxIntersection(R, RootMin, RootMax);

    uint CurrentOct;
    uint CurrentDepth;

    // Check if the ray is within the octree at all
    if (RaySpan.tMin <= RaySpan.tMax && RaySpan.tMax > 0)
    {
        // Ray enters octree --- begin processing

        vec3 RayP = (RaySpan.tMin >= 0.0) ? R.Origin + (RaySpan.tMin * R.Dir) : R.Origin;
        vec3 ParentCentre = vec3(Scale >> 1);

        // Current octant the ray is in (confirmed good)
        CurrentOct = GetOctant(RayP, ParentCentre*InvBiasUniform);

        uint ParentNode = SvoInputBuffer.Nodes[0];
        
        // Initialise depth to 1
        CurrentDepth = 1;

        // Stack of previous voxels
        st_frame Stack[MAX_STEPS + 1];

        // Drop down one scale value to the initial children of the
        // root node.
        Scale >>= 1;


        Stack[CurrentDepth] = st_frame(ParentNode, 
                                       ParentCentre,
                                       BlkIndex);

        // Begin stepping along the ray
        for (int Step = 0; Step < MAX_STEPS; ++Step)
        {
            // Radius of the current octant's cube (half the current scale);
            vec3 Rad = vec3(Scale >> 1);

            // Get the centre position of this octant
            vec3 OctSgn = mix(vec3(-1), vec3(1), bvec3(CurrentOct & OCT_BITS));
            vec3 NodeCentre = ParentCentre + (OctSgn * Rad);

            vec3 NodeMin = (NodeCentre - Rad) * InvBiasUniform;
            vec3 NodeMax = (NodeCentre + Rad) * InvBiasUniform;

            RaySpan = ComputeRayBoxIntersection(R, NodeMin, NodeMax);

            if (RaySpan.tMin <= RaySpan.tMax && RaySpan.tMax > 0)
            {
                // Ray hit this voxel
                
                // Check if voxel occupied
                if (IsOctantOccupied(ParentNode, CurrentOct))
                {
                    // Octant is occupied, check if leaf
                    if (IsOctantLeaf(ParentNode, CurrentOct))
                    {
                        return RaySpan.tMin;
                    }
                    else
                    {
                        // Voxel has children --- execute push
                        // NOTE(Liam): BlkIndex (potentially) updated here

                        Stack[CurrentDepth] = st_frame(ParentNode,
                                                   ParentCentre,
                                                   BlkIndex);

                        ParentNode = GetNodeChild(ParentNode, CurrentOct, BlkIndex /*out*/);
                        CurrentOct = GetOctant(RayP, NodeCentre*InvBiasUniform);
                        ParentCentre = NodeCentre;
                        Scale >>= 1;
                        ++CurrentDepth;

                        continue;
                    }
                }

                // Octant not occupied, need to handle advance/pop
                uint NextOct = GetNextOctant(RaySpan.tMax, RaySpan.tMaxV, CurrentOct);
                RayP = R.Origin + (RaySpan.tMax + 0.001765625) * R.Dir;

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

                    if (NextDepth < CurrentDepth)
                    {
                        CurrentDepth = NextDepth;
                        Scale = 1 << M;;
                        ParentCentre = Stack[CurrentDepth].ParentCentre;
                        ParentNode = Stack[CurrentDepth].Node;
                        BlkIndex = Stack[CurrentDepth].BlkIndex;

                        CurrentOct = GetOctant(RayP, ParentCentre*InvBiasUniform);
                    }
                    else
                    {
                        return RaySpan.tMin;
                    }
                }
            }
            else
            {
                return RaySpan.tMin;
            }
        }
    }

    return RaySpan.tMin;
}

vec3 Raycast(in ivec2 BeamCoords, in ray R)
{
    // Scale up by the tree bias.
    uint Scale = (1U << ScaleExponentUniform) << BiasUniform;

    const float BiasScale = (1.0 / InvBiasUniform);

    uint BlkIndex = 0;
    
    vec3 RootMin = vec3(0);
    vec3 RootMax = vec3(Scale) * InvBiasUniform;
    vec3 RaySgn = sign(R.Dir);

    // Intersection of the ray with the root cube (i.e. entire tree)
    ray_intersection RaySpan = ComputeRayBoxIntersection(R, RootMin, RootMax);

    uint CurrentOct;
    uint CurrentDepth;

    // Check if the ray is within the octree at all
    if (RaySpan.tMin <= RaySpan.tMax && RaySpan.tMax > 0)
    {
        // Ray enters octree --- begin processing

        vec3 RayP = (RaySpan.tMin >= 0.0) ? R.Origin + (RaySpan.tMin * R.Dir) : R.Origin;
        const ivec2 Offsets[4] = { ivec2(0, 0), ivec2(1, 0), ivec2(0, 1), ivec2(1, 1) };
        float MinDst = min(min(
                imageLoad(BeamImgUniform, BeamCoords + Offsets[0]).x,
                imageLoad(BeamImgUniform, BeamCoords + Offsets[1]).x),
                min(imageLoad(BeamImgUniform, BeamCoords + Offsets[2]).x,
                imageLoad(BeamImgUniform, BeamCoords + Offsets[3]).x)
                ) - 0.1;
        RayP = R.Origin + MinDst*R.Dir;
        vec3 ParentCentre = vec3(Scale >> 1);

        // Current octant the ray is in (confirmed good)
        CurrentOct = GetOctant(RayP, ParentCentre*InvBiasUniform);

        uint ParentNode = SvoInputBuffer.Nodes[0];
        
        // Initialise depth to 1
        CurrentDepth = 1;

        // Stack of previous voxels
        st_frame Stack[MAX_STEPS + 1];

        // Drop down one scale value to the initial children of the
        // root node.
        Scale >>= 1;


        Stack[CurrentDepth] = st_frame(ParentNode, 
                                       //Scale,
                                       ParentCentre,
                                       BlkIndex);

        int Step;
        // Begin stepping along the ray
        for (Step = 0; Step < MAX_STEPS; ++Step)
        {
            // Radius of the current octant's cube (half the current scale);
            //vec3 Rad = vec3(Scale >> 1);
            float Rad = float(Scale >> 1);

            // Get the centre position of this octant
            vec3 OctSgn = mix(vec3(-1), vec3(1), bvec3(CurrentOct & OCT_BITS));
            vec3 NodeCentre = ParentCentre + (OctSgn * Rad);

            vec3 NodeMin = (NodeCentre - Rad) * InvBiasUniform;
            vec3 NodeMax = (NodeCentre + Rad) * InvBiasUniform;

            RaySpan = ComputeRayBoxIntersection(R, NodeMin, NodeMax);

            if (RaySpan.tMin <= RaySpan.tMax && RaySpan.tMax > 0)
            {
                // Ray hit this voxel
                
                // Check if voxel occupied
                if (IsOctantOccupied(ParentNode, CurrentOct))
                {
                    // Octant is occupied, check if leaf
                    if (IsOctantLeaf(ParentNode, CurrentOct))
                    {
                        vec3 Ldir = normalize((NodeCentre*InvBiasUniform) - vec3(ViewPosUniform));

                        vec3 N, C;
                        LookupLeafVoxelData(uvec3(NodeCentre), N, C);

                        float A =  (float(Step) / MAX_STEPS);
                        return mix(max(vec3(dot(Ldir, N)), vec3(0.25))*C, vec3(1, 0,1), A);

                    }
                    else
                    {
                        // Voxel has children --- execute push
                        // NOTE(Liam): BlkIndex (potentially) updated here

                        Stack[CurrentDepth] = st_frame(ParentNode,
                                                       ParentCentre,
                                                       BlkIndex);

                        ParentNode = GetNodeChild(ParentNode, CurrentOct, BlkIndex /*out*/);
                        CurrentOct = GetOctant(RayP, NodeCentre*InvBiasUniform);
                        ParentCentre = NodeCentre;
                        Scale >>= 1;
                        ++CurrentDepth;

                        continue;
                    }
                }

                // Octant not occupied, need to handle advance/pop
                uint NextOct = GetNextOctant(RaySpan.tMax, RaySpan.tMaxV, CurrentOct);
                RayP = R.Origin + (RaySpan.tMax + 0.001765625) * R.Dir;

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

                    if (NextDepth < CurrentDepth)
                    {
                        CurrentDepth = NextDepth;
                        Scale = 1 << M;
                        ParentCentre = Stack[CurrentDepth].ParentCentre;
                        ParentNode = Stack[CurrentDepth].Node;
                        BlkIndex = Stack[CurrentDepth].BlkIndex;

                        CurrentOct = GetOctant(RayP, ParentCentre*InvBiasUniform);
                    }
                    else
                    {
                        return (float(Step) / MAX_STEPS) * vec3(1, 0, 1);
                    }
                }
            }
            else
            {
                return (float(Step) / MAX_STEPS) * vec3(1, 0, 1);
            }
        }
    }

    return vec3(0.16);
}


#line 730
void main()
{
    // Ray XY coordinates of the screen pixels; goes from 0-512
    // in each dimension.
    ivec2 PixelCoords = ivec2(gl_GlobalInvocationID.xy);
    bool IsBeam = all(equal(PixelCoords & 0x07, ivec2(0))) && IsCoarsePassUniform;
    vec3 K = vec3(256, 256, 512);
    vec3 ScreenOrigin = ViewPosUniform - K;

    vec3 ScreenCoord = ScreenOrigin + vec3(PixelCoords, 0);
    ScreenCoord.x *= 1280.0/720.0;

    vec3 RayP = ViewPosUniform;
    vec3 RayD = normalize(ScreenCoord - ViewPosUniform);

    RayD = RayD * ViewMatrixUniform;

    ray R = { RayP, RayD, 1.0 / RayD };

    if (! IsBeam)
    {
        vec3 OutCr = Raycast(PixelCoords >> 3, R);
        imageStore(OutputImgUniform, PixelCoords, vec4(OutCr, 1.0));
    }
    else
    {
        float MinDst = RaycastDst(R); 
        imageStore(BeamImgUniform, PixelCoords >> 3, vec4(MinDst, 0, 0, 0));
    }
}
)GLSL";
// }}}

// {{{ HasherComputeKernel
extern const char* const HasherComputeKernel = R"GLSL(
#version 450 core

layout (local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

#define EMPTY_KEY 0xFFFFFFFF
#define MAX_STEPS 1024

struct entry
{
    uint Key;
    uint PackedNormal;
    uint PackedColour;
};

layout (std430, binding = 0) coherent restrict buffer htable_out
{
    entry Entries[];
} HTableOutBuffer;

layout (std430, binding = 1) readonly restrict buffer data_in
{
    entry Entries[];
} DataInputBuffer;

layout (location = 2) uniform uint TableSizeUniform;
layout (location = 3) uniform uint OffsetUniform;

uvec4 ComputeHashes(uint Key)
{
    const uvec4 S0 = uvec4(16, 18, 17, 16);
    const uvec4 S1 = uvec4(15, 16, 15, 16);
    const uvec4 S2 = uvec4(16, 17, 16, 15);

    const uvec4 C0 = uvec4(0x7feb352d, 0xa136aaad, 0x24f4d2cd, 0xe2d0d4cb);
    const uvec4 C1 = uvec4(0x846ca68b, 0x9f6d62d7, 0x1ba3b969, 0x3c6ad939);

    uvec4 K = uvec4(Key);
    K ^= K >> S0;
    K *= C0;
    K ^= K >> S1;
    K *= C1;
    K ^= K >> S2;
    return K % (TableSizeUniform - 1);
}

void main()
{
    uint ThreadID = gl_GlobalInvocationID.x;
    entry InputPair = DataInputBuffer.Entries[OffsetUniform + ThreadID];

    uint Slot0 = ComputeHashes(InputPair.Key).x;

    int Step;
    for (Step = 0; Step < MAX_STEPS; ++Step)
    {
        InputPair.Key = atomicExchange(HTableOutBuffer.Entries[Slot0].Key, InputPair.Key); 
        InputPair.PackedNormal = atomicExchange(HTableOutBuffer.Entries[Slot0].PackedNormal, InputPair.PackedNormal); 
        InputPair.PackedColour = atomicExchange(HTableOutBuffer.Entries[Slot0].PackedColour, InputPair.PackedColour); 
        if (EMPTY_KEY != InputPair.Key)
        {
            uvec4 Hashes = ComputeHashes(InputPair.Key);

            bvec4 Msk = equal(Hashes, uvec4(Slot0));
            Slot0 = uint(dot(mix(uvec4(0), Hashes.yzwx, Msk), uvec4(1)));
        }
        else
        {
            break;
        }
    }
}

)GLSL";
// }}}

