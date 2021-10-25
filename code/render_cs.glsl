#version 450 core

// NOTE(Liam): Theoretical f32 ops we can do in 16ms on a 620M: 3840000000
#define SVO_NODE_OCCUPIED_MASK  0x0000FF00U
#define SVO_NODE_LEAF_MASK      0x000000FFU
#define SVO_NODE_CHILD_PTR_MASK 0x7FFF0000U
#define SVO_FAR_PTR_BIT_MASK    0x80000000U


#define ASPECT_RATIO (1280.0/720.0)

#define MAX_STEPS     512
#define EMPTY_KEY     0xFFFFFFFFU
#define F32_MAX       3.402823e+38f
#define STEP_EPS      0.001765625f
#define BACKGROUND_CR vec4(0.12, 0.12, 0.12, 1.0)

#define HORZ_OR3(A)  ((A.x)|(A.y)|(A.z))
#define HORZ_MIN3(A) min(min(A.x, A.y), A.z)
#define HORZ_MAX3(A) max(max(A.x, A.y), A.z)


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
    uint PackedN;
    uint PackedC;
};

layout (local_size_x = 8, local_size_y = 8) in;

layout (rgba8, binding = 0) writeonly restrict uniform image2D OutputImgUniform;
layout (r32f, binding = 1) restrict uniform image2D BeamImgUniform;
layout (rgba32f, binding = 2) restrict uniform image2D TileImgUniform;

layout (location = 0) uniform uint MaxDepthUniform;
layout (location = 1) uniform uint BlockCountUniform;
layout (location = 2) uniform uint ScaleExpUniform;
layout (location = 3) uniform uint EntriesPerBlockUniform;
layout (location = 4) uniform uint FarPtrsPerBlockUniform;

layout (location = 5) uniform uint BiasUniform;
layout (location = 6) uniform uint TableSizeUniform;
layout (location = 7) uniform float InvBiasUniform;

layout (location = 8) uniform vec3 ViewPosUniform;
layout (location = 9) uniform mat3 ViewMatrixUniform;
layout (location = 10) uniform bool IsCoarsePassUniform;

const uvec3 OCT_BITS = uvec3(1, 2, 4);
const vec3 OCT_BITS_F32 = vec3(1.0, 2.0, 4.0);

const far_ptr NULL_FAR_PTR = {0, 0};

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

layout (std430, binding = 6) restrict buffer indirect_cmd
{
    uint GroupSizeX;
    uint GroupSizeY;
    uint GroupSizeZ;
} IndirectCmdBuffer;

void GetNodeChild(inout uvec2 NodeRef, in uint Oct)
{
    uvec2 Out;
    uint ChildPtr = bitfieldExtract(NodeRef.x, 16, 15);
    uint OccBits = bitfieldExtract(NodeRef.x, 8, 8);
    uint LeafBits = bitfieldExtract(NodeRef.x, 0, 8);
    uint OccupiedNonLeafOcts = OccBits & (~LeafBits);
    uint SetBitsBehindOctIdx = (1U << Oct) - 1U;

    uint ChildOffset = bitCount(OccupiedNonLeafOcts & SetBitsBehindOctIdx);
    bool Far = bool(NodeRef.x & SVO_FAR_PTR_BIT_MASK);

    uint FarPtrIndex;
    FarPtrIndex = (Far) ? (NodeRef.y*FarPtrsPerBlockUniform) + ChildPtr : FarPtrIndex;
    far_ptr FarPtr = (Far) ? SvoFarPtrBuffer.FarPtrs[FarPtrIndex] : NULL_FAR_PTR;
    NodeRef.y = (Far) ? (FarPtr.BlkIndex) : NodeRef.y;
    uint NodeOffset = (Far) ? (FarPtr.NodeOffset) : ChildPtr;

    uint ChildIndex = (NodeRef.y*EntriesPerBlockUniform) + NodeOffset + ChildOffset;
    NodeRef.y += (NodeOffset + ChildOffset) >> 14U;
    NodeRef.x = SvoInputBuffer.Nodes[ChildIndex];
}

ray_intersection RayBoxIntersection(in ray R, in vec3 vMin, in vec3 vMax)
{
    vec3 t0 = (vMin - R.Origin) * R.InvDir; // Distance along ray to vmin planes
    vec3 t1 = (vMax - R.Origin) * R.InvDir; // Distance along ray to vmax planes

    vec3 tMin = min(t0, t1); // Minimums of all distances
    vec3 tMax = max(t0, t1); // Maximums of all distances

    float ttMin = HORZ_MAX3(tMin); // Max of min distances (closest to box)
    float ttMax = HORZ_MIN3(tMax); // Min of max distances (closest to box)

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

bool IsAdvanceValid2(in uint NewOct, in uint OldOct, in uint RaySgnMsk)
{
    if (OldOct > NewOct)
    {
        ivec2 Tmp = ivec2(NewOct, OldOct);
        NewOct = Tmp.y;
        OldOct = Tmp.x;
        RaySgnMsk ^= 7;
    }

    return ((NewOct - OldOct) & RaySgnMsk) != 0;
}


uint GetNextOctant(in float tMax, in vec3 tValues, in uint CurrentOct)
{
    // For every element of tValues that is equal to tMax, XOR the corresponding
    // bit in CurrentOct.
    // if tMax >= tValues.x then NextOct ^= 1;
    // if tMax >= tValues.y then NextOct ^= 2;
    // if tMax >= tValues.z then NextOct ^= 4;
    uvec3 XorMsk3 = ivec3(greaterThanEqual(vec3(tMax), tValues)) * OCT_BITS;
    uint XorMsk = XorMsk3.x + XorMsk3.y + XorMsk3.z;
    return CurrentOct ^ XorMsk;
}

uint GetOctant(in vec3 P, in vec3 ParentCentreP)
{
    uvec3 G = mix(uvec3(0), OCT_BITS, greaterThan(P, ParentCentreP));
    return HORZ_OR3(G);
}

vec3 Oct2Cr(in uint Oct)
{
    return vec3(bvec3(uvec3(Oct) & uvec3(1, 2, 4))).bgr;
}

struct st_frame
{
// x node, y blkindex
    uvec2 NodeRef;
    vec3 ParentCentre;
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

void LookupLeafVoxelData(uvec3 Pos, out vec3 N, out vec3 C)
{
    uint Key = HashVec3(Pos);
    uvec4 Hashes = ComputeHashes(Key);

    if (LeafInputBuffer.Entries[Hashes.x].Key == Key)
    {
        N = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.x].PackedN).xyz;
        C = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.x].PackedC).xyz;
    }
    else if (LeafInputBuffer.Entries[Hashes.y].Key == Key)
    {
        N = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.y].PackedN).xyz;
        C = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.y].PackedC).xyz;
    }
    else if (LeafInputBuffer.Entries[Hashes.z].Key == Key)
    {
        N = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.z].PackedN).xyz;
        C = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.z].PackedC).xyz;
    }
    else if (LeafInputBuffer.Entries[Hashes.w].Key == Key)
    {
        N = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.w].PackedN).xyz;
        C = unpackSnorm4x8(LeafInputBuffer.Entries[Hashes.w].PackedC).xyz;
    }
}


vec4 Raycast(in ray R, in bool IsRay, in ivec2 BeamXY)
{
    vec4 Result;
    // Scale up by the tree bias.
    uint Scale = (1 << ScaleExpUniform) << BiasUniform;
    const float BiasScale = (1.0 / InvBiasUniform);
    const uint StopDepth = MaxDepthUniform - 1;

    vec3 RootMin = vec3(0);
    vec3 RootMax = vec3(Scale) * InvBiasUniform;
    vec3 RaySgn = sign(R.Dir);
    const uint C0 = ScaleExpUniform + BiasUniform;

    // Intersection of the ray with the root cube (i.e. entire tree)
    ray_intersection RaySpan = RayBoxIntersection(R, RootMin, RootMax);

    uint CurrentOct;
    uint CurrentDepth;

    // Check if the ray is within the octree at all
    if (RaySpan.tMin <= RaySpan.tMax && RaySpan.tMax > 0)
    {
        // Ray enters octree --- begin processing

        vec3 RayP = (RaySpan.tMin >= 0.0) ? R.Origin + (RaySpan.tMin * R.Dir) : R.Origin;
        
        if (IsRay)
        {
            const ivec2 Off = ivec2(0, 1);

            float MinDst = min(min(
                    imageLoad(BeamImgUniform, BeamXY + Off.xx).x,
                    imageLoad(BeamImgUniform, BeamXY + Off.yx).x),
                    min(imageLoad(BeamImgUniform, BeamXY + Off.xy).x,
                    imageLoad(BeamImgUniform, BeamXY + Off.yy).x)
                    );

            // Clamp MinDst to the scene dimensions
            MinDst = max(MinDst, RaySpan.tMin);

            RayP = R.Origin + MinDst*R.Dir;
        }

        vec3 ParentCentre = vec3(Scale >> 1U);

        // Current octant the ray is in (confirmed good)
        CurrentOct = GetOctant(RayP, ParentCentre*InvBiasUniform);

        uvec2 ParentRef = uvec2(SvoInputBuffer.Nodes[0], 0);
        
        // Initialise depth to 1
        CurrentDepth = 1;

        // Stack of previous voxels
        st_frame Stack[17];

        // Drop down one scale value to the initial children of the
        // root node.
        Scale >>= 1U;

        Stack[CurrentDepth] = st_frame(ParentRef, ParentCentre);

        // Begin stepping along the ray
        for (int Step = 0; Step < MAX_STEPS; ++Step)
        {
            // Radius of the current octant's cube (half the current scale);
            float Rad = float(Scale >> 1U);

            // Get the centre position of this octant
            vec3 OctSgn = mix(vec3(-1), vec3(1), bvec3(CurrentOct & OCT_BITS));
            Result.xyz = ParentCentre + (OctSgn * Rad);

            vec3 NodeMin = (Result.xyz - Rad) * InvBiasUniform;
            vec3 NodeMax = (Result.xyz + Rad) * InvBiasUniform;

            RaySpan = RayBoxIntersection(R, NodeMin, NodeMax);

            if (RaySpan.tMin <= RaySpan.tMax)
            {
                // Ray hit this voxel
                uint ChildMsk = ParentRef.x << (CurrentOct ^ 7);
                // Check if voxel occupied
                if ((ChildMsk & 0x8000U) != 0U)
                {
                    // Octant is occupied, check if leaf
                    if (CurrentDepth == StopDepth)
                    {
                        Result.w = RaySpan.tMin;
                        return Result;
                    }
                    else
                    {
                        // Voxel has children --- execute push
                        // NOTE(Liam): BlkIndex (potentially) updated here

                        Stack[CurrentDepth] = st_frame(ParentRef, ParentCentre);

                        GetNodeChild(ParentRef, CurrentOct);
                        CurrentOct = GetOctant(RayP, Result.xyz*InvBiasUniform);
                        ParentCentre = Result.xyz;
                        Scale >>= 1;
                        ++CurrentDepth;

                        continue;
                    }
                }

                // Octant not occupied, need to handle advance/pop
                uint NextOct = GetNextOctant(RaySpan.tMax, RaySpan.tMaxV, CurrentOct);
                RayP = R.Origin + (RaySpan.tMax + STEP_EPS) * R.Dir;

                if (IsAdvanceValid(NextOct, CurrentOct, RaySgn))
                {
                    CurrentOct = NextOct;
                }
                else
                {
                    // Determined that NodeCentre is never < 0
                    uvec3 NodeCentreBits = uvec3(Result.xyz);
                    uvec3 RayPBits = uvec3(RayP * BiasScale);

                    // NOTE(Liam): It is **okay** to have negative values here
                    // because the HDB will end up being equal to ScaleExp.
                    //
                    // Find the highest differing bit
                    uvec3 HDB = findMSB(NodeCentreBits ^ RayPBits);

                    uint M = HORZ_MAX3(mix(uvec3(0), HDB, lessThan(HDB, uvec3(C0))));

                    uint NextDepth = (C0 - M);

                    if (NextDepth < CurrentDepth)
                    {
                        CurrentDepth = NextDepth;
                        Scale = 1U << M;
                        ParentCentre = Stack[CurrentDepth].ParentCentre;
                        ParentRef = Stack[CurrentDepth].NodeRef;
                        CurrentOct = GetOctant(RayP, ParentCentre*InvBiasUniform);
                    }
                    else
                    {
                        Result.w = F32_MAX;
                        return Result;
                    }
                }
            }
            else
            {
                Result.w = F32_MAX;
                return Result;
            }
        }
    }

    Result.w = F32_MAX;
    return Result;
}


#define COT_30DEG 1.73205080
void main()
{
    ivec2 PixelCoords = ivec2(gl_GlobalInvocationID.xy);
    if (IsCoarsePassUniform)
    {
        PixelCoords <<= 3;
    }
    vec3 ScreenCoords = vec3(512, 384, 1024);

    vec3 ScreenOrigin = ViewPosUniform - ScreenCoords;
    vec3 ScreenCoord = ScreenOrigin + vec3(PixelCoords.xy, 0);
    ScreenCoord.x *= 1024.0/768.0;

    vec3 RayP = ViewPosUniform;
    vec3 RayD = normalize(ScreenCoord - ViewPosUniform);
    RayD *= ViewMatrixUniform;

    vec3 HitP;
    ray R = { RayP, RayD, 1.0 / RayD };

    // Branch is fine here because all pixels will take the branch.
    if (IsCoarsePassUniform)
    {
        vec4 MinDst = Raycast(R, false, ivec2(0));
        imageStore(BeamImgUniform, PixelCoords >> 3U, MinDst.wzyx);
    }
    else
    {
        vec3 N = vec3(0);
        vec3 Ldir = vec3(0);
        vec3 C = vec3(0);

        vec4 Dst = Raycast(R, true, PixelCoords >> 3U);
        if (Dst.w != F32_MAX)
        {
            Ldir = normalize((Dst.xyz*InvBiasUniform) - vec3(ViewPosUniform));
            LookupLeafVoxelData(uvec3(Dst.xyz), N, C);
            vec3 OutCr = max(vec3(dot(Ldir, N)), vec3(0.25))*C;
            imageStore(OutputImgUniform, PixelCoords, vec4(OutCr, 1.0));
        }
        else
        {
            imageStore(OutputImgUniform, PixelCoords, BACKGROUND_CR);
        }
    }
}

