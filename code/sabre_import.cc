#include <stdio.h>
#include "sabre.h"
#include "sabre_svo.h"

#define CGLTF_IMPLEMENTATION
#include <cgltf.h>
#include <vector>
#include <assert.h>
#include <math.h>
#include <unordered_set>
#include <stack>
#include <deque>
#include <xmmintrin.h>
#include <immintrin.h>
#include <smmintrin.h>

typedef __m128 m128;
typedef __m256 m256;
typedef u32 morton_index;

struct tri3
{
    vec3 V0;
    vec3 V1;
    vec3 V2;
};

static inline u32
Part1By2(u32 X)
{
    X &= 0X000003ff;                  // X = ---- ---- ---- ---- ---- --98 7654 3210
    X = (X ^ (X << 16)) & 0Xff0000ff; // X = ---- --98 ---- ---- ---- ---- 7654 3210
    X = (X ^ (X <<  8)) & 0X0300f00f; // X = ---- --98 ---- ---- 7654 ---- ---- 3210
    X = (X ^ (X <<  4)) & 0X030c30c3; // X = ---- --98 ---- 76-- --54 ---- 32-- --10
    X = (X ^ (X <<  2)) & 0X09249249; // X = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
    return X;
}

static inline u32
EncodeMorton3(u32 X, u32 Y, u32 Z)
{
    return (Part1By2(Z) << 2) + (Part1By2(Y) << 1) + Part1By2(X);
}

struct u32_hash
{
public:
    size_t operator()(const u32& Element) const{
        return (size_t) Element;
    }
};

struct tri_buffer
{
    u32  TriangleCount;
    tri3 Triangles[1];
};


struct pos_attrib
{
    float V[3];
};

static inline bool
TriangleAABBIntersection(m128 Centre, m128 Radius, m128 Tri[3])
{
    //                THE BEAST AWAITS

    static const m128 F32SgnMsk = _mm_set1_ps(-0.0f);
    static const m128 Zero4 = _mm_set1_ps(0.0f);

    // Stackoverflow: https://stackoverflow.com/a/20084034/3121161
    const m128 NRadius = _mm_xor_ps(Radius, F32SgnMsk);

    // Transformed triangle vertices
    m128 V0 = _mm_sub_ps(Tri[0], Centre);
    m128 V1 = _mm_sub_ps(Tri[1], Centre);
    m128 V2 = _mm_sub_ps(Tri[2], Centre);

    // Triangle edge vectors
    m128 E0 = _mm_sub_ps(V1, V0);
    m128 E1 = _mm_sub_ps(V2, V1);
    m128 E2 = _mm_sub_ps(V0, V2);
    
    
    // Test the bounding box of the triangle against the box
    m128 TriMin = _mm_min_ps(_mm_min_ps(V0, V1), V2);
    int MinMask = _mm_movemask_ps(_mm_cmpgt_ps(TriMin, Radius));
    if (0x0 != (0x7 & MinMask)) return false;

    m128 TriMax = _mm_max_ps(_mm_max_ps(V0, V1), V2);
    int MaxMask = _mm_movemask_ps(_mm_cmplt_ps(TriMax, NRadius));
    if (0x0 != (0x7 & MaxMask)) return false;

    // Msk: 3 2 1 0
    //      W Z Y X
    {
        // Check if triangle and box overlap on the triangle's normal axis.
        m128 E1_YZXW = _mm_shuffle_ps(E1, E1, _MM_SHUFFLE(3, 0, 2, 1));//_MM_SHUFFLE(1, 2, 0, 3));
        m128 E0_YZXW = _mm_shuffle_ps(E0, E0, _MM_SHUFFLE(3, 0, 2, 1));//_MM_SHUFFLE(1, 2, 0, 3));
        m128 XP_ZXYW = _mm_sub_ps(_mm_mul_ps(E1_YZXW, E0), _mm_mul_ps(E0_YZXW, E1));

        // Reshuffle to get the correct cross product
        m128 TNormal = _mm_shuffle_ps(XP_ZXYW, XP_ZXYW, _MM_SHUFFLE(3, 0, 2, 1));//_MM_SHUFFLE(1, 2, 0, 3)); 

        m128 Rv = _mm_sub_ps(Radius, V0);
        m128 NRv = _mm_sub_ps(NRadius, V0);

        // Positive (>0) mask
        m128 Pmsk = _mm_cmpgt_ps(TNormal, _mm_set1_ps(0.0));
        //m128 Xmsk = _mm_xor_ps(NRv, Rv);
        // Select parts of Rv according to TNormal > 0, otherwise use NRV
        //m128 VMax = _mm_xor_ps(_mm_and_ps(Pmsk, Xmsk), Rv); // Gregory, pp. 347 - 348
        // Select parts of NRv according to TNormal < 0, otherwise use Rv
        //m128 VMin = _mm_xor_ps(_mm_and_ps(Pmsk, Xmsk), NRv);
        m128 VMax = _mm_blendv_ps(NRv, Rv, Pmsk);
        m128 VMin = _mm_blendv_ps(Rv, NRv, Pmsk);

        // Dot product between triangle normal and the min vertex
        // https://stackoverflow.com/a/35270026/3121161
        m128 Pdt = _mm_mul_ps(TNormal, VMin);
        m128 Pdt_s = _mm_movehdup_ps(Pdt);
        m128 Hsum = _mm_add_ps(Pdt, Pdt_s);
        Pdt_s = _mm_movehl_ps(Pdt_s, Hsum);
        Hsum = _mm_add_ps(Hsum, Pdt_s); // Dot in elements 0 and 2

        int Dmsk = _mm_movemask_ps(_mm_cmpgt_ps(Hsum, Zero4));
        if (0x0 != (0x1 & Dmsk)) return false; // If d.p. > 0 then return false

        Pdt = _mm_mul_ps(TNormal, VMax);
        Pdt_s = _mm_movehdup_ps(Pdt);
        Hsum = _mm_add_ps(Pdt, Pdt_s);
        Pdt_s = _mm_movehl_ps(Pdt_s, Hsum);
        Hsum = _mm_add_ps(Hsum, Pdt_s);

        Dmsk = _mm_movemask_ps(_mm_cmplt_ps(Hsum, Zero4));
        if (0x0 != (0x1 & Dmsk)) return false;

    }
    
    // Compute absolute value of triangle edges
    m128 F0 = _mm_andnot_ps(F32SgnMsk, E0);
    m128 F1 = _mm_andnot_ps(F32SgnMsk, E1);
    m128 F2 = _mm_andnot_ps(F32SgnMsk, E2);

    // p0 = e0z.v0y - e0y.v0z
    // p2 = e0z.v2y - e0y.v2z
    // min = min(p0, p2), max = max(p0, p2)
    // rad = F0z*Hy + F0y*Hz
    // if min > rad || max < -rad return 0
    //
    // rad0 = F0z*Hy + F0y*Hz
    // rad1 = F0z*Hx + F0x*Hz
    // rad2 = F0y*Hx + F0x*Hy
    //
    m128 F0_ZZYW = _mm_shuffle_ps(F0, F0, _MM_SHUFFLE(3, 1, 2, 2));//_MM_SHUFFLE(2, 2, 1, 3));
    m128 F0_YXXW = _mm_shuffle_ps(F0, F0, _MM_SHUFFLE(3, 0, 0, 1));//_MM_SHUFFLE(1, 0, 0, 3));
    m128 H_YXXW = _mm_shuffle_ps(Radius, Radius, _MM_SHUFFLE(3, 0, 0, 1));//_MM_SHUFFLE(1, 0, 0, 3));
    m128 H_ZZYW = _mm_shuffle_ps(Radius, Radius, _MM_SHUFFLE(3, 1, 2, 2));//_MM_SHUFFLE(2, 2, 1, 3));
    m128 R_123 = _mm_add_ps(_mm_mul_ps(F0_ZZYW, H_YXXW), _mm_mul_ps(F0_YXXW, H_ZZYW));
    m128 NR_123 = _mm_xor_ps(R_123, F32SgnMsk);
    // p2_1 = e0z.v2y - e0y.v2z
    // p2_2 = e0x.v2z - e0z.v2x
    // p2_3 = e0y.v2x - e0x.v2y

    // p0 = e0z*v0y - e0y*v0z
	// p0 = e0x*v0z - e0z*v0x
	// p0 = e0y*v1x - e0x*v1y

    m128 E0_ZXYW = _mm_shuffle_ps(E0, E0, _MM_SHUFFLE(3, 1, 0, 2));//_MM_SHUFFLE(2, 0, 1, 3));
    m128 E0_YZXW = _mm_shuffle_ps(E0, E0, _MM_SHUFFLE(3, 0, 2, 1));//_MM_SHUFFLE(1, 2, 0, 3));
    m128 V0YZV1X = _mm_shuffle_ps(V0, V1, _MM_SHUFFLE(3, 4, 2, 1));//_MM_SHUFFLE(1, 2, 4, 3));
    m128 V0ZXV1Y = _mm_shuffle_ps(V0, V1, _MM_SHUFFLE(3, 5, 0, 2));//_MM_SHUFFLE(2, 0, 5, 3));

    m128 P0_123 = _mm_sub_ps(_mm_mul_ps(E0_ZXYW, V0YZV1X), _mm_mul_ps(E0_YZXW, V0ZXV1Y));

    //m128 E0_ZZYW = _mm_shuffle_ps(E0, E0, _MM_SHUFFLE(3, 1, 2, 2));//_MM_SHUFFLE(2, 2, 1, 3));
    //m128 E0_YXXW = _mm_shuffle_ps(E0, E0, _MM_SHUFFLE(3, 1, 1, 0));//_MM_SHUFFLE(0, 1, 1, 3));
    m128 V2_YZXW = _mm_shuffle_ps(V2, V2, _MM_SHUFFLE(3, 0, 2, 1));//_MM_SHUFFLE(1, 2, 0, 3));
    m128 V2_ZXYW = _mm_shuffle_ps(V2, V2, _MM_SHUFFLE(3, 1, 0, 2));//_MM_SHUFFLE(2, 0, 1, 3));


    m128 P2_123 = _mm_sub_ps(_mm_mul_ps(E0_ZXYW, V2_YZXW), _mm_mul_ps(E0_YZXW, V2_ZXYW));



    // Get min, max between 
    m128 P_Min = _mm_min_ps(P0_123, P2_123);
    m128 P_Max = _mm_max_ps(P0_123, P2_123);

    m128 Gmsk = _mm_cmpgt_ps(P_Min, R_123);
    m128 Lmsk = _mm_cmplt_ps(P_Max, NR_123);
    m128 Or = _mm_or_ps(Gmsk, Lmsk);
    int OutMsk = _mm_movemask_ps(Or);
    if (0x0 != (OutMsk & 0x7)) return false;

    // p0_1 = e1z.v0y - e1y.v0z
    // p0_2 = e1x.v0z - e1z.v0x
    // p0_3 = e1y.v0x - e1x.v0y
    //
    // p2_1 = e1z.v2y - e1y.v2z
    // p2_2 = e1x.v2z - e1z.v2x
    // p2_3 = e1y.v1x - e1x.v1y

    m128 E1_ZXYW = _mm_shuffle_ps(E1, E1, _MM_SHUFFLE(3, 1, 0, 2));//_MM_SHUFFLE(2, 0, 1, 3));
    m128 E1_YZXW = _mm_shuffle_ps(E1, E1, _MM_SHUFFLE(3, 0, 2, 1));//_MM_SHUFFLE(1, 2, 0, 3));
    m128 V0_YZXW = _mm_shuffle_ps(V0, V0, _MM_SHUFFLE(3, 0, 2, 1));//_MM_SHUFFLE(1, 2, 0, 3));
    m128 V0_ZXYW = _mm_shuffle_ps(V0, V0, _MM_SHUFFLE(3, 1, 0, 2));//_MM_SHUFFLE(2, 0, 1, 3));
    P0_123 = _mm_sub_ps(_mm_mul_ps(E1_ZXYW, V0_YZXW), _mm_mul_ps(E1_YZXW, V0_ZXYW));

    m128 V2YV2XV1X = _mm_shuffle_ps(V2, V1, _MM_SHUFFLE(3, 4, 2, 1));//_MM_SHUFFLE(1, 2, 4, 3));
    m128 V2ZV2XV1Y = _mm_shuffle_ps(V2, V1, _MM_SHUFFLE(3, 5, 0, 2));//_MM_SHUFFLE(2, 0, 5, 3));
    P2_123 = _mm_sub_ps(_mm_mul_ps(E1_ZXYW, V2YV2XV1X), _mm_mul_ps(E1_YZXW, V2ZV2XV1Y));


    // rad0 = F1z.Hy + F1y.Hz
    // rad1 = F1z.Hx + F1x.Hz
    // rad2 = F1y.Hx + F1x.Hy
    m128 F1_ZZYW = _mm_shuffle_ps(F1, F1, _MM_SHUFFLE(3, 1, 2, 2));//_MM_SHUFFLE(2, 2, 1, 3));
    m128 F1_YXXW = _mm_shuffle_ps(F1, F1, _MM_SHUFFLE(3, 0, 0, 1));//_MM_SHUFFLE(1, 0, 0, 3));

    R_123 = _mm_add_ps(_mm_mul_ps(F1_ZZYW, H_YXXW), _mm_mul_ps(F1_YXXW, H_ZZYW));
    NR_123 = _mm_xor_ps(F32SgnMsk, R_123);

    P_Min = _mm_min_ps(P0_123, P2_123);
    P_Max = _mm_max_ps(P0_123, P2_123);

    Gmsk = _mm_cmpgt_ps(P_Min, R_123);
    Lmsk = _mm_cmplt_ps(P_Max, NR_123);
    Or = _mm_or_ps(Gmsk, Lmsk);
    OutMsk = _mm_movemask_ps(Or);
    if (0x0 != (OutMsk & 0x7)) return false;

    // p0_1 = e2z.v0y - e2y.v0z
    // p0_2 = e2x.v0z - e2z.v0x
    // p0_3 = e2y.v1x - e2x.v1y
    // 
    // p2_1 = e2z.v1y - e2y.v1z
    // p2_2 = e2x.v1z - e2z.v1x 
    // p2_3 = e2y.v2x - e2x.v2y
    //
    // rad0 = F2z.Hy + F2y.Hz
    // rad1 = F2z.Hx + F2x.Hz
    // rad2 = F2y.Hx + F2x.Hy
    m128 E2_ZXYW = _mm_shuffle_ps(E2, E2, _MM_SHUFFLE(3, 1, 0, 2));//_MM_SHUFFLE(2, 0, 1, 3));
    m128 E2_YZXW = _mm_shuffle_ps(E2, E2, _MM_SHUFFLE(3, 0, 2, 1));//_MM_SHUFFLE(1, 2, 0, 3));
    P0_123 = _mm_sub_ps(_mm_mul_ps(E2_ZXYW, V0YZV1X), _mm_mul_ps(E2_YZXW, V0ZXV1Y));

    m128 V1YZV2X = _mm_shuffle_ps(V1, V2, _MM_SHUFFLE(3, 4, 2, 1));//_MM_SHUFFLE(1, 2, 4, 3));
    m128 V1ZXV2Y = _mm_shuffle_ps(V1, V2, _MM_SHUFFLE(3, 5, 0, 2));//_MM_SHUFFLE(2, 0, 5, 3));
    P2_123 = _mm_sub_ps(_mm_mul_ps(E2_ZXYW, V1YZV2X), _mm_mul_ps(E2_YZXW, V1ZXV2Y));

    P_Min = _mm_min_ps(P0_123, P2_123);
    P_Max = _mm_max_ps(P0_123, P2_123);

    m128 F2_ZZYW = _mm_shuffle_ps(F2, F2, _MM_SHUFFLE(3, 1, 2, 2));//_MM_SHUFFLE(2, 2, 1, 3));
    m128 F2_YXXW = _mm_shuffle_ps(F2, F2, _MM_SHUFFLE(3, 0, 0, 1));//_MM_SHUFFLE(1, 0, 0, 3));
    R_123 = _mm_add_ps(_mm_mul_ps(F2_ZZYW, H_YXXW), _mm_mul_ps(F2_YXXW, H_ZZYW));
    NR_123 = _mm_xor_ps(F32SgnMsk, R_123);
    
    Gmsk = _mm_cmpgt_ps(P_Min, R_123);
    Lmsk = _mm_cmplt_ps(P_Max, NR_123);

    Or = _mm_or_ps(Gmsk, Lmsk);
    OutMsk = _mm_movemask_ps(Or);
    if (0x0 != (OutMsk & 0x7)) return false;

    return true;
}


static std::vector<tri3> GlobalTriangleList;
static std::unordered_set<u32, u32_hash> GlobalTriangleIndex;


static tri_buffer*
LoadMeshTriangles(cgltf_mesh* Mesh)
{
    // Locate the primitive entry for the mesh's triangle data block.
    // We do not handle non-triangle meshes.
    for (u32 PrimIndex = 0; PrimIndex < Mesh->primitives_count; ++PrimIndex)
    {
        cgltf_primitive* Prim = &Mesh->primitives[PrimIndex];

        if (cgltf_primitive_type_triangles == Prim->type)
        {
            if (Prim->indices)
            {
                u32  IndexCount = Prim->indices->count;
                u32* IndexBuffer = (u32*) malloc(IndexCount * sizeof(u32));

                // Copy the indices into the index buffer.
                for (u32 Elem = 0; Elem < Prim->indices->count; ++Elem)
                {
                    IndexBuffer[Elem] = (u32) cgltf_accessor_read_index(Prim->indices, Elem);
                }

                for (u32 AttribIndex = 0; AttribIndex < Prim->attributes_count; ++AttribIndex)
                {
                    cgltf_attribute* Attrib = &Prim->attributes[AttribIndex];

                    if (cgltf_attribute_type_position == Attrib->type)
                    {
                        //std::vector<pos_attrib> PosList;

                        cgltf_accessor* Accessor = Attrib->data;
                        u32 PosCount = Accessor->count;
                        assert(cgltf_num_components(Accessor->type) == 3);
                        pos_attrib* PosBuffer = (pos_attrib*) malloc(sizeof(pos_attrib) * PosCount);

                        cgltf_accessor_unpack_floats(Accessor, (f32*)PosBuffer, PosCount*3);

                        u32 TriCount = IndexCount - 3;
                        tri_buffer* TriBuffer = (tri_buffer*) malloc(sizeof(tri_buffer) + (sizeof(tri3) * TriCount));
                        TriBuffer->TriangleCount = TriCount;

                        for (u32 TriIndex = 0; TriIndex < TriCount; TriIndex += 3)
                        {
                            tri3 T;

                            // Load the indices
                            u32 I0 = IndexBuffer[TriIndex];
                            u32 I1 = IndexBuffer[TriIndex + 1];
                            u32 I2 = IndexBuffer[TriIndex + 2];
                            pos_attrib P0 = PosBuffer[I0];
                            pos_attrib P1 = PosBuffer[I1];
                            pos_attrib P2 = PosBuffer[I2];

                            // FIXME(Liam): MAJOR HACK ALERT MAJOR HACK ALERT!!!
                            // I love Blender. Really, I do. But it does some incredibly
                            // stupid things sometimes, like insisting that the Z axis
                            // is inverted, despite my option selections. So, we abs the
                            // Z coordinate here for no real reason other than to work around
                            // Blender's brain damage.
                            T.V0 = vec3(P0.V[0], P0.V[1], fabsf(P0.V[2]));
                            T.V1 = vec3(P1.V[0], P1.V[1], fabsf(P1.V[2]));
                            T.V2 = vec3(P2.V[0], P2.V[1], fabsf(P2.V[2]));

                            TriBuffer->Triangles[TriIndex] = T;
                        }

                        free(PosBuffer);
                        free(IndexBuffer);

                        return TriBuffer;
                    }
                }

                // Didn't find any positions, free the index buffer and exit.
                free(IndexBuffer);

                return nullptr;
            }

        }

    }

    // Mesh does not have triangle primitives; bail out
    return nullptr;
}

static inline vec3
GetOctantCentre(u32 Oct, u32 Scale, vec3 ParentCentreP)
{
    f32 Rad = (f32)(Scale >> 1U);
    f32 X = (Oct & 1U) ? 1.0f : -1.0f;
    f32 Y = (Oct & 2U) ? 1.0f : -1.0f;
    f32 Z = (Oct & 4U) ? 1.0f : -1.0f;

    return ParentCentreP + (vec3(X, Y, Z) * Rad);
}

static inline u32
NextPowerOf2Exponent(u32 X)
{ 
    return 32U - (u32)__builtin_clz(X - 1U);
}  
   

static void
BuildTriangleIndex(u32 MaxDepth, u32 ScaleExponent, tri_buffer* Tris, std::unordered_set<u32, u32_hash>& IndexOut)
{
    struct st_ctx
    {
        u32 Oct;
        u32 Scale;
        u32 Depth;
        vec3 ParentCentre;
    };

    std::deque<st_ctx> Stack;

    for (u32 TriIndex = 0; TriIndex < Tris->TriangleCount; ++TriIndex)
    {
        tri3 T = Tris->Triangles[TriIndex];//Tris.at(TriIndex);

        // For every triangle, check if it is enclosed in a bounding box
        
        u32 Bias = 0;
        f32 InvBias = 1.0f;
        if (MaxDepth > ScaleExponent)
        {
            Bias = (MaxDepth - ScaleExponent) + 1;
            InvBias = 1.0f / (1 << Bias);
        }

        u32 RootScale = 1U << (ScaleExponent) << Bias;
        vec3 ParentCentreP = vec3(RootScale >> 1);

        st_ctx RootCtx = { };
        RootCtx.Oct = 0;
        RootCtx.Depth = 1;
        RootCtx.Scale = RootScale >> 1;
        RootCtx.ParentCentre = ParentCentreP;

        Stack.push_front(RootCtx);

        alignas(16) m128 TriVerts[3];
        TriVerts[0] = _mm_set_ps(0.0f, T.V0.Z, T.V0.Y, T.V0.X);
        TriVerts[1] = _mm_set_ps(0.0f, T.V1.Z, T.V1.Y, T.V1.X);
        TriVerts[2] = _mm_set_ps(0.0f, T.V2.Z, T.V2.Y, T.V2.X);

        u32 RootCode = EncodeMorton3(ParentCentreP.X, ParentCentreP.Y, ParentCentreP.Z);
        IndexOut.insert(RootCode);

        while (false == Stack.empty())
        {
            st_ctx CurrentCtx = Stack.front();
            Stack.pop_front();

            for (u32 Oct = 0; Oct < 8; ++Oct)
            {
                // TODO(Liam): Make GetOctantCentre *only* return uvecs, then can differentiate
                // at the type level between scaled and unscaled vectors.
                vec3 Centre = GetOctantCentre(Oct, CurrentCtx.Scale, CurrentCtx.ParentCentre);

                vec3 Radius = vec3(CurrentCtx.Scale >> 1) * InvBias;

                m128 CentreM = _mm_set_ps(0.0f, Centre.Z*InvBias, Centre.Y*InvBias, Centre.X*InvBias);
                m128 RadiusM = _mm_set_ps(0.0f, Radius.Z, Radius.Y, Radius.X);

                if (TriangleAABBIntersection(CentreM, RadiusM, TriVerts))
                {
                    IndexOut.insert(EncodeMorton3(Centre.X, Centre.Y, Centre.Z));
                    /*usize OffsetX = (u32)Centre.X / 64;
                    usize OffsetY = (u32)Centre.Y / 64;
                    usize OffsetZ = (u32)Centre.Z / 64;
                    TriangleBitmapX[OffsetX] |= (1 << (u32)Centre.X % 64);
                    TriangleBitmapY[OffsetY] |= (1 << (u32)Centre.Y % 64);
                    TriangleBitmapZ[OffsetZ] |= (1 << (u32)Centre.Z % 64);*/

                    if (CurrentCtx.Depth < MaxDepth)
                    {
                        st_ctx NewCtx = { };
                        NewCtx.Oct = Oct;
                        NewCtx.Depth = CurrentCtx.Depth + 1;
                        NewCtx.Scale = CurrentCtx.Scale >> 1;
                        NewCtx.ParentCentre = Centre;
                        Stack.push_front(NewCtx);
                    }
                }
            }
        }

        Stack.clear();
    }
}


static bool
IntersectorFunction(vec3 vMin, vec3 vMax, const svo* const Tree)
{
    vec3 Halfsize = (vMax - vMin) * 0.5f;
    uvec3 Centre = uvec3((vMin + Halfsize) * (1 << Tree->Bias));
    
    u32 MortonCode = EncodeMorton3(Centre.X, Centre.Y, Centre.Z);
    /*u32 Xmsk = (1 << (Centre.X % 64));
    u32 Ymsk = (1 << (Centre.Y % 64));
    u32 Zmsk = (1 << (Centre.Z % 64));

    u32 IndexX = Centre.X / 64;
    u32 IndexY = Centre.Y / 64;
    u32 IndexZ = Centre.Z / 64;*/

    return GlobalTriangleIndex.find(MortonCode) != GlobalTriangleIndex.end();
    //return ((TriangleBitmapX[IndexX] & Xmsk) && (TriangleBitmapY[IndexY] & Ymsk) && (TriangleBitmapZ[IndexZ] & Zmsk));
}

static inline f32
GetMeshMaxDimension(const cgltf_primitive* const Prim)
{
    cgltf_accessor* PosAttrib = nullptr;
    for (u32 AttribIndex = 0; AttribIndex < Prim->attributes_count; ++AttribIndex)
    {
        if (Prim->attributes[AttribIndex].type == cgltf_attribute_type_position)
        {
            PosAttrib = Prim->attributes[AttribIndex].data;
        }
    }

    if (nullptr == PosAttrib) return 0.0f;
    if (false == PosAttrib->has_max) return 0.0f;

    f32 MaxX = PosAttrib->max[0];
    f32 MaxY = PosAttrib->max[1];
    f32 MaxZ = PosAttrib->max[2];

    return Max(Max(MaxX, MaxY), MaxZ);
}

static inline f32
GetMeshMinDimension(const cgltf_primitive* const Prim)
{
    cgltf_accessor* PosAttrib = nullptr;
    for (u32 AttribIndex = 0; AttribIndex < Prim->attributes_count; ++AttribIndex)
    {
        if (Prim->attributes[AttribIndex].type == cgltf_attribute_type_position)
        {
            PosAttrib = Prim->attributes[AttribIndex].data;
        }
    }

    if (nullptr == PosAttrib) return 0.0f;
    if (false == PosAttrib->has_min) return 0.0f;

    f32 MinX = PosAttrib->min[0];
    f32 MinY = PosAttrib->min[1];
    f32 MinZ = PosAttrib->min[2];

    return Min(Min(MinX, MinY), MinZ);
}

extern "C" svo*
ImportGltfToSvo(u32 MaxDepth, const char* const GLTFPath)
{
    //                        W     Z     Y     X
#if 0 
    m128 Centre = _mm_set_ps(0.0f, 30.25f, 49.25, 52.25);
    m128 Radius = _mm_set_ps(0.0f, 0.25f, 0.25f, 0.25f);

    m128 Triangle[3];
    Triangle[0] = _mm_set_ps(0.0f, 30.9630013f, 49.5660324f, 52.4217682f);
    Triangle[1] = _mm_set_ps(0.0f, 30.3846645f, 49.7899361f, 52.1688766f);
    Triangle[2] = _mm_set_ps(0.0f, 30.1548805f, 49.1613388f, 52.4547081f);

    f32 Centre2[3] = { 52.25f, 49.25f, 30.25f };
    f32 Radius2[3] = { 0.25f, 0.25f, 0.25f };
    f32 Triangle2[3][3] = {
        {52.4217682f, 49.5660324f, 30.9630013f},
        {52.1688766f, 49.7899361f, 30.3846645f},
        {52.4547081f, 49.1613388f, 30.1548805f}
    };

    bool A = TriangleAABBIntersection(Centre, Radius, Triangle);
#endif

	cgltf_options Options = { };
    cgltf_data* Data = nullptr;

	cgltf_result Result = cgltf_parse_file(&Options, GLTFPath, &Data);

    Result = cgltf_load_buffers(&Options, Data, nullptr);
    Result = cgltf_validate(Data);

	if (cgltf_result_success == Result)
    {
        tri_buffer* TriangleData = LoadMeshTriangles(&Data->meshes[0]);
        if (nullptr == TriangleData)
        {
            fprintf(stderr, "Failed to load triangle data");
            return nullptr;
        }

        // Compute max scale exponent from mesh max and min vertices.
        f32 MaxDim = GetMeshMaxDimension(&Data->meshes[0].primitives[0]);
        f32 MinDim = GetMeshMinDimension(&Data->meshes[0].primitives[0]);

        assert(MaxDim > 0);
        // TODO(Liam): Check here in case max < 0
        u32 ScaleExponent = NextPowerOf2Exponent((u32)ceilf(MaxDim));
        assert(ScaleExponent > 0);

        GlobalTriangleIndex.reserve(TriangleData->TriangleCount);
        BuildTriangleIndex(MaxDepth, ScaleExponent, TriangleData, GlobalTriangleIndex);

        // TODO(Liam): Figure out max scale exponent from max/min in data
        svo* Svo = CreateSparseVoxelOctree(ScaleExponent, MaxDepth, &IntersectorFunction);

        free(TriangleData);
        cgltf_free(Data);

        return Svo;
    }
    else
    {
        // TODO(Liam): Handle failure case
        return nullptr;
    }
}
