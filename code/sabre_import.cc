#include <stdio.h>
#include "sabre.h"
#include "sabre_svo.h"

#define CGLTF_IMPLEMENTATION
#include <cgltf.h>
#include <vector>
#include <assert.h>
#include <math.h>
#include <unordered_set>
#include <set>
#include <unordered_map>
#include <stack>
#include <deque>
#include <xmmintrin.h>
#include <smmintrin.h>
#include "vecmath.h"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

using namespace vm;

typedef __m128 m128;

struct tri3
{
    vec3 V0;
    vec3 V1;
    vec3 V2;
};

struct img
{
    u32 Width;
    u32 Height;
    u32 Channels;
    unsigned char* Pixels;
};


struct tri_data
{
    tri3 T;
    vec3 Normal;

    vec2 TexCoord[3]; // TODO Split into different table
    cgltf_material* Material;
};


struct material_data
{
    std::vector<img> Images;
    std::vector<cgltf_material*> Material;
};

struct tri_buffer
{
    u32      TriangleCount;
    tri_data Triangles[1];
};


struct pos_attrib
{
    f32 V[3];
};

struct texcoord_attrib
{
    f32 V[2];
};


static std::unordered_map<cgltf_image*, img>* GlobalTextureCache;


static constexpr u64
U64Hash(u64 X)
{
    X = (X ^ (X >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    X = (X ^ (X >> 27)) * UINT64_C(0x94d049bb133111eb);
    X = X ^ (X >> 31);
    return X;
}

struct u64_hash
{
    constexpr inline size_t
    operator()(const u64& Element) const
    {
        return static_cast<size_t>(U64Hash(Element));
    }
};


using morton_map = std::unordered_map<morton_key, vec3, u64_hash>;

static inline vec3
BarycentricCoords(vec3 V0, vec3 V1, vec3 V2, vec3 X)
{
    vec3 Barycentric;

    vec3 E0 = V1 - V0;
    vec3 E1 = V2 - V0;
    vec3 EX = X - V0;

    float D00 = Dot(E0, E0);
    float D01 = Dot(E0, E1);
    float D11 = Dot(E1, E1);
    float D20 = Dot(EX, E0);
    float D21 = Dot(EX, E1);

    float Denom = (D00 * D11) - (D01 * D01);
    float SafeRatio = (Denom == 0.0f) ? 1.0f : (1.0f / Denom);
    Barycentric.Y = (D11 * D20 - D01 * D21) * SafeRatio;
    Barycentric.Z = (D00 * D21 - D01 * D20) * SafeRatio;
    Barycentric.X = 1.0f - Barycentric.Y - Barycentric.Z;

    return Barycentric;
}


static inline void
DecodeTextureImage(const cgltf_buffer_view* const Tex, img* const ImgOut)
{
    int Width, Height, Channels;
    stbi_uc* ImgData = static_cast<stbi_uc*>(Tex->buffer->data) + Tex->offset;
    stbi_uc* Pixels = stbi_load_from_memory(ImgData,
                                            static_cast<int>(Tex->size),
                                            &Width,
                                            &Height,
                                            &Channels,
                                            0);
    assert(Pixels);

    *ImgOut = img{ u32(Width), u32(Height), u32(Channels), Pixels };
}

static inline vec3
ComputeTriangleNormal(tri3* Triangle)
{
    vec3 E0 = Triangle->V1 - Triangle->V0;
    vec3 E1 = Triangle->V2 - Triangle->V0;

    return Normalize(Cross(E0, E1));
}

static inline vec3
SampleMaterialColour(const cgltf_material* const Mat, vec2 UV)
{
    // Figure out where in the material to sample
    if (Mat->has_pbr_metallic_roughness)
    {
        // Sample the metallic texture
        const cgltf_pbr_metallic_roughness* R = &Mat->pbr_metallic_roughness;        

        vec3 BaseColourFactor{
            R->base_color_factor[0],
            R->base_color_factor[1],
            R->base_color_factor[2],
        };

        cgltf_texture* BaseColourTex = R->base_color_texture.texture;
        if (BaseColourTex && BaseColourTex->image)
        {
            img Img;
            if (GlobalTextureCache->find(BaseColourTex->image) != GlobalTextureCache->end())
            {
                Img = GlobalTextureCache->at(BaseColourTex->image);
            }             
            else
            {
                cgltf_image* TextureImage = BaseColourTex->image;

                DecodeTextureImage(TextureImage->buffer_view, &Img);
                GlobalTextureCache->emplace(TextureImage, Img);
            }

            assert(Img.Pixels);

            u32 TexelX = u32(f32(Img.Width) * UV.X);
            u32 TexelY = u32(f32(Img.Height) * UV.Y);
            assert(TexelX < Img.Width && TexelY < Img.Height);

            stbi_uc* Pixel = &Img.Pixels[Img.Channels*(TexelY * Img.Width + TexelX)];
            if (3 == Img.Channels || 4 == Img.Channels)
            {
                f32 Red = f32(Pixel[0]) / 255.0f;
                f32 Green = f32(Pixel[1]) / 255.0f;
                f32 Blue = f32(Pixel[2]) / 255.0f;

                return vec3(Red, Green, Blue) * BaseColourFactor;
            }
            else
            {
                return vec3(1, 0, 0);
            }
        }
        else
        {
            return BaseColourFactor;
        }
    }
    else
    {
        return vec3(1, 0, 0);
    }
}

static inline vec3
ComputeVoxelColour(const tri_data* const Tri, vec3 VoxelCentre)
{
    cgltf_material* Mat = Tri->Material;

    if (nullptr == Mat)
    {
        return vec3{1.0f, 0.84f, 0.0f};
    }
    else
    {
        // Get the voxel's UV coords within the triangle through
        // barycentric interpolation.
        vec2 V0{ Tri->TexCoord[0].X, Tri->TexCoord[0].Y };
        vec2 V1{ Tri->TexCoord[1].X, Tri->TexCoord[1].Y };
        vec2 V2{ Tri->TexCoord[2].X, Tri->TexCoord[2].Y };

        vec3 B = BarycentricCoords(Tri->T.V0, Tri->T.V1, Tri->T.V2, VoxelCentre);
        vec2 VoxelUV;
        VoxelUV.X = fabsf(fmod(V0.X + B.Y*(V1.X - V0.X) + B.Z*(V2.X - V0.X), 1.0f));
        VoxelUV.Y = fabsf(fmod(V0.Y + B.Y*(V1.Y - V0.Y) + B.Z*(V2.Y - V0.Y), 1.0f));

        return SampleMaterialColour(Mat, VoxelUV);
    }
}


static inline bool
TriangleAABBIntersection(m128 Centre, m128 Radius, m128 Tri[3])
{
    // Note: This code is not likely to be particularly faster than
    // a scalar version; it exists more as a learning vehicle for
    // SSE intrinsics.
    //
    // The formula used is the Askine-Moller box-triangle intersection
    // test.

    const m128 F32SgnMsk = _mm_set1_ps(-0.0f);
    const m128 Zero4 = _mm_set1_ps(0.0f);

    // Stackoverflow: https://stackoverflow.com/a/20084034/3121161
    m128 NRadius = _mm_xor_ps(Radius, F32SgnMsk);

    // Transformed triangle vertices
    m128 V0 = _mm_sub_ps(Tri[0], Centre);
    m128 V1 = _mm_sub_ps(Tri[1], Centre);
    m128 V2 = _mm_sub_ps(Tri[2], Centre);

    // Triangle edge vectors
    m128 E0 = _mm_sub_ps(V1, V0);
    m128 E1 = _mm_sub_ps(V2, V1);
    m128 E2 = _mm_sub_ps(V0, V2);
    
    // First test: does the bounding box of the triangle fit inside
    // the supplied min/max dimensions?
    m128 TriMin = _mm_min_ps(_mm_min_ps(V0, V1), V2);
    int MinMask = _mm_movemask_ps(_mm_cmpgt_ps(TriMin, Radius));
    if (0x7 & MinMask) return false;

    m128 TriMax = _mm_max_ps(_mm_max_ps(V0, V1), V2);
    int MaxMask = _mm_movemask_ps(_mm_cmplt_ps(TriMax, NRadius));
    if (0x7 & MaxMask) return false;

    // Msk: 3 2 1 0
    //      W Z Y X
    {
        // Check if triangle and box overlap on the triangle's normal axis.
        m128 E1_YZXW = _mm_shuffle_ps(E1, E1, _MM_SHUFFLE(3, 0, 2, 1));
        m128 E0_YZXW = _mm_shuffle_ps(E0, E0, _MM_SHUFFLE(3, 0, 2, 1));
        m128 XP_ZXYW = _mm_sub_ps(_mm_mul_ps(E1_YZXW, E0), _mm_mul_ps(E0_YZXW, E1));

        // Reshuffle to get the correct cross product
        m128 TNormal = _mm_shuffle_ps(XP_ZXYW, XP_ZXYW, _MM_SHUFFLE(3, 0, 2, 1));

        m128 Rv = _mm_sub_ps(Radius, V0);
        m128 NRv = _mm_sub_ps(NRadius, V0);

        // Positive (>0) mask
        m128 Pmsk = _mm_cmpgt_ps(TNormal, _mm_set1_ps(0.0));
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
        if (0x1 & Dmsk) return false; // If d.p. > 0 then return false

        Pdt = _mm_mul_ps(TNormal, VMax);
        Pdt_s = _mm_movehdup_ps(Pdt);
        Hsum = _mm_add_ps(Pdt, Pdt_s);
        Pdt_s = _mm_movehl_ps(Pdt_s, Hsum);
        Hsum = _mm_add_ps(Hsum, Pdt_s);

        Dmsk = _mm_movemask_ps(_mm_cmplt_ps(Hsum, Zero4));
        if (0x1 & Dmsk) return false;

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
    m128 F0_ZZYW = _mm_shuffle_ps(F0, F0, _MM_SHUFFLE(3, 1, 2, 2));
    m128 F0_YXXW = _mm_shuffle_ps(F0, F0, _MM_SHUFFLE(3, 0, 0, 1));
    m128 H_YXXW = _mm_shuffle_ps(Radius, Radius, _MM_SHUFFLE(3, 0, 0, 1));
    m128 H_ZZYW = _mm_shuffle_ps(Radius, Radius, _MM_SHUFFLE(3, 1, 2, 2));
    m128 R_123 = _mm_add_ps(_mm_mul_ps(F0_ZZYW, H_YXXW), _mm_mul_ps(F0_YXXW, H_ZZYW));
    m128 NR_123 = _mm_xor_ps(R_123, F32SgnMsk);
    // p2_1 = e0z.v2y - e0y.v2z
    // p2_2 = e0x.v2z - e0z.v2x
    // p2_3 = e0y.v2x - e0x.v2y

    // p0 = e0z*v0y - e0y*v0z
	// p0 = e0x*v0z - e0z*v0x
	// p0 = e0y*v1x - e0x*v1y

    m128 E0_ZXYW = _mm_shuffle_ps(E0, E0, _MM_SHUFFLE(3, 1, 0, 2));
    m128 E0_YZXW = _mm_shuffle_ps(E0, E0, _MM_SHUFFLE(3, 0, 2, 1));
    m128 V0YZV1X = _mm_shuffle_ps(V0, V1, _MM_SHUFFLE(3, 4, 2, 1));
    m128 V0ZXV1Y = _mm_shuffle_ps(V0, V1, _MM_SHUFFLE(3, 5, 0, 2));

    m128 P0_123 = _mm_sub_ps(_mm_mul_ps(E0_ZXYW, V0YZV1X), _mm_mul_ps(E0_YZXW, V0ZXV1Y));

    m128 V2_YZXW = _mm_shuffle_ps(V2, V2, _MM_SHUFFLE(3, 0, 2, 1));
    m128 V2_ZXYW = _mm_shuffle_ps(V2, V2, _MM_SHUFFLE(3, 1, 0, 2));


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

    m128 E1_ZXYW = _mm_shuffle_ps(E1, E1, _MM_SHUFFLE(3, 1, 0, 2));
    m128 E1_YZXW = _mm_shuffle_ps(E1, E1, _MM_SHUFFLE(3, 0, 2, 1));
    m128 V0_YZXW = _mm_shuffle_ps(V0, V0, _MM_SHUFFLE(3, 0, 2, 1));
    m128 V0_ZXYW = _mm_shuffle_ps(V0, V0, _MM_SHUFFLE(3, 1, 0, 2));
    P0_123 = _mm_sub_ps(_mm_mul_ps(E1_ZXYW, V0_YZXW), _mm_mul_ps(E1_YZXW, V0_ZXYW));

    m128 V2YV2XV1X = _mm_shuffle_ps(V2, V1, _MM_SHUFFLE(3, 4, 2, 1));
    m128 V2ZV2XV1Y = _mm_shuffle_ps(V2, V1, _MM_SHUFFLE(3, 5, 0, 2));
    P2_123 = _mm_sub_ps(_mm_mul_ps(E1_ZXYW, V2YV2XV1X), _mm_mul_ps(E1_YZXW, V2ZV2XV1Y));


    // rad0 = F1z.Hy + F1y.Hz
    // rad1 = F1z.Hx + F1x.Hz
    // rad2 = F1y.Hx + F1x.Hy
    m128 F1_ZZYW = _mm_shuffle_ps(F1, F1, _MM_SHUFFLE(3, 1, 2, 2));
    m128 F1_YXXW = _mm_shuffle_ps(F1, F1, _MM_SHUFFLE(3, 0, 0, 1));

    R_123 = _mm_add_ps(_mm_mul_ps(F1_ZZYW, H_YXXW), _mm_mul_ps(F1_YXXW, H_ZZYW));
    NR_123 = _mm_xor_ps(F32SgnMsk, R_123);

    P_Min = _mm_min_ps(P0_123, P2_123);
    P_Max = _mm_max_ps(P0_123, P2_123);

    Gmsk = _mm_cmpgt_ps(P_Min, R_123);
    Lmsk = _mm_cmplt_ps(P_Max, NR_123);
    Or = _mm_or_ps(Gmsk, Lmsk);
    OutMsk = _mm_movemask_ps(Or);
    if (OutMsk & 0x7) return false;

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
    m128 E2_ZXYW = _mm_shuffle_ps(E2, E2, _MM_SHUFFLE(3, 1, 0, 2));
    m128 E2_YZXW = _mm_shuffle_ps(E2, E2, _MM_SHUFFLE(3, 0, 2, 1));
    P0_123 = _mm_sub_ps(_mm_mul_ps(E2_ZXYW, V0YZV1X), _mm_mul_ps(E2_YZXW, V0ZXV1Y));

    m128 V1YZV2X = _mm_shuffle_ps(V1, V2, _MM_SHUFFLE(3, 4, 2, 1));
    m128 V1ZXV2Y = _mm_shuffle_ps(V1, V2, _MM_SHUFFLE(3, 5, 0, 2));
    P2_123 = _mm_sub_ps(_mm_mul_ps(E2_ZXYW, V1YZV2X), _mm_mul_ps(E2_YZXW, V1ZXV2Y));

    P_Min = _mm_min_ps(P0_123, P2_123);
    P_Max = _mm_max_ps(P0_123, P2_123);

    m128 F2_ZZYW = _mm_shuffle_ps(F2, F2, _MM_SHUFFLE(3, 1, 2, 2));
    m128 F2_YXXW = _mm_shuffle_ps(F2, F2, _MM_SHUFFLE(3, 0, 0, 1));
    R_123 = _mm_add_ps(_mm_mul_ps(F2_ZZYW, H_YXXW), _mm_mul_ps(F2_YXXW, H_ZZYW));
    NR_123 = _mm_xor_ps(F32SgnMsk, R_123);
    
    Gmsk = _mm_cmpgt_ps(P_Min, R_123);
    Lmsk = _mm_cmplt_ps(P_Max, NR_123);

    Or = _mm_or_ps(Gmsk, Lmsk);
    OutMsk = _mm_movemask_ps(Or);
    if (OutMsk & 0x7) return false;

    return true;
}


static inline vec3
NormalSampler(vec3 C, const svo* const, const void* const UserData)
{
    morton_key MortonCode = EncodeMorton3(uvec3(C));

    morton_map* const* Index = (morton_map* const*)UserData;
    return (*Index)->at(MortonCode);
}

static inline vec3
ColourSampler(vec3 C, const svo* const, const void* const UserData)
{
    // Loading mesh colours isn't implemented, however it would
    // be simple to implement in the same manner as the normals.
    morton_key MortonCode = EncodeMorton3(uvec3(C));

    morton_map* const* Index = (morton_map* const*)UserData;

    return (*Index)->at(MortonCode);
}

static tri_buffer*
LoadMeshTriangles(const cgltf_data* const MeshData)
{
    cgltf_mesh* Mesh = &MeshData->meshes[0];

    usize LastTri = 0;
    tri_buffer* TriBuffer = nullptr;
    std::unordered_map<std::string, cgltf_material*> MaterialMap;
    MaterialMap.reserve(MeshData->materials_count);

    // Locate the primitive entry for the mesh's triangle data block.
    // We do not handle non-triangle meshes.
    for (cgltf_size PrimIndex = 0; PrimIndex < Mesh->primitives_count; ++PrimIndex)
    {
        cgltf_primitive* Prim = &Mesh->primitives[PrimIndex];

        if (cgltf_primitive_type_triangles == Prim->type)
        {
            if (Prim->indices)
            {
                cgltf_size IndexCount = Prim->indices->count;
                u32* IndexBuffer = (u32*) malloc(IndexCount * sizeof(u32));
                assert(IndexBuffer);

                // Copy the indices into the index buffer.
                for (cgltf_size Elem = 0; Elem < IndexCount; ++Elem)
                {
                    IndexBuffer[Elem] = (u32) cgltf_accessor_read_index(Prim->indices, Elem);
                }

                usize TriCount = IndexCount / 3;

                
                if (nullptr == TriBuffer)
                {
                    TriBuffer = (tri_buffer*) calloc(1, sizeof(tri_buffer) + (sizeof(tri_data) * TriCount));
                    assert(TriBuffer);
                }
                else
                {
                    TriBuffer = (tri_buffer*) realloc(TriBuffer, sizeof(tri_buffer) + (sizeof(tri_data) * (TriBuffer->TriangleCount + TriCount)));
                    assert(TriBuffer);
                }

                TriBuffer->TriangleCount += TriCount;

                pos_attrib* PosBuffer = nullptr;
                texcoord_attrib* TexCoordBuffer = nullptr;
                for (cgltf_size AttribIndex = 0; AttribIndex < Prim->attributes_count; ++AttribIndex)
                {
                    cgltf_attribute* Attrib = &Prim->attributes[AttribIndex];

                    cgltf_accessor* Accessor = Attrib->data;

                    if (cgltf_attribute_type_position == Attrib->type)
                    {
                        cgltf_size PosCount = Accessor->count;

                        assert(cgltf_num_components(Accessor->type) == 3);
                        PosBuffer = (pos_attrib*) malloc(sizeof(pos_attrib) * PosCount);
                        assert(PosBuffer);

                        cgltf_accessor_unpack_floats(Accessor, (f32*)PosBuffer, PosCount*3);
                    }
                    else if (cgltf_attribute_type_texcoord == Attrib->type)
                    {
                        // Load texcoords into a temporary buffer
                        cgltf_size TexCoordCount = Accessor->count;    
                        assert(cgltf_num_components(Accessor->type) == 2);
                        TexCoordBuffer = (texcoord_attrib*)malloc(sizeof(texcoord_attrib) * TexCoordCount);  
                        assert(TexCoordBuffer);
                        cgltf_accessor_unpack_floats(Accessor, (f32*)TexCoordBuffer, TexCoordCount * 2);
                    }

                }

                for (cgltf_size TriIndex = 0; TriIndex < (TriCount*3); TriIndex+=3)
                {
                    tri3 T;

                    // Load the indices
                    u32 I0 = IndexBuffer[TriIndex];
                    u32 I1 = IndexBuffer[TriIndex + 1];
                    u32 I2 = IndexBuffer[TriIndex + 2];
                    pos_attrib P0 = PosBuffer[I0];
                    pos_attrib P1 = PosBuffer[I1];
                    pos_attrib P2 = PosBuffer[I2];

                    if (TexCoordBuffer)
                    {
                        texcoord_attrib T0 = TexCoordBuffer[I0];
                        texcoord_attrib T1 = TexCoordBuffer[I1];
                        texcoord_attrib T2 = TexCoordBuffer[I2];

                        if (Prim->material)
                        {
                            TriBuffer->Triangles[LastTri].Material = Prim->material;
                        }

                        TriBuffer->Triangles[LastTri].TexCoord[0] = vec2{ T0.V[0], T0.V[1] };
                        TriBuffer->Triangles[LastTri].TexCoord[1] = vec2{ T1.V[0], T1.V[1] };
                        TriBuffer->Triangles[LastTri].TexCoord[2] = vec2{ T2.V[0], T2.V[1] };
                    }

                    // TODO Hack here to get around blender exporting bug
                    T.V0 = vec3(P0.V[0], (P0.V[1]), fabsf(P0.V[2]));
                    T.V1 = vec3(P1.V[0], (P1.V[1]), fabsf(P1.V[2]));
                    T.V2 = vec3(P2.V[0], (P2.V[1]), fabsf(P2.V[2]));

                    TriBuffer->Triangles[LastTri].T = T;
                    TriBuffer->Triangles[LastTri].Normal = ComputeTriangleNormal(&T);

                    ++LastTri;
                }

                if (PosBuffer) free(PosBuffer);
                if (TexCoordBuffer) free(TexCoordBuffer);

                // Didn't find any positions, free the index buffer and exit.
                free(IndexBuffer);
            }

        }

    }

    // Mesh does not have triangle primitives; bail out
    return TriBuffer;
}

static inline vec3
GetOctantCentre(u32 Oct, u32 Scale, vec3 ParentCentreP)
{
    assert(Scale > 0);

    f32 Rad = (f32)(Scale >> 1U);
    f32 X = (Oct & 1U) ? 1.0f : -1.0f;
    f32 Y = (Oct & 2U) ? 1.0f : -1.0f;
    f32 Z = (Oct & 4U) ? 1.0f : -1.0f;

    return ParentCentreP + (vec3(X, Y, Z) * Rad);
}

static inline void
MeshMinMaxDimensions(const cgltf_mesh* const Mesh, vec3& MinOut, vec3& MaxOut)
{
    // Check the max/min values of each primitive
    vec3 Min(F32_MAX);
    vec3 Max(F32_MIN);

    for (u32 PrimIndex = 0; PrimIndex < Mesh->primitives_count; ++PrimIndex)
    {
        const cgltf_primitive* const Prim = &Mesh->primitives[PrimIndex];
        cgltf_accessor* PosAttr = nullptr;
        for (u32 AttrIndex = 0; AttrIndex < Prim->attributes_count; ++AttrIndex)
        {
            if (cgltf_attribute_type_position == Prim->attributes[AttrIndex].type)
            {
                PosAttr = Prim->attributes[AttrIndex].data;
            }
        }

        if (nullptr != PosAttr && PosAttr->has_min)
        {
            f32* Mins = PosAttr->min;
            f32* Maxes = PosAttr->max;

            if (Mins[0] < Min.X) Min.X = Mins[0];
            if (Mins[1] < Min.Y) Min.Y = Mins[1];
            if (Mins[2] < Min.Z) Min.Z = Mins[2];
        
            if (Maxes[0] > Max.X) Max.X = Maxes[0];
            if (Maxes[1] > Max.Y) Max.Y = Maxes[1];
            if (Maxes[2] > Max.Z) Max.Z = Maxes[2];
        }
    }
    
    // TODO: Try to eliminate copying here
    MinOut = Min;
    MaxOut = Max;
}


static inline u32
NextPowerOf2Exponent(u32 X)
{ 
    return (X > 1U) ? 32U - (u32)__builtin_clz(X - 1U)
                    : 1U;
}  


static void
BuildTriangleIndex(u32 MaxDepth,
                   u32 ScaleExponent,
                   tri_buffer* Tris,
                   std::set<morton_key>& IndexOut,
                   morton_map& NormalsMap,
                   morton_map& ColourMap)
{
    struct st_ctx
    {
        u32 Oct;
        u32 Scale;
        u32 Depth;
        vec3 ParentCentre;
    };

    std::deque<st_ctx> Stack;

    svo_bias Bias = ComputeScaleBias(MaxDepth, ScaleExponent);

    u32 RootScale = 1U << (ScaleExponent) << Bias.Scale;
    vec3 ParentCentreP = vec3(RootScale >> 1U);

    st_ctx RootCtx;
    RootCtx.Oct = 0;
    RootCtx.Depth = 1;
    RootCtx.Scale = RootScale >> 1U;
    RootCtx.ParentCentre = ParentCentreP;

    morton_key RootCode = EncodeMorton3(uvec3(ParentCentreP));

    IndexOut.insert(RootCode);

    for (size_t TriIndex = 0; TriIndex < Tris->TriangleCount; ++TriIndex)
    {
        tri3 T = Tris->Triangles[TriIndex].T;

        // For every triangle, check if it is enclosed in a bounding box
        Stack.push_front(RootCtx);

        alignas(16) m128 TriVerts[3];
        TriVerts[0] = _mm_set_ps(0.0f, T.V0.Z, T.V0.Y, T.V0.X);
        TriVerts[1] = _mm_set_ps(0.0f, T.V1.Z, T.V1.Y, T.V1.X);
        TriVerts[2] = _mm_set_ps(0.0f, T.V2.Z, T.V2.Y, T.V2.X);

        while (false == Stack.empty())
        {
            st_ctx CurrentCtx = Stack.front();
            Stack.pop_front();

            vec3 Radius = vec3(CurrentCtx.Scale >> 1U) * Bias.InvScale;
            alignas(16) m128 RadiusM = _mm_set_ps(0.0f, Radius.Z, Radius.Y, Radius.X);

            for (u32 Oct = 0; Oct < 8; ++Oct)
            {
                // TODO(Liam): Make GetOctantCentre *only* return uvecs, then can differentiate
                // at the type level between scaled and unscaled vectors.
                vec3 Centre = GetOctantCentre(Oct, CurrentCtx.Scale, CurrentCtx.ParentCentre);
                alignas(16) m128 CentreM = _mm_set_ps(0.0f, Centre.Z*Bias.InvScale, Centre.Y*Bias.InvScale, Centre.X*Bias.InvScale);

                if (TriangleAABBIntersection(CentreM, RadiusM, TriVerts))
                {
                    morton_key ChildVoxelCode = EncodeMorton3(uvec3(Centre)); 
                    if ((CurrentCtx.Depth + 1) >= MaxDepth)
                    {
                        NormalsMap.emplace(ChildVoxelCode, Tris->Triangles[TriIndex].Normal);
                        ColourMap.emplace(ChildVoxelCode, ComputeVoxelColour(&Tris->Triangles[TriIndex], Centre*Bias.InvScale));
                    }

                    IndexOut.insert(ChildVoxelCode);

                    if (CurrentCtx.Depth < MaxDepth)
                    {
                        st_ctx NewCtx;
                        NewCtx.Oct = Oct;
                        NewCtx.Depth = CurrentCtx.Depth + 1;
                        NewCtx.Scale = CurrentCtx.Scale >> 1U;
                        NewCtx.ParentCentre = Centre;
                        Stack.push_front(NewCtx);
                    }
                }
            }
        }
    }
}


static bool
IntersectorFunction(vec3 vMin, vec3 vMax, const svo* const Tree, const void* const UserData)
{
    vec3 Halfsize = (vMax - vMin) * 0.5f;
    vec3 Centre = ((vMin + Halfsize) * f32(1 << Tree->Bias.Scale));

    const std::set<morton_key>* const* OccupancyIndex = (const std::set<morton_key>* const*)UserData;

    morton_key MortonCode = EncodeMorton3(uvec3(Centre));
    return (*OccupancyIndex)->find(MortonCode) != (*OccupancyIndex)->end();
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

    return Maximum(Maximum(MaxX, MaxY), MaxZ);
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

    return Minimum(Minimum(MinX, MinY), MinZ);
}


extern "C" svo*
ImportGLBFile(u32 MaxDepth, const char* const GLTFPath)
{
	cgltf_options Options = { };
    cgltf_data* Data = nullptr;

	cgltf_result Result = cgltf_parse_file(&Options, GLTFPath, &Data);
    GlobalTextureCache = new std::unordered_map<cgltf_image*, img>();

    Result = cgltf_load_buffers(&Options, Data, nullptr);
    Result = cgltf_validate(Data);

	if (cgltf_result_success == Result)
    {
        tri_buffer* TriangleData = LoadMeshTriangles(Data);

        if (nullptr == TriangleData)
        {
            fprintf(stderr, "Failed to load triangle data");
            cgltf_free(Data);

            return nullptr;
        }

        // Compute max scale exponent from mesh max and min vertices.
        f32 MaxDim = GetMeshMaxDimension(&Data->meshes[0].primitives[0]);
        vec3 Min, Max;
        MeshMinMaxDimensions(&Data->meshes[0], Min, Max);
        printf("%lf\n", f64(MaxDim));
        DEBUGPrintVec3(Min);
        DEBUGPrintVec3(Max);

        assert(MaxDim > 0);
        u32 ScaleExponent = NextPowerOf2Exponent((u32)ceilf(MaxDim));
        assert(ScaleExponent > 0);

        std::set<morton_key> OccupancyIndex{};
        morton_map NormalIndex{};
        morton_map ColourIndex{};

        NormalIndex.reserve(TriangleData->TriangleCount);
        ColourIndex.reserve(TriangleData->TriangleCount);
        BuildTriangleIndex(MaxDepth,
                           ScaleExponent,
                           TriangleData,
                           OccupancyIndex,
                           NormalIndex,
                           ColourIndex);


        const morton_map* const NormalsPtr = &NormalIndex;
        const morton_map* const ColoursPtr = &ColourIndex;
        const std::set<morton_key>* const OccupancyIndexPtr = &OccupancyIndex;

        shape_sampler ShapeS = shape_sampler{ &OccupancyIndexPtr, IntersectorFunction };
        data_sampler NormalS = data_sampler{ &NormalsPtr, NormalSampler };
        data_sampler ColourS = data_sampler{ &ColoursPtr, ColourSampler };

        svo* Svo = CreateScene(ScaleExponent,
                               MaxDepth,
                               &ShapeS,
                               &NormalS,
                               &ColourS);

        NormalIndex.clear();
        free(TriangleData);
        cgltf_free(Data);

        for (auto It = GlobalTextureCache->begin(); It != GlobalTextureCache->end(); ++It)
        {
            stbi_image_free(It->second.Pixels);
        }

        delete GlobalTextureCache;
        return Svo;
    }
    else
    {
        fprintf(stderr, "Failed to load mesh data file\n");

        if (Data) cgltf_free(Data);
        delete GlobalTextureCache;
        return nullptr;
    }
}

