#ifndef SABRE_SVO_H
#define SABRE_SVO_H

#include <vector>

#include "sabre.h"
#include "sabre_math.h"

static constexpr u32 SBR_NODES_PER_BLK = 8192;
static constexpr u32 SBR_FAR_PTRS_PER_BLK = 8192;

static_assert(SBR_FAR_PTRS_PER_BLK >= SBR_NODES_PER_BLK, "Far Ptrs Per Blk must be >= Entries per Blk");

struct svo_block;

typedef u32 packed_snorm3;
typedef u32 packed_snorm3;

struct svo;

typedef vec3 (*data_sampler_fn)(vec3, const svo* const, const void* const);
typedef bool (*shape_sampler_fn)(vec3, vec3, const svo* const, const void* const);

struct shape_sampler
{
    const void* const UserData;
    const shape_sampler_fn SamplerFn;
};

struct data_sampler
{
    const void* const UserData;
    const data_sampler_fn SamplerFn;
};

enum sbr_surface
{
    SURFACE_INTERSECTED,
    SURFACE_INSIDE,
    SURFACE_OUTSIDE
};

struct far_ptr
{
    // Index of the child node's block
    u32 BlkIndex;

    // Which slot in the child block the 
    // particular child node resides at
    u32 NodeOffset;
};

enum svo_blk_flags
{
    SVO_BLK_LEAKY = 0x000001
};

union alignas(4) svo_node
{
    struct
    {
        u8  LeafMask;
        u8  OccupiedMask;
        u16 ChildPtr;
    };

    u32 Packed;
};


struct svo_bias
{
    float    InvScale;
    u32 Scale;
};


struct svo_block
{
    usize       NextFreeSlot;
    usize       NextFarPtrSlot;
    u32         Index;
    svo_block*  Prev;
    svo_block*  Next;
    svo_node    Entries[SBR_NODES_PER_BLK];
    far_ptr     FarPtrs[SBR_FAR_PTRS_PER_BLK];
};

struct svo
{
    // How many total blocks are used in the tree.
    u32 UsedBlockCount;

    // The maximum depth of the svo to traverse
    u32 MaxDepth;

    // Extant of the octree in world space.
    u32 ScaleExponent;

    // Required to maintain subtree-scale values as
    // integers. 
    //
    // In the case that MaxDepth >= ScaleExponent, the
    // scale values of the subtrees will become fractional
    // values. This scale factor must first be applied to
    // the entire tree scale so that the smallest extant
    // of any child region is 1. Remember to divide by this
    // value when doing any space-operations!
    svo_bias Bias;

    // Last block of nodes in this tree
    // NOTE(Liam): Warning! This field is volatile and unsafe
    // to use; it is frequently modified by the implementation
    // and probably not a good way to achieve much of anything
    // besides tree construction!
    svo_block* LastBlock;

    // First block of nodes in this tree
    svo_block* RootBlock;


    std::vector<std::pair<uvec3, packed_snorm3>> Normals;

    // TODO Use unorms
    std::vector<std::pair<uvec3, packed_snorm3>> Colours;
};

typedef sbr_surface (*intersector_fn)(vec3, vec3, const svo* const, const void* const);
typedef vec3 (*normal_fn)(vec3, const svo* const, const void* const);
typedef vec3 (*colour_fn)(vec3, const svo* const, const void* const);


extern "C" vec3
GetNearestFreeSlot(vec3 Pos, vec3 Dir, const svo* const Tree);

extern "C" vec3
GetNearestLeafSlot(vec3 Pos, vec3 Dir, const svo* const Tree);

extern "C" void
InsertVoxel(svo* Tree, vec3 P);

extern "C" void
DeleteVoxel(svo* Tree, vec3 P);

extern "C" svo_bias
ComputeScaleBias(u32 MaxDepth,
                 u32 ScaleExponent);

extern "C" void
DeleteScene(svo* Tree);

extern "C" svo*
ImportGLBFile(u32 MaxDepth,
              const char* const GLTFPath);

extern "C" svo*
CreateScene(u32 ScaleExp,
            u32 MaxDepth,
            shape_sampler* SurfaceFn,
            data_sampler* NormalFn,
            data_sampler* ColourFn);

extern "C" void
OutputSvoToFile(const svo* const Svo, FILE* FileOut);


extern "C" unsigned int
GetSvoUsedBlockCount(const svo* const Svo);

extern "C" unsigned int
GetSvoDepth(const svo* const Svo);

extern "C" void*
CopyRawBlockData(const svo_block* const Blk, usize CopySize, void* const BlkDataOut);


static inline void
DEBUGPrintVec3(vec3 V)
{
    printf("(%f, %f, %f)", (f64)V.X, (f64)V.Y, (f64)V.Z);
}

#endif
