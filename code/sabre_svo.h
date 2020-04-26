#ifndef SABRE_SVO_H
#define SABRE_SVO_H

#include <vector>

#include "sabre.h"
#include "sabre_math.h"

static constexpr u32 SBR_NODES_PER_BLK = 8192;
static constexpr u32 SBR_FAR_PTRS_PER_BLK = 8192;

static_assert(SBR_FAR_PTRS_PER_BLK >= SBR_NODES_PER_BLK, "Far Ptrs Per Blk must be >= Entries per Blk");

struct svo_block;

typedef uint32_t packed_snorm3;
typedef uint32_t packed_snorm3;
struct sbr_svo;

typedef sbrv3 (*data_sampler_fn)(sbrv3, const sbr_svo* const, const void* const);
typedef bool (*shape_sampler_fn)(sbrv3, sbrv3, const sbr_svo* const, const void* const);

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

struct sbr_far_ptr
{
    // Index of the child node's block
    uint32_t BlkIndex;

    // Which slot in the child block the 
    // particular child node resides at
    uint32_t NodeOffset;
};

enum svo_blk_flags
{
    SVO_BLK_LEAKY = 0x000001
};

union alignas(4) svo_node
{
    struct
    {
        uint8_t  LeafMask;
        uint8_t  OccupiedMask;
        uint16_t ChildPtr;
    };

    uint32_t Packed;
};


struct svo_bias
{
    float    InvScale;
    uint32_t Scale;
};


struct svo_block
{
    size_t      NextFreeSlot;
    size_t      NextFarPtrSlot;
    uint32_t    Index;
    svo_block*  Prev;
    svo_block*  Next;
    svo_node    Entries[SBR_NODES_PER_BLK];
    sbr_far_ptr FarPtrs[SBR_FAR_PTRS_PER_BLK];
    uint32_t    LeafDataPtrs[8192];
};

struct sbr_svo
{
    // How many total blocks are used in the tree.
    uint32_t UsedBlockCount;

    // The maximum depth of the svo to traverse
    uint32_t MaxDepth;

    // Extant of the octree in world space.
    uint32_t ScaleExponent;

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


    std::vector<std::pair<sbrv3u, packed_snorm3>> Normals;

    // TODO Use unorms
    std::vector<std::pair<sbrv3u, packed_snorm3>> Colours;
};

typedef sbr_surface (*intersector_fn)(sbrv3, sbrv3, const sbr_svo* const, const void* const);
typedef sbrv3 (*normal_fn)(sbrv3, const sbr_svo* const, const void* const);
typedef sbrv3 (*colour_fn)(sbrv3, const sbr_svo* const, const void* const);


extern "C" void
SBR_InsertVoxel(sbr_svo* Tree, sbrv3 P, uint32_t VoxelScale);

extern "C" void
SBR_DeleteVoxel(sbr_svo* Tree, sbrv3 P);

extern "C" svo_bias
SBR_ComputeScaleBias(uint32_t MaxDepth,
                     uint32_t ScaleExponent);

extern "C" void
SBR_DeleteScene(sbr_svo* Tree);

extern "C" sbr_svo*
SBR_ImportGLBFile(uint32_t MaxDepth,
                  const char* const GLTFPath);

#if 0
extern "C" sbr_svo*
SBR_CreateScene(u32 ScaleExponent,
                u32 MaxDepth,
                intersector_fn Surface,
                normal_fn NormalFn,
                colour_fn ColourFn);
#endif

extern "C" sbr_svo*
SBR_CreateScene(uint32_t ScaleExp,
                uint32_t MaxDepth,
                shape_sampler* SurfaceFn,
                data_sampler* NormalFn,
                data_sampler* ColourFn);

extern "C" void
OutputSvoToFile(const sbr_svo* const Svo, FILE* FileOut);

extern "C" sbr_svo*
LoadSvoFromFile(FILE* FileIn);

extern "C" unsigned int
GetSvoUsedBlockCount(const sbr_svo* const Svo);

extern "C" unsigned int
GetSvoDepth(const sbr_svo* const Svo);

static inline void
DEBUGPrintVec3(sbrv3 V)
{
    printf("(%f, %f, %f)", (f64)V.X, (f64)V.Y, (f64)V.Z);
}

#endif
