#ifndef SABRE_SVO_H
#define SABRE_SVO_H

constexpr u32 SVO_ENTRIES_PER_BLOCK  = 4096;
constexpr u32 SVO_FAR_PTRS_PER_BLOCK = 4096;
constexpr u32 SVO_FAR_PTR_BIT_MASK   = 0x8000;


static_assert(SVO_FAR_PTRS_PER_BLOCK >= SVO_ENTRIES_PER_BLOCK, "Far Ptrs Per Blk must be >= Entries per Blk");

typedef bool (*intersector_fn)(vec3, vec3);

struct svo_block;

enum svo_oct
{
    OCT_C000 = 0,
    OCT_C001 = 1,
    OCT_C010 = 2,
    OCT_C011 = 3,
    OCT_C100 = 4,
    OCT_C101 = 5,
    OCT_C110 = 6,
    OCT_C111 = 7
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

struct svo_block
{
    usize      NextFreeSlot;
    usize      NextFarPtrSlot;
    u32        Index;
    svo_block* Prev;
    svo_block* Next;
    svo_node   Entries[SVO_ENTRIES_PER_BLOCK];
    far_ptr    FarPtrs[SVO_FAR_PTRS_PER_BLOCK];
};

struct svo
{
    // How many total blocks are used in the tree.
    u32 UsedBlockCount;

    // The maximum depth of the svo to traverse
    u32 MaxDepth;

    // Extant of the octree in world space.
    u32 ScaleExponent;

    // Last block of nodes in this tree
    // NOTE(Liam): Warning! This field is volatile and unsafe
    // to use; it is frequently modified by the implementation
    // and probably not a good way to achieve much of anything
    // besides tree construction!
    svo_block* LastBlock;

    // First block of nodes in this tree
    svo_block* RootBlock;
};

extern "C" svo*
BuildSparseVoxelOctree(u32 ScaleExponent,
                       u32 MaxDepth,
                       intersector_fn Surface);

extern "C" void
InsertVoxel(svo* Svo, vec3 P, u32 VoxelScale);

extern "C" void
DeleteSparseVoxelOctree(svo* Tree);

extern "C" svo*
BuildFromPlyFile(const char* const FileName);

extern "C" svo*
CreateSparseVoxelOctree(u32 ScaleExponent,
                        u32 MaxDepth,
                        intersector_fn Surface);

static inline void
DEBUGPrintVec3(vec3 V)
{
    printf("(%f, %f, %f)", (f64)V.X, (f64)V.Y, (f64)V.Z);
}

#endif
