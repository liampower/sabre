#ifndef SABRE_SVO_H
#define SABRE_SVO_H

static constexpr u32 SVO_ENTRIES_PER_BLOCK  = 4096;
static constexpr u32 SVO_FAR_PTRS_PER_BLOCK = 64;
static constexpr u32 SVO_FAR_PTR_BIT_MASK   = 0x8000;

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
    svo_block* Block; // Which particular block this node points into.
    u32 Offset;       // Which slot inside that block the node resides at. 
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

    svo_block* CurrentBlock;
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

#endif
