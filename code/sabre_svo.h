#ifndef SABRE_SVO_H
#define SABRE_SVO_H

constexpr u32 SVO_ENTRIES_PER_BLOCK = 4096;
constexpr u32 SVO_FAR_PTRS_PER_BLOCK = 64;

typedef bool (*intersector_fn)(vec3, vec3);

struct svo_block;

struct far_ptr
{
    svo_block* Block; // Which particular block this node points into.
    u32 Offset;       // Which slot inside that block the node resides at. 
};

union alignas(4) svo_node
{
    struct
    {
        u16 ChildPtr;
        u8  LeafMask;
        u8  OccupiedMask;
    };

    u32 Packed;
};

struct svo_block
{
    usize      NextFreeSlot;
    svo_block* Prev;
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

#endif
