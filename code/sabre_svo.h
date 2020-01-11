#ifndef SABRE_SVO_H
#define SABRE_SVO_H

constexpr u32 SVO_ENTRIES_PER_BLOCK = 4096;

typedef bool (*intersector_fn)(vec3, vec3);

#if 0
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
#endif

struct svo_node
{
    u8  LeafMask;
    u8  OccupiedMask;
    u16 ChildPtr;
};

struct svo_block
{
    usize      NextFreeSlot;
    svo_block* Prev;
    svo_node   Entries[SVO_ENTRIES_PER_BLOCK];
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
    svo_block* DEBUGFirstBlock;
};

extern "C" svo*
BuildSparseVoxelOctree(u32 ScaleExponent,
                       u32 MaxDepth,
                       intersector_fn Surface);

extern "C" void
DeleteSparseVoxelOctree(svo* Tree);

#endif
