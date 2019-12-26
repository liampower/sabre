#ifndef SABRE_SVO_H
#define SABRE_SVO_H

constexpr u32 SVO_ENTRIES_PER_BLOCK = 4096;

typedef bool (*intersector_fn)(vec3, vec3);

struct svo_node
{
    u16 ChildPtr;
    u8  OccupiedMask;
    u8  LeafMask;
};

struct svo_block
{
    usize      NextFreeSlot;
    svo_block* Prev;
    svo_node   Entries[SVO_ENTRIES_PER_BLOCK];
};

struct svo
{
    u32 UsedBlockCount;
    u32 MaxDepth;

    svo_block* CurrentBlock;
    svo_block* DEBUGFirstBlock;
};

extern svo*
BuildSparseVoxelOctree(u32 MaxDepth, intersector_fn Surface);

extern void
DeleteSparseVoxelOctree(svo* Tree);

#endif
