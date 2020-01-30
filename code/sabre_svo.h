#ifndef SABRE_SVO_H
#define SABRE_SVO_H

static constexpr u32 SVO_ENTRIES_PER_BLOCK  = 16;
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
    // How many blocks forwards (negative values are backwards) needed
    // to be traversed to arrive at the resident block
    i32 BlkOffset;

    // Which slot in the resident block the particular child node
    // resides at.
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
    svo_block* Prev;
    svo_block* Next;
    svo_node   Entries[SVO_ENTRIES_PER_BLOCK];
    far_ptr    FarPtrs[SVO_FAR_PTRS_PER_BLOCK];
};

#if 0
struct svo_edit
{
};

void UpdateSvo(svo_edit* Cmds, usize CmdCount);
#endif



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

    svo_block** BlkTable;
    usize       BlkCount;
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
