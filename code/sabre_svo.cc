#include "sabre.h"
#include "sabre_math.h"

#define SVO_ENTRIES_PER_BLOCK 64
#define SVO_INITIAL_BLOCKS 4

typedef u32 svo_entry;

typedef u32 (*intersector_fn)(vec3, vec3);

struct svo_block
{
    svo_entry Entries[SVO_ENTRIES_PER_BLOCK];
};

struct svo
{
    u32 UsedBlockCount;
    u32 FreeBlockCount;
    u32 MaxDepth;

    svo_block* Blocks;
};

enum svo_oct
{
    OCT_C000 = 0,
    OCT_C001 = 1,
    OCT_C010 = 2,
    OCT_C011 = 3,
    OCT_C100 = 4,
    OCT_C101 = 5,
    OCT_C110 = 6,
    OCT_C111 = 7,
};

static void
AllocateMoreBlocks(svo* const Tree)
{

}

static svo_entry*
AllocateChildEntry(svo* const Tree)
{
    if (Tree->UsedBlockCount + 1 < Tree->FreeBlockCount)
    {
        // Gone over the block limit; need to allocate more blocks
        AllocateMoreBlocks(Tree);
    }
    else
    {
    }
}

static inline vec3
GetNodeCentrePosition(svo_oct Octant, u32 Depth, vec3 ParentCentreP)
{
    u32 Oct = (u32) Octant;
    u32 Radius = 1U << Depth;

    f32 X = (Oct & 1U) ? 1.0f : -1.0f;
    f32 Y = (Oct & 2U) ? 1.0f : -1.0f;
    f32 Z = (Oct & 4U) ? 1.0f : -1.0f;

    return ParentCenter + (vec3(X, Y, Z) * Radius);
}


extern svo*
BuildSparseVoxelOctree(u32 MaxDepth, surface_fn SurfaceFn)
{
    svo* Tree = (svo*) calloc(sizeof(svo) + (SVO_INITIAL_BLOCKS * sizeof(svo_block)));

    if (Tree)
    {
        Tree->MaxDepth = MaxDepth;

        // Start off with the pre-selected amount of blocks free
        Tree->UsedBlockCount = 0;
        Tree->FreeBlockCount = SVO_INITIAL_BLOCKS;

        // The entire tree is centred at the origin.
        vec3 TreeCentreP = vec3(0, 0, 0);

        svo_entry* RootEntry = AllocateChildEntry(Tree);

        InsertNode(Tree, SurfaceFn, OCT_C000, TreeCentreP);

    }
    else
    {
        return nullptr;
    }
}

