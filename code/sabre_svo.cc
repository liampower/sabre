#include <assert.h>
#include <stdlib.h>

#include "sabre.h"
#include "sabre_math.h"
#include "sabre_svo.h"

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

enum svo_voxel_type
{
    VOXEL_PARENT = 0U,
    VOXEL_LEAF   = 1U
};


static inline void
SetNodeOccupied(svo_oct SubOctant, svo_voxel_type Type, svo_node* OutEntry)
{
   u32 ValidBit = 1U << SubOctant; 

   OutEntry->OccupiedMask |= ValidBit;

   if (VOXEL_LEAF == Type)
   {
       u32 LeafBit = 1U << SubOctant;

       OutEntry->LeafMask |= LeafBit;
   }
}


static inline void
SetNodeChildPointer(u16 ChildPtr, bool InSameBlock, svo_node* OutNode)
{
    if (InSameBlock)
    {
        // Extract first 15 bits
        OutNode->ChildPtr = ChildPtr & 0x7FFF;
    }
    else
    {
        // Not implemented
        assert(false);
    }
}


// TODO(Liam): Need a *way* more clear API here!
static inline vec3
GetNodeCentrePosition(svo_oct Octant, u32 Radius, vec3 ParentCentreP)
{
    u32 Oct = (u32) Octant;

    f32 X = (Oct & 1U) ? 1.0f : -1.0f;
    f32 Y = (Oct & 2U) ? 1.0f : -1.0f;
    f32 Z = (Oct & 4U) ? 1.0f : -1.0f;

    return ParentCentreP + (vec3(X, Y, Z) * Radius);
}


static void
AllocateNewBlock(svo* const Tree)
{
    svo_block* OldBlock = Tree->CurrentBlock;
    svo_block* NewBlock = (svo_block*) calloc(1, sizeof(svo_block));

    NewBlock->Prev = OldBlock; // Link together
    Tree->CurrentBlock = NewBlock;
    ++Tree->UsedBlockCount;
}


static inline bool
NeedMoreBlocks(svo* const Tree)
{
    return SVO_ENTRIES_PER_BLOCK <= Tree->CurrentBlock->NextFreeSlot;
}


static svo_node*
AllocateChildEntry(svo* const Tree)
{
    if (NeedMoreBlocks(Tree))
    {
        AllocateNewBlock(Tree);
    }

    svo_node* Child = &Tree->CurrentBlock->Entries[Tree->CurrentBlock->NextFreeSlot];
    ++Tree->CurrentBlock->NextFreeSlot;

    return Child;
}


static svo_node*
AllocateNodes(svo* const Tree, usize NodeCount)
{
    // FIXME(Liam): Need cleanup here
    svo_node* Head = AllocateChildEntry(Tree);
    for (u32 NodeIndex = 0; NodeIndex < NodeCount - 1; ++NodeIndex)
    {
        AllocateChildEntry(Tree);
    }

    return Head;
}


static void
InsertNode(svo* Tree, intersector_fn Surface, u32 Depth, svo_oct Octant, svo_node* ParentNode, vec3 ParentCentreP)
{
    u32 Scale = 1 << (Tree->MaxDepth - Depth);

    if (Scale > 0 && Depth <= Tree->MaxDepth)
    {
        vec3 Radius = vec3(Scale);

        vec3 NodeCentreP = GetNodeCentrePosition(Octant, Scale, ParentCentreP);
        vec3 NodeMinP = NodeCentreP - Radius;
        vec3 NodeMaxP = NodeCentreP + Radius;

        // Test the surface for this octant
        if (Surface(NodeMinP, NodeMaxP))
        {
            // Set this octant occupied
            SetNodeOccupied(Octant, VOXEL_PARENT, ParentNode);

            // Allocate a new "sub-parent"
            svo_node* Child = AllocateChildEntry(Tree);

            for (u32 ChildOctant = 0; ChildOctant < 8; ++ChildOctant)
            {
                InsertNode(Tree, Surface, Depth + 1, (svo_oct) ChildOctant, Child, NodeCentreP);
            }

            // NOTE(Liam): Do we *really* need the child ptrs? If the nodes
            // are in depth order anyway the children are always directly
            // adjacent to their parent.
        }
        else
        {
            SetNodeOccupied(Octant, VOXEL_LEAF, ParentNode);
        }
    }
    else
    {
        SetNodeOccupied(Octant, VOXEL_LEAF, ParentNode);
    }
}


extern svo*
BuildSparseVoxelOctree(u32 MaxDepth, intersector_fn SurfaceFn)
{
    // TODO(Liam): Maybe combine allocation ?
    svo* Tree = (svo*) calloc(1, sizeof(svo));

    if (Tree)
    {
        Tree->MaxDepth = MaxDepth;

        Tree->CurrentBlock = (svo_block*) calloc(1, sizeof(svo_block));
        Tree->UsedBlockCount = 1;
        Tree->DEBUGFirstBlock = Tree->CurrentBlock;

        u32 R = 1 << MaxDepth;

        vec3 TreeParentP = vec3(R);

        svo_node* RootEntry = AllocateChildEntry(Tree);

        // Add the root node to the tree. This kicks off the recursive
        // tree building.
        //InsertNode(Tree, SurfaceFn, 0, OCT_C000, TreeParentP, RootEntry);

        InsertNode(Tree, SurfaceFn, 0, OCT_C000, RootEntry, TreeParentP);

        return Tree;
    }
    else
    {
        return nullptr;
    }
}


extern void
DeleteSparseVoxelOctree(svo* Tree)
{
    for (u32 BlockIndex = 0; BlockIndex < Tree->UsedBlockCount; ++BlockIndex)
    {
        svo_block* Block = Tree->CurrentBlock;
        Tree->CurrentBlock = Block->Prev;

        free(Block);
    }

    free(Tree);
}

