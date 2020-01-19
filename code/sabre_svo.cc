#include <assert.h>
#include <stdlib.h>
#include <queue>

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

static void
GetNodeBounds(u32 Oct, u32 Scale, vec3 ParentCentre, vec3* Min, vec3* Max)
{
    printf("SCALE %d\n", Scale);
    vec3 Dir;
    vec3 Rad = vec3(Scale);
    bvec3 Msk = bvec3(uvec3(Oct) & uvec3(1, 2, 4));;

    Dir.X = (Oct & 1) ? 1.0 : -1.0;
    Dir.Y = (Oct & 2) ? 1.0 : -1.0;
    Dir.Z = (Oct & 4) ? 1.0 : -1.0;

    if (Oct & 2)
    {
        *Min = ParentCentre; 
        *Max = ParentCentre + (Dir * Rad);
    }
    else
    {
        *Max = ParentCentre; 
        *Min = ParentCentre + (Dir * Rad);
    }

}

static inline void
SetOctantOccupied(svo_oct SubOctant, svo_voxel_type Type, svo_node* OutEntry)
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
SetNodeChildPointer(u16 ChildPtr, bool InSameBlock, svo_node* OutParentNode)
{
    // If the child is in the same block, we can just treat the child ptr as
    // a direct offset into the block array.
    if (InSameBlock)
    {
        // Extract first 15 bits
        // TODO(Liam): Implement support for far ptrs
        OutParentNode->ChildPtr = ChildPtr;// & 0x7FFF;

    }
    else
    {
        // Not implemented
        assert(false);
    }
}


// TODO(Liam): Need a *way* more clear API here!
static inline vec3
GetNodeCentrePosition(svo_oct Octant, u32 Scale, vec3 ParentCentreP)
{
    u32 Rad = Scale >> 1;
    u32 Oct = (u32) Octant;

    f32 X = (Oct & 1U) ? 1.0f : -1.0f;
    f32 Y = (Oct & 2U) ? 1.0f : -1.0f;
    f32 Z = (Oct & 4U) ? 1.0f : -1.0f;

    return ParentCentreP + (vec3(X, Y, Z) * Rad);
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
AllocateNode(svo* const Tree)
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
ProcessOctant(svo* Tree, svo_oct Octant, u32 Scale, u32 Depth, intersector_fn Surface, svo_node* ContainingNode, vec3 OctantCentre, vec3 ParentCentre)
{
    vec3 Radius = vec3(Scale >> 1);
    vec3 OctMin = OctantCentre - Radius;
    vec3 OctMax = OctantCentre + Radius;

    //vec3 OctMin, OctMax;
    //GetNodeBounds((u32)Octant, Scale, ParentCentre, &OctMin, &OctMax);


    if (Surface(OctMin, OctMax))
    {
        if (Depth < Tree->MaxDepth - 1)
        {
            SetOctantOccupied(Octant, VOXEL_PARENT, ContainingNode);

            // Allocate a new child for this node
            svo_node* Child = AllocateNode(Tree);

            return Child;
        }
        else
        {
            SetOctantOccupied(Octant, VOXEL_LEAF, ContainingNode);
        }

    }

    return nullptr;
}


struct node_context
{
    svo_node* Node;
    svo_oct   Oct;
    u32       Depth;
    u32       Scale;
    vec3      Centre;
};



static void
BuildTree(svo* Tree, intersector_fn Surface, svo_node* Root)
{
    std::queue<node_context> Queue;

    u32 RootScale = 1 << (Tree->ScaleExponent);
    vec3 RootCentre = vec3(RootScale >> 1);

    node_context RootContext = { Root, OCT_C000, 0, RootScale, RootCentre };
    Queue.push(RootContext);

    while (false == Queue.empty())
    {
        node_context CurrentContext = Queue.front();

        printf("\nBEGIN L(%d)\n", CurrentContext.Depth);

        u32 Scale = CurrentContext.Scale >> 1;

        //printf("SCALE %u   CTR (%f %f %f)\n", Scale, CurrentContext.Centre.X, CurrentContext.Centre.Y, CurrentContext.Centre.Z);
        for (u32 Oct = 0; Oct < 8; ++Oct)
        {
            vec3 OctantCentre = GetNodeCentrePosition((svo_oct) Oct, Scale, CurrentContext.Centre);

            svo_node* MaybeChild = ProcessOctant(Tree, (svo_oct) Oct, Scale, CurrentContext.Depth, Surface, CurrentContext.Node, OctantCentre, CurrentContext.Centre);

            // Check if octant needs to be subdivided
            if (MaybeChild && CurrentContext.Depth < Tree->MaxDepth - 1)
            {
                node_context ChildContext = { MaybeChild, (svo_oct)Oct, CurrentContext.Depth + 1, Scale, OctantCentre };

                // If the parent node's child ptr hasn't already been set, we can set it here.
                // If it's already been set, that means that some octant before this one has 
                // already set it, so we can assume that we follow that one in sequential memory
                // order.
                if (CurrentContext.Node->ChildPtr == 0x0000)
                {
                    ptrdiff_t ChildOffset = MaybeChild - Tree->CurrentBlock->Entries;
                    // TODO(Liam): Handle case where ChildOffset > U16_MAX, this is where we
                    // do far ptrs
                    SetNodeChildPointer((u16) ChildOffset, true, CurrentContext.Node);
                }

                Queue.push(ChildContext);
            }
        }
        
        Queue.pop();
    }
}



extern "C" svo*
BuildSparseVoxelOctree(u32 ScaleExponent, u32 MaxDepth, intersector_fn SurfaceFn)
{
    // TODO(Liam): Maybe combine allocation ?
    svo* Tree = (svo*) calloc(1, sizeof(svo));

    if (Tree)
    {
        Tree->MaxDepth = MaxDepth;
        Tree->ScaleExponent = ScaleExponent;

        Tree->CurrentBlock = (svo_block*) calloc(1, sizeof(svo_block));
        Tree->UsedBlockCount = 1;
        Tree->DEBUGFirstBlock = Tree->CurrentBlock;

        svo_node* Root = AllocateNode(Tree);

        BuildTree(Tree, SurfaceFn, Root);

        return Tree;
    }
    else
    {
        return nullptr;
    }
}


extern "C" void
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

