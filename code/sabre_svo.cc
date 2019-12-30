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
        OutParentNode->ChildPtr = ChildPtr & 0x7FFF;

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
ProcessOctant(svo* Tree, svo_oct Octant, u32 Scale, intersector_fn Surface, svo_node* ContainingNode, vec3 OctantCentre)
{
    //u32 Scale = 1 << (Tree->MaxDepth - Depth);
    vec3 Radius = vec3(Scale);
    vec3 OctMin = OctantCentre - Radius;
    vec3 OctMax = OctantCentre + Radius;
    if (Surface(OctMin, OctMax))
    {
        SetOctantOccupied(Octant, VOXEL_PARENT, ContainingNode);

        // Allocate a new child for this node
        svo_node* Child = AllocateNode(Tree);

        return Child;
    }
    else
    {
        SetOctantOccupied(Octant, VOXEL_LEAF, ContainingNode);

        return nullptr;
    }
}


struct node_context
{
    svo_node* Node;
    svo_oct   Oct;
    u32       Depth;
    vec3      Centre;
};

static void
BuildTree(svo* Tree, intersector_fn Surface, svo_node* Root)
{
    u32 CurrentDepth = 0;
    
    std::queue<node_context> Queue;

    node_context RootContext = { Root, OCT_C000, 0, vec3(0, 0, 0) };
    Queue.push(RootContext);

    while (false == Queue.empty())
    {
        node_context CurrentContext = Queue.front();

        printf("\nBEGIN L(%d)\n", CurrentContext.Depth);

        u32 Scale = 1 << (Tree->MaxDepth - CurrentContext.Depth);

        for (u32 Oct = 0; Oct < 8; ++Oct)
        {
            vec3 OctantCentre = GetNodeCentrePosition((svo_oct) Oct, Scale, CurrentContext.Centre);

            svo_node* MaybeChild = ProcessOctant(Tree, (svo_oct) Oct, Scale, Surface, CurrentContext.Node, OctantCentre);

            // Check if octant needs to be subdivided
            if (MaybeChild && (CurrentDepth < Tree->MaxDepth))
            {
                node_context ChildContext = { MaybeChild, (svo_oct)Oct, CurrentContext.Depth + 1, OctantCentre };

                Queue.push(ChildContext);
            }
        }
        
        Queue.pop(); // Nonsense
    }
}


#if 0
static void
BuildTree(svo* Tree, intersector_fn Surface, svo_node* Root)
{
    u32 Depth = 0;

    vec3 ParentCentre = vec3(0, 0, 0);

    svo_node* ContainingNode = Root;


    svo_node* Queue[8];
    usize QueueFront = 0;
    usize QueueBack = 0;

    while (Depth < Tree->MaxDepth)
    {
        u32 Scale = 1 << (Tree->MaxDepth - Depth);
        vec3 Radius = vec3(Scale);

        for (u32 Octant = 0; Octant < 8; ++Octant)
        {
            vec3 OctantCentre = GetNodeCentrePosition((svo_oct)Octant, Scale, ParentCentre);

            vec3 OctMin = OctantCentre - Radius;
            vec3 OctMax = OctantCentre + Radius;

            if (Surface(OctMin, OctMax))
            {
                SetOctantOccupied((svo_oct) Octant, VOXEL_PARENT, ContainingNode);

                // Allocate a child node for this octant
                svo_node* OctantNode = AllocateNode(Tree);

                

                
            }
            else
            {
                SetOctantOccupied((svo_oct) Octant, VOXEL_LEAF, ContainingNode);
            }
        }

        ++Depth;
    }
}
#endif

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
            SetOctantOccupied(Octant, VOXEL_PARENT, ParentNode);

            // Allocate a new "sub-parent"
            svo_node* Child = AllocateNode(Tree);

            for (u32 ChildOctant = 0; ChildOctant < 8; ++ChildOctant)
            {
                InsertNode(Tree, Surface, Depth + 1, (svo_oct) ChildOctant, Child, NodeCentreP);
            }
        }
        else
        {
            SetOctantOccupied(Octant, VOXEL_LEAF, ParentNode);
        }
    }
    else
    {
        SetOctantOccupied(Octant, VOXEL_LEAF, ParentNode);
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

        svo_node* Root = AllocateNode(Tree);

        // Add the root node to the tree. This kicks off the recursive
        // tree building.
        //InsertNode(Tree, SurfaceFn, 0, OCT_C000, TreeParentP, Root);

        BuildTree(Tree, SurfaceFn, Root);
        //InsertNode(Tree, SurfaceFn, 0, OCT_C000, Root, TreeParentP);

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

