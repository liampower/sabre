#include <assert.h>
#include <stdlib.h>
#include <intrin.h>
#include <queue>
#include <iostream>
#include <memory>

#include "sabre.h"
#include "sabre_math.h"
#include "sabre_svo.h"

struct poly
{
    u8  VertexCount;
    u32 VertexIndices[];
};

enum svo_voxel_type
{
    VOXEL_PARENT = 0U,
    VOXEL_LEAF   = 1U
};

static inline u32
CountSetBits(u32 Msk)
{
#if defined(_MSC_VER)
    return (u32)(__popcnt(Msk));
#else
    return (u32)(__builtin_popcount(Msk));
#endif
}

static inline u32
FindMSB(u32 Msk)
{
#if defined(_MSC_VER)
    unsigned long MSB;
    _BitScanReverse(&MSB, Msk);

    return (u32)MSB;
#else
    return __builtin_ctz(Msk);
#endif
}

static inline bool
IsOctantOccupied(svo_node* ContainingNode, svo_oct Oct)
{
    return ContainingNode->OccupiedMask & (1U << (u32)Oct);
}

static inline bool
IsOctantLeaf(svo_node* ContainingNode, svo_oct Oct)
{
    return ContainingNode->LeafMask & (1U << (u32)Oct);
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

#if 0
static inline usize
GetBlockFreeSlotCount(svo_block* Blk)
{
    return (SVO_ENTRIES_PER_BLOCK - Blk->NextFreeSlot)
}

static inline svo_node*
AllocateChildNode(svo_node* Parent, svo_block* ParentBlk, svo* Tree)
{
    usize FreeSlotCount = GetBlockFreeSlotCount(ParentBlk);

    // No more space in parent block at all.
    // Need to allocate an entirely new block
    if (FreeSlotCount == 0)
    {
        svo_block* NextBlk = AllocateNewBlock2(ParentBlk, Tree);

        svo_node* ChildNode = &NextBlk->Entries[NextBlk->NextFreeSlot];
        ++NextBlk->NextFreeSlot;

        // Setup a far ptr
        far_ptr* FarPtr = AllocateFarPtr2(ParentBlk);
        FarPtr.Block = NextBlk;
    }
}
#endif


// TODO(Liam) Running into a *lot* of problems with passing the blocks around.
// May need to reconsider design where we split the world up into uniform octree
// blocks, each with a separate SVO index.
static inline svo_node*
GetNodeChild(svo_node* Parent, svo* Tree, svo_oct ChildOct)
{
    u32 NonLeafChildren = Parent->OccupiedMask & (~Parent->LeafMask);
    u32 SetBitsBehindOctIdx = (1U << (u32)ChildOct) - 1;
    u32 ChildOffset = CountSetBits(NonLeafChildren & SetBitsBehindOctIdx); 

    // Extract the actual offset bits from the child pointer (the topmost
    // bit is reserved for the far flag).
    u32 ChildPtrBase = Parent->ChildPtr & 0x7FFF;

    // Check if the node references a child outside this block.
    if (Parent->ChildPtr & SVO_FAR_PTR_BIT_MASK)
    {
        svo_block* OldBlk = Tree->CurrentBlock->Prev;

        // Extract the far ptr from this block
        far_ptr FarPtr = OldBlk->FarPtrs[ChildPtrBase];

        svo_block* NextBlk = FarPtr.Block;

        return &NextBlk->Entries[ChildPtrBase + ChildOffset];
    }
    else
    {

        return &Tree->CurrentBlock->Entries[ChildPtrBase + ChildOffset];
    }
}

#if 0
static inline far_ptr*
AllocateFarPtr2(svo_block* ContainingBlk)
{
    usize NextFarPtrSlot = ContainingBlk->NextFarPtrSlot;
    if (NextFarPtrSlot < SVO_FAR_PTRS_PER_BLOCK)
    {
        far_ptr* Ptr = &ContainingBlk->FarPtrs[NextFreeSlot];
        ++ContainingBlk->NextFarPtrSlot;

        return Ptr;
    }
    else
    {
        return nullptr;
    }
}
#endif

static far_ptr*
AllocateFarPtr(svo* const Tree)
{
    usize NextFreeSlot = Tree->CurrentBlock->NextFarPtrSlot;
    if (NextFreeSlot < SVO_FAR_PTRS_PER_BLOCK)
    {
        far_ptr* Ptr = &Tree->CurrentBlock->FarPtrs[NextFreeSlot];
        ++Tree->CurrentBlock->NextFarPtrSlot;

        return Ptr;
    }
    else
    {
        return nullptr;
    }
}

static void
SetNodeChildPointer(u16 ChildPtr, bool InNewBlock, svo* Tree, svo_node* OutParentNode)
{
    // If the child is in a new block, we need to allocate and assign a 
    // far ptr for the old block.
    if (InNewBlock)
    {
        far_ptr* FarPtr = AllocateFarPtr(Tree);
        FarPtr->Block = Tree->CurrentBlock;
        FarPtr->Offset = ChildPtr;

        // TODO(Liam): Check type conversions here
        OutParentNode->ChildPtr = (u16)(FarPtr - Tree->CurrentBlock->FarPtrs);
		OutParentNode->ChildPtr |= SVO_FAR_PTR_BIT_MASK;
    }
    else
    {
        // Extract first 15 bits
        u16 ChildPtrBits = ChildPtr & 0x7FFF;

        OutParentNode->ChildPtr = ChildPtrBits;
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

static inline svo_oct
GetOctantForPosition(vec3 P, vec3 ParentCentreP)
{
    uvec3 G = uvec3(GreaterThan(P, ParentCentreP));

    return (svo_oct) (G.X + G.Y*2 + G.Z*4);
}

#if 0
static svo_block
AllocateNewBlock2(svo_block* const PrevBlk, svo* const Tree)
{
    svo_block* NextBlk = (svo_block*) calloc(1, sizeof(svo_block));
    PrevBlk->Next = NextBlk;
    NextBlk->Prev = PrevBlk;

    ++Tree->UsedBlockCount;

    return NextBlk;
}
#endif

static void
AllocateNewBlock(svo* const Tree)
{
    svo_block* OldBlock = Tree->CurrentBlock;
    svo_block* NewBlock = (svo_block*) calloc(1, sizeof(svo_block));
 
    // Link blocks together
    NewBlock->Prev = OldBlock;
    OldBlock->Next = NewBlock;

    Tree->CurrentBlock = NewBlock;
    ++Tree->UsedBlockCount;
}


static inline bool
NeedMoreBlocks(svo* const Tree)
{
    return SVO_ENTRIES_PER_BLOCK <= Tree->CurrentBlock->NextFreeSlot;
}


// TODO(Liam): Garbo api here
static svo_node*
AllocateNode(svo* const Tree, bool* NewBlockOut)
{
    if (NeedMoreBlocks(Tree))
    {
        AllocateNewBlock(Tree);
        *NewBlockOut = true;
    }
    else
    {
        *NewBlockOut = false;
    }

    svo_node* Child = &Tree->CurrentBlock->Entries[Tree->CurrentBlock->NextFreeSlot];
    ++Tree->CurrentBlock->NextFreeSlot;

    return Child;
}


static void
PushNode(svo* const Tree, svo_node Node)
{
    if (NeedMoreBlocks(Tree))
    {
        AllocateNewBlock(Tree);
    }

    Tree->CurrentBlock->Entries[Tree->CurrentBlock->NextFreeSlot] = Node;
    ++Tree->CurrentBlock->NextFreeSlot;
}

struct processed_oct
{
    svo_node* MaybeChild;
    bool      NewBlock;
};

static processed_oct
ProcessOctant(svo* Tree, svo_oct Octant, u32 Scale, u32 Depth, intersector_fn Surface, svo_node* ContainingNode, vec3 OctantCentre, vec3 ParentCentre)
{
    processed_oct Result = { nullptr, false };

    vec3 Radius = vec3(Scale >> 1);
    vec3 OctMin = OctantCentre - Radius;
    vec3 OctMax = OctantCentre + Radius;

    if (Surface(OctMin, OctMax))
    {
        if (Depth < Tree->MaxDepth - 1)
        {
            SetOctantOccupied(Octant, VOXEL_PARENT, ContainingNode);

            // Allocate a new child for this node
            bool NewBlock;
            svo_node* Child = AllocateNode(Tree, &NewBlock);

            Result.MaybeChild = Child;
            Result.NewBlock = NewBlock;

            return Result;
        }
        else
        {
            SetOctantOccupied(Octant, VOXEL_LEAF, ContainingNode);
        }

    }

    return Result;
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

        //printf("\nBEGIN L(%d)\n", CurrentContext.Depth);

        u32 Scale = CurrentContext.Scale >> 1;

        //printf("SCALE %u   CTR (%f %f %f)\n", Scale, CurrentContext.Centre.X, CurrentContext.Centre.Y, CurrentContext.Centre.Z);
        for (u32 Oct = 0; Oct < 8; ++Oct)
        {
            vec3 OctantCentre = GetNodeCentrePosition((svo_oct) Oct, Scale, CurrentContext.Centre);

            // TODO(Liam): Cleanup here
            processed_oct Result = ProcessOctant(Tree, (svo_oct) Oct, Scale, CurrentContext.Depth, Surface, CurrentContext.Node, OctantCentre, CurrentContext.Centre);

            // Check if octant needs to be subdivided
            if (Result.MaybeChild && CurrentContext.Depth < Tree->MaxDepth - 1)
            {
                node_context ChildContext = { Result.MaybeChild, (svo_oct)Oct, CurrentContext.Depth + 1, Scale, OctantCentre };

                // If the parent node's child ptr hasn't already been set, we can set it here.
                // If it's already been set, that means that some octant before this one has 
                // already set it, so we can assume that we follow that one in sequential memory
                // order.
                if (CurrentContext.Node->ChildPtr == 0x0000)
                {
                    if (Result.NewBlock)
                    {
                        ptrdiff_t ChildOffset = 0; // TODO(Liam) ??? 
                        SetNodeChildPointer((u16) ChildOffset, Result.NewBlock, Tree, CurrentContext.Node);

                    }
                    else
                    {
                        ptrdiff_t ChildOffset = Result.MaybeChild - Tree->CurrentBlock->Entries;
                        SetNodeChildPointer((u16) ChildOffset, Result.NewBlock, Tree, CurrentContext.Node);
                    }
                }

                Queue.push(ChildContext);
            }
        }
        
        Queue.pop();
    }
}


static void
BuildTree2(svo* Tree, intersector_fn Surface, svo_node* RootNode, u32 RootScale)
{
    std::queue<node_context> Queue;

    vec3 RootCentre = vec3(RootScale >> 1);

    node_context RootContext = { RootNode, OCT_C000, 0, RootScale, RootCentre };
    u32 CurrentScale = RootScale >> 1;

    vec3 ParentCentre = RootCentre;

    u32 CurrentDepth = 1;
    for (u32 Oct = 0; Oct < 8; ++Oct)
    {
        vec3 Rad = vec3(CurrentScale >> 1);
        vec3 OctMin = ParentCentre - Rad;
        vec3 OctMax = ParentCentre + Rad;

        if (Surface(OctMin, OctMax))
        {
            if (CurrentDepth < Tree->MaxDepth - 1)
            {
                SetOctantOccupied((svo_oct)Oct, VOXEL_PARENT, RootNode);

                // Allocate child
                bool NewBlk = false;
                svo_node* ChildNode = AllocateNode(Tree, &NewBlk);

            }
        }
        else
        {
            
        }

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

        // NOTE(Liam): No need to further initialise the block's
        // NextFreeSlot and NextFarPtrSlot fields because we are 
        // zero-initialising with calloc.
        Tree->CurrentBlock = (svo_block*) calloc(1, sizeof(svo_block));
        Tree->UsedBlockCount = 1;
        Tree->RootBlock = Tree->CurrentBlock;

        bool _Ignored;
        svo_node* Root = AllocateNode(Tree, &_Ignored);

        u32 RootScale = 1 << Tree->ScaleExponent;
        BuildTree(Tree, SurfaceFn, Root);

        return Tree;
    }
    else
    {
        return nullptr;
    }
}


extern "C" void
InsertVoxel(svo* Svo, vec3 P, u32 VoxelScale)
{
    // FIXME(Liam): What do do if P is outside root bounds??
    u32 SvoScale = 1 << (Svo->ScaleExponent - 1);

    vec3 ParentCentreP = vec3(SvoScale);
    u32 CurrentOct = GetOctantForPosition(P, ParentCentreP);

    bool CreatedParent = false;

    // Initialised to root node
    // TODO(Liam): Switch to a forward-linked list??
    svo_node* ParentNode = &Svo->RootBlock->Entries[0]; 

    for (u32 Scale = SvoScale; Scale >= VoxelScale; Scale >>= 1)
    {
        if (Scale <= VoxelScale)
        {
            SetOctantOccupied((svo_oct)CurrentOct, VOXEL_LEAF, ParentNode); 
        }

        // Check if the current octant is already occupied
        if (IsOctantOccupied(ParentNode, (svo_oct)CurrentOct))
        {
            if (IsOctantLeaf(ParentNode, (svo_oct)CurrentOct))
            {
                return;
            }
            else // This node is a parent
            {
                // Descend further into tree.
                ParentCentreP = GetNodeCentrePosition((svo_oct)CurrentOct, Scale >> 1, ParentCentreP);
                CurrentOct = GetOctantForPosition(P, ParentCentreP);

                ParentNode = GetNodeChild(ParentNode, Svo, (svo_oct)CurrentOct);

                continue;
            }
        }
        else
        {
            // ... if not, we need to mark it as occupied and
            // allocate a child ptr.
            SetOctantOccupied((svo_oct)CurrentOct, VOXEL_PARENT, ParentNode);

            // Do not need to copy children if we created the parent (because all of the 
            // enclosed space is guaranteed to have no siblings)
            //
            // Copy child octants to new memory (ugh)

            if (CreatedParent)
            {
                bool NewBlk;
                ParentNode = AllocateNode(Svo, &NewBlk);
                continue;
            }
            else
            {
                // Identifies octants that are occupied but not leaves.
                u32 NonLeafChildMsk = ParentNode->OccupiedMask & ~(ParentNode->LeafMask);
                svo_node* NewParent;
                svo_node* FirstChild = nullptr;

                u32 ChildCount = CountSetBits(NonLeafChildMsk);

                for (u32 ChildIndex = 0; ChildIndex < ChildCount; ++ChildIndex)
                {
                    u32 ChildOct = (u32)FindMSB(NonLeafChildMsk);

                    if (ChildOct == CurrentOct)
                    {
                        // Allocate a new child node for this octant and 
                        // assign the current parent to it.
                        bool NewBlock;
                        NewParent = AllocateNode(Svo, &NewBlock);

                        if (! FirstChild) FirstChild = NewParent;
                    }
                    else
                    {
                        u32 SetBitsBehindOctIdx = (1 << ChildOct) - 1;
                        u32 ChildOffset = CountSetBits(NonLeafChildMsk & SetBitsBehindOctIdx);

                        // FIXME(Liam): Fix to work with block memory
                        svo_node* SiblingNode = &Svo->CurrentBlock->Entries[ParentNode->ChildPtr + ChildOffset];

                        // Copy the old node to a new memory region
                        PushNode(Svo, *SiblingNode);

                        if (! FirstChild) FirstChild = SiblingNode;
                    }

                    // Clear this oct from the mask
                    NonLeafChildMsk &= ~(1U << ChildOct);
                }


                // Set the parent's Child ptr to point to the new node.
                if (FirstChild)
                {
                    // FIXME(Liam): Broken for multi-blocks
                    u16 ChildPtr = (u16)(FirstChild - ParentNode);
                    SetNodeChildPointer(ChildPtr, false, Svo, ParentNode);
                }

                if (NewParent)
                {
                    ParentNode = NewParent;
                }

                CreatedParent = true;

            }
        }
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

