#include <assert.h>
#include <stdlib.h>
#include <intrin.h>
#include <queue>
#include <deque>
#include <iostream>
#include <memory>
#include <stack>

#include "sabre.h"
#include "sabre_math.h"
#include "sabre_svo.h"

constexpr u16 SVO_NODE_CHILD_PTR_MSK = 0x7FFFU;

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
FindHighestSetBit(u32 Msk)
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

        svo_block* NextBlk = OldBlk;

        i32 BlksJumped = 0;

        do
        {
            if (FarPtr.BlkOffset < 0)
            {
                NextBlk = NextBlk->Prev;
                --BlksJumped;
            }
            else
            {
                NextBlk = NextBlk->Next;
                ++BlksJumped;
            }
        } while (BlksJumped < FarPtr.BlkOffset);

        return &NextBlk->Entries[ChildPtrBase + ChildOffset];
    }
    else
    {

        return &Tree->CurrentBlock->Entries[ChildPtrBase + ChildOffset];
    }
}

static far_ptr*
AllocateFarPtr(svo_block* const ContainingBlk)
{
    usize NextFarPtrSlot = ContainingBlk->NextFarPtrSlot;
    far_ptr* Ptr = &ContainingBlk->FarPtrs[NextFarPtrSlot];
    ++ContainingBlk->NextFarPtrSlot;

    return Ptr;
}

static void
SetNodeChildPointer(svo_node* Child, svo_block* ChildBlk, svo_block* ParentBlk, svo_node* Parent)
{
    // Compute the child offset from the start of the block
    u16 ChildOffset = (u16)ChildBlk->NextFreeSlot - 1;

    // Extract first 15 bits
    u16 ChildPtrBits = ChildOffset & SVO_NODE_CHILD_PTR_MSK;

    // If the child node is in the same block as the parent node, all we need to do
    // is set the parent's child ptr to the child offset from the beginning of
    // their shared block.
    if (ChildBlk == ParentBlk)
    {
        Parent->ChildPtr = ChildPtrBits;
    }
    else // Otherwise, we need to allocate a far pointer
    {
        far_ptr* FarPtr = AllocateFarPtr(ParentBlk);
        FarPtr->BlkOffset = ChildBlk->Index - ParentBlk->Index;
        FarPtr->NodeOffset = ChildPtrBits;

        // Set the child ptr value to the far bit with the remaining 15 bits
        // set as the index into the far ptrs storage block.
        Parent->ChildPtr = SVO_FAR_PTR_BIT_MASK | u16(ParentBlk->NextFarPtrSlot - 1);
    }
}



static inline vec3
GetOctantCentre(svo_oct Octant, u32 Scale, vec3 ParentCentreP)
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


static svo_block*
AllocateAndLinkNewBlock(svo_block* const OldBlk, svo* const Tree)
{
    // TODO(Liam): Handle failure case
    svo_block* NewBlk = (svo_block*)calloc(1, sizeof(svo_block));
    assert(NewBlk);

    NewBlk->Prev = OldBlk;
    OldBlk->Next = NewBlk;

    NewBlk->Index = OldBlk->Index + 1;

    ++Tree->UsedBlockCount;

    return NewBlk;
}


static inline int
GetAvailableBlockSlots(svo_block* const Blk)
{
    return (int)SVO_ENTRIES_PER_BLOCK - (int)Blk->NextFreeSlot;
}

static svo_node*
AllocateNode(svo_block* const Blk)
{
    if (GetAvailableBlockSlots(Blk) > 0)
    {
        svo_node* Child = &Blk->Entries[Blk->NextFreeSlot];
        ++Blk->NextFreeSlot;

        return Child;
    }
    else
    {
        return nullptr;
    }
}


#if 0
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
#endif


#if 0
static svo_node*
AllocateChildNode(svo_node* Parent, svo_block* ParentBlk, u32 ParentBlkIndex, svo* Tree)
{
    svo_node* CurrentBlk = ParentBlk;

    // Allocate a new child node for this octant.
    // First, attempt to allocate within the same block
    svo_node* Child = AllocateNode(CurrentBlk);

    if (nullptr == Child)
    {
        svo_block* NewBlk = AllocateAndLinkNewBlock(CurrentBlk, Tree);
        Child = AllocateNode(NewBlk);
        CurrentBlk = NewBlk;
    }

    if (0x0000 == Parent->ChildPtr)
    {
        // Didn't move to  new block, no need to 
        // allocate far ptrs
        if (CurrentBlk == CurrentCtx.ParentBlk)
        {
            // Same block; okay to compute offset using the current block
            ptrdiff_t ChildOffset = Child - CurrentBlk->Entries;

            // Extract first 15 bits
            u16 ChildPtrBits = (u16)(ChildOffset) & SVO_NODE_CHILD_PTR_MSK;
            Parent->ChildPtr = ChildPtrBits;
        }
        else // Need a far ptr
        {
            // Compute offset using the new block
            ptrdiff_t ChildOffset = Child - CurrentBlk->Entries;
            // Extract first 15 bits
            u16 ChildPtrBits = (u16)(ChildOffset) & SVO_NODE_CHILD_PTR_MSK;

            // TODO(Liam): Handle failure case
            far_ptr* FarPtr = AllocateFarPtr(ParentBlk);
            FarPtr->BlkOffset = (i32)(CurrentBlk->Index - ParentBlk->Index);
            FarPtr->NodeOffset = ChildPtrBits;

            // Set the child ptr value to the far bit with the remaining 15 bits
            // set as the index into the far ptrs storage block.
            Parent->ChildPtr = SVO_FAR_PTR_BIT_MASK | (ParentBlk->NextFarPtrSlot - 1);
        }
    }
}
#endif

struct q_ctx
{
    svo_node* Node;
    svo_oct   Oct;
    u32       Depth;
    u32       Scale;
    vec3      Centre;
    svo_block* ParentBlk;
};


extern "C" svo*
CreateSparseVoxelOctree(u32 ScaleExponent, u32 MaxDepth, intersector_fn SurfaceFn)
{
    svo* Tree = (svo*)calloc(1, sizeof(svo));
    
    if (Tree)
    {
        Tree->MaxDepth = MaxDepth;
        Tree->ScaleExponent = ScaleExponent;

        // TODO(Liam): Check failure
        // No need to initialise indices, nodes, etc. to zero
        // here because blocks are designed to be fully initialised
        // and working with zeroed elements.
        svo_block* RootBlk = (svo_block*)calloc(1, sizeof(svo_block));
        assert(RootBlk);
        Tree->RootBlock = RootBlk;
        Tree->UsedBlockCount = 1;

        // Allocate the root node
        svo_node* RootNode = &RootBlk->Entries[RootBlk->NextFreeSlot];
        ++RootBlk->NextFreeSlot;

        svo_block* CurrentBlk = Tree->RootBlock;
        
        // Begin building tree
        u32  RootScale = 1 << ScaleExponent;
        vec3 RootCentre = vec3(RootScale >> 1);

        q_ctx RootCtx = { RootNode, OCT_C000, 0, RootScale, RootCentre, Tree->RootBlock };

        std::deque<q_ctx> Queue;
        Queue.push_front(RootCtx);

        while (false == Queue.empty())
        {
            // Retrieve the next node to process from
            // the front of the queue
            q_ctx CurrentCtx = Queue.front();
            Queue.pop_front();

            u32 NextScale = CurrentCtx.Scale >> 1;

            {
                // Process each octant of this node
                for (u32 Oct = 0; Oct < 8; ++Oct)
                {
                    vec3 Rad = vec3(NextScale >> 1);
                    vec3 OctCentre = GetOctantCentre((svo_oct)Oct, NextScale, CurrentCtx.Centre);
                    vec3 OctMin = OctCentre - Rad;
                    vec3 OctMax = OctCentre + Rad;

                    if (SurfaceFn(OctMin, OctMax))
                    {
                        // Node is intersected by the surface
                        if (CurrentCtx.Depth < MaxDepth - 1)
                        {
                            // Need to subdivide
                            SetOctantOccupied((svo_oct)Oct, VOXEL_PARENT, CurrentCtx.Node);

                            // Allocate a new child node for this octant.
                            // First, attempt to allocate within the same block
                            svo_node* Child = AllocateNode(CurrentBlk);

                            if (nullptr == Child)
                            {
                                svo_block* NewBlk = AllocateAndLinkNewBlock(CurrentBlk, Tree);
                                Child = AllocateNode(NewBlk);
                                CurrentBlk = NewBlk;
                            }

                            assert(Child);
                            assert(CurrentBlk);
                            assert(CurrentCtx.ParentBlk);
                            assert(CurrentCtx.Node);

                            if (CurrentCtx.Node->ChildPtr == 0x0000)
                            {
                                SetNodeChildPointer(Child, CurrentBlk, CurrentCtx.ParentBlk, CurrentCtx.Node);
                            }

                            if (Child)
                            {
                                q_ctx ChildCtx = { Child, (svo_oct)Oct, CurrentCtx.Depth + 1, NextScale, OctCentre, CurrentBlk };
                                Queue.push_front(ChildCtx);
                            }
                        }
                        else
                        {
                            SetOctantOccupied((svo_oct)Oct, VOXEL_LEAF, CurrentCtx.Node);
                        }
                    }
                }
            }
        }

        return Tree;
    }
    else
    {
        return nullptr;
    }
}

// InsertVoxel {{{
#if 0
extern "C" void
InsertVoxel(svo* Svo, vec3 P, u32 VoxelScale)
{
    // FIXME(Liam): What to do if P is outside root bounds??
    u32 SvoScale = 1 << (Svo->ScaleExponent - 1);

    vec3 ParentCentreP = vec3(SvoScale);
    u32 CurrentOct = GetOctantForPosition(P, ParentCentreP);

    bool CreatedParent = false;

    // Initialised to root node
    // TODO(Liam): Switch to a forward-linked list??
    svo_node* ParentNode = &Svo->RootBlock->Entries[0]; 
    //svo_block* ParentBlk = Svo->RootBlock;

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
                ParentCentreP = GetOctantCentre((svo_oct)CurrentOct, Scale >> 1, ParentCentreP);
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
                svo_node* NewParent = nullptr;
                svo_node* FirstChild = nullptr;

                u32 ChildCount = CountSetBits(NonLeafChildMsk);

                for (u32 ChildIndex = 0; ChildIndex < ChildCount; ++ChildIndex)
                {
                    u32 ChildOct = FindHighestSetBit(NonLeafChildMsk);

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
                    // FIXME(Liam): Re-instate!!
                    //SetNodeChildPointer(ChildPtr, false, Svo, ParentNode);
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
#endif
// }}}


extern "C" void
DeleteSparseVoxelOctree(svo* Tree)
{
    svo_block* CurrentBlk = Tree->RootBlock;
    for (u32 BlkIndex = 0; BlkIndex < Tree->UsedBlockCount; ++BlkIndex)
    {
        free(CurrentBlk);

        CurrentBlk = CurrentBlk->Next;
    }

    free(Tree);
}

