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
SetNodeChildPointer(u16 ChildPtr, bool InNewBlock, svo* Tree, svo_node* OutParentNode, svo_block* ParentBlk)
{
    // If the child is in a new block, we need to allocate and assign a 
    // far ptr for the old block.
    if (InNewBlock)
    {
        // The 'new' block is the current block; set the far-ptr containing block
        // to be the previous
        
        svo_block* ContainingBlk = Tree->CurrentBlock->Prev;
        assert(ContainingBlk != nullptr);

        assert(ParentBlk != nullptr);
        far_ptr* FarPtr = AllocateFarPtr(ParentBlk);

        // One block forward
        FarPtr->BlkOffset = 1;
        FarPtr->NodeOffset = ChildPtr;

        // TODO(Liam): Check type conversions here
        OutParentNode->ChildPtr = (u16)(FarPtr - ContainingBlk->FarPtrs);
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

static inline vec3
GetOctCentreP(svo_oct Octant, u32 Scale, vec3 ParentCentreP)
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

static svo_block*
AllocateAndLinkNewBlock(svo_block* const OldBlk, svo* const Tree)
{
    // TODO(Liam): Handle failure case
    svo_block* NewBlk = (svo_block*)calloc(1, sizeof(svo_block));

    NewBlk->Prev = OldBlk;
    OldBlk->Next = NewBlk;

    ++Tree->UsedBlockCount;

    return NewBlk;
}


static inline bool
NeedMoreBlocks(svo* const Tree)
{
    return SVO_ENTRIES_PER_BLOCK <= Tree->CurrentBlock->NextFreeSlot;
}

static inline bool
BlockHasSlotsAvailable(svo_block* const Blk)
{
    return SVO_ENTRIES_PER_BLOCK > Blk->NextFreeSlot;
}

static svo_node*
AllocateNode(svo_block* const Blk)
{
    if (BlockHasSlotsAvailable(Blk))
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


// TODO(Liam): SLATED FOR REMOVAL
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


extern "C" svo*
CreateSparseVoxelOctree(u32 ScaleExponent, u32 MaxDepth, intersector_fn SurfaceFn)
{
    svo* Tree = (svo*)calloc(1, sizeof(svo));
    
    if (Tree)
    {
        Tree->MaxDepth = MaxDepth;
        Tree->ScaleExponent = ScaleExponent;

        // TODO(Liam): Check failure
        svo_block* RootBlk = (svo_block*)calloc(1, sizeof(svo_block));
        Tree->RootBlock = RootBlk;
        Tree->UsedBlockCount = 1;

        // Allocate the root node
        svo_node* RootNode = &RootBlk->Entries[RootBlk->NextFreeSlot];
        ++RootBlk->NextFreeSlot;

        svo_block* CurrentBlk = RootBlk;
        
        // Begin building tree
        u32 RootScale = 1 << ScaleExponent;
        vec3 RootCentre = vec3(RootScale >> 1);

        struct q_ctx
        {
            svo_node* Node;
            svo_oct   Oct;
            u32       Depth;
            u32       Scale;
            vec3      Centre;
            i32       ParentBlkIndex;
            svo_block* ParentBlk;
        };

        // Index of the block holding the node which is the
        // parent of the current level we're examining.
        i32 ParentBlkIndex = 0;

        q_ctx RootCtx = { RootNode, OCT_C000, 0, RootScale, RootCentre, ParentBlkIndex, RootBlk };

        std::deque<q_ctx> Queue;
        Queue.push_front(RootCtx);
        int N = 0;

        while (false == Queue.empty() || N > 100)
        {
            ++N;

            // Retrieve the next node to process from
            // the front of the queue
            q_ctx CurrentCtx = Queue.front();
            Queue.pop_front();

            u32 NextScale = CurrentCtx.Scale >> 1;

            // Process each octant of this node
            for (u32 Oct = 0; Oct < 8; ++Oct)
            {
                vec3 Rad = vec3(NextScale >> 1);
                vec3 OctCentre = GetOctCentreP((svo_oct)Oct, NextScale, CurrentCtx.Centre);
                vec3 OctMin = OctCentre - Rad;
                vec3 OctMax = OctCentre + Rad;

                if (SurfaceFn(OctMin, OctMax))
                {
                    // Node is intersected by the surface
                    if (CurrentCtx.Depth < MaxDepth - 1)
                    {
                        i32 CurrentBlkIndex = CurrentCtx.ParentBlkIndex;
                        // Need to subdivide
                        SetOctantOccupied((svo_oct)Oct, VOXEL_PARENT, CurrentCtx.Node);

                        // Allocate a new child node for this octant.
                        // First, attempt to allocate within the same block
                        svo_node* Child = AllocateNode(CurrentBlk);
                        printf("%d %d\n", CurrentCtx.Depth, Oct);
                        DEBUGPrintVec3(OctCentre); printf("\n");

                        if (nullptr == Child)
                        {
                            svo_block* NewBlk = AllocateAndLinkNewBlock(CurrentBlk, Tree);
                            Child = AllocateNode(NewBlk);
                            CurrentBlk = NewBlk;
                            ++CurrentBlkIndex;
                        }

                        assert(Child);
                        assert(CurrentBlk);
                        assert(CurrentCtx.ParentBlk);
                        assert(CurrentCtx.Node);


                        if (CurrentCtx.Node->ChildPtr == 0x0000)
                        {
                            // Didn't move to  new block, no need to 
                            // allocate far ptrs
                            if (CurrentBlk == CurrentCtx.ParentBlk)
                            {
                                    // Same block; okay to compute offset using the current block
                                    ptrdiff_t ChildOffset = Child - CurrentBlk->Entries;

                                    // Extract first 15 bits
                                    u16 ChildPtrBits = (u16)(ChildOffset) & 0x7FFFU;
                                    CurrentCtx.Node->ChildPtr = ChildPtrBits;
                            }
                            else // Need a far ptr
                            {
                                // Compute offset using the new block
                                ptrdiff_t ChildOffset = Child - CurrentBlk->Entries;
                                // Extract first 15 bits
                                u16 ChildPtrBits = (u16)(ChildOffset) & 0x7FFF;

                                // TODO(Liam): Handle failure case
                                far_ptr* FarPtr = AllocateFarPtr(CurrentCtx.ParentBlk);
                                FarPtr->BlkOffset = (i32)((Tree->UsedBlockCount - 1) - CurrentCtx.ParentBlkIndex);
                                FarPtr->NodeOffset = ChildPtrBits;

                                // Set the child ptr value to the far bit with the remaining 15 bits
                                // set as the index into the far ptrs storage block.
                                CurrentCtx.Node->ChildPtr = SVO_FAR_PTR_BIT_MASK | (CurrentCtx.ParentBlk->NextFarPtrSlot - 1);
                            }
                        }

                        if (Child)
                        {
                            q_ctx ChildCtx = { Child, (svo_oct)Oct, CurrentCtx.Depth + 1, NextScale, OctCentre, CurrentBlkIndex, CurrentBlk };
                            Queue.push_front(ChildCtx);
                        }
                    }
                    else
                    {
                        SetOctantOccupied((svo_oct)Oct, VOXEL_LEAF, CurrentCtx.Node);
                    }
                }
            }

            //Queue.pop();
        }

        return Tree;

    }
    else
    {
        return nullptr;
    }
}

// InsertVoxel {{{
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
// }}}


extern "C" void
DeleteSparseVoxelOctree(svo* Tree)
{
    svo_block* CurrentBlk = Tree->RootBlock;
    for (u32 BlockIndex = 0; BlockIndex < Tree->UsedBlockCount; ++BlockIndex)
    {
        CurrentBlk = CurrentBlk->Next;
        free(CurrentBlk);
    }

    free(Tree);
}

