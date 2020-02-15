#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <intrin.h>

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

static inline u32
FindLowestSetBit(u32 Msk)
{
#if defined(_MSC_VER)
    unsigned long LSB;
    _BitScanForward(&LSB, Msk);

    return (u32)LSB;
#else
    return __builtin_clz(Msk);
#endif
}

static inline u32
GetChildCount(svo_node* Parent)
{
    return CountSetBits(Parent->OccupiedMask & (~Parent->LeafMask));
}

static inline int
GetFreeSlotCount(svo_block* const Blk)
{
    return (int)SVO_ENTRIES_PER_BLOCK - (int)Blk->NextFreeSlot;
}

static inline bool
HasFarChildren(svo_node* Node)
{
    return Node->ChildPtr & SVO_FAR_PTR_BIT_MASK;
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


static inline svo_node*
PushNode(svo_block* Blk, svo_node Node)
{
    if (GetFreeSlotCount(Blk) > 0)
    {
        svo_node* NewNode = &Blk->Entries[Blk->NextFreeSlot];
        *NewNode = Node;
        ++Blk->NextFreeSlot;

        return NewNode;
    }
    else
    {
        return nullptr;
    }
}

// TODO(Liam) Running into a *lot* of problems with passing the blocks around.
// May need to reconsider design where we split the world up into uniform octree
// blocks, each with a separate SVO index.
#if 0
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
        svo_block* OldBlk = Tree->LastBlock->Prev;

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

        return &Tree->LastBlock->Entries[ChildPtrBase + ChildOffset];
    }
}
#endif

static inline u32
GetNodeChildOffset(svo_node* Parent, svo_oct ChildOct)
{
    u32 NonLeafChildren = Parent->OccupiedMask & (~Parent->LeafMask);
    u32 SetBitsBehindOctIdx = (1U << (u32)ChildOct) - 1;
    u32 ChildOffset = CountSetBits(NonLeafChildren & SetBitsBehindOctIdx); 

    return ChildOffset;
}


static svo_node*
GetNodeChild(svo_node* Parent, svo_block* ParentBlk, svo_oct ChildOct)
{
    u32 ChildOffset = GetNodeChildOffset(Parent, ChildOct);

    u32 ChildPtrBase = Parent->ChildPtr & SVO_NODE_CHILD_PTR_MSK;

    if (HasFarChildren(Parent))
    {
        far_ptr* ParentFarPtr = &ParentBlk->FarPtrs[ChildPtrBase];
        svo_block* ChildBlk = ParentBlk;

        i32 BlksJumped = 0;
        // TODO(Liam): Clean up signed/unsigned mismatches here
        i32 BlksToJump = ParentFarPtr->BlkOffset + (i32)((ChildOffset + ParentFarPtr->NodeOffset) / SVO_ENTRIES_PER_BLOCK);

        // Skip to the first child's block.
        while (BlksJumped != BlksToJump)
        {
            if (BlksToJump < 0)
            {
                ChildBlk = ChildBlk->Prev;
                --BlksJumped;
            }
            else
            {
                ChildBlk = ChildBlk->Next;
                ++BlksJumped;
            }
        }

        return &ChildBlk->Entries[ParentFarPtr->NodeOffset + ChildOffset];
    }
    else
    {
        u32 ChildIndex = ChildPtrBase + ChildOffset;
        i32 BlksToJump = (i32)((ChildPtrBase + ChildOffset) / SVO_ENTRIES_PER_BLOCK);
        i32 BlksJumped = 0;
        svo_block* ChildBlk = ParentBlk;

        // Skip to the first child's block.
        while (BlksJumped != BlksToJump)
        {
            if (BlksToJump < 0)
            {
                ChildBlk = ChildBlk->Prev;
                --BlksJumped;
            }
            else
            {
                ChildBlk = ChildBlk->Next;
                ++BlksJumped;
            }

            ChildIndex -= SVO_ENTRIES_PER_BLOCK;
        }

        return &ChildBlk->Entries[ChildIndex];
    }
}

static far_ptr*
AllocateFarPtr(svo_block* const ContainingBlk)
{
    usize NextFarPtrSlot = ContainingBlk->NextFarPtrSlot;
    far_ptr* Ptr = &ContainingBlk->FarPtrs[NextFarPtrSlot];
    ++ContainingBlk->NextFarPtrSlot;

    assert(Ptr->NodeOffset == 0);
    assert(Ptr->BlkOffset == 0);

    return Ptr;
}

static void
SetNodeChildPointer(svo_node* Child, svo_block* ChildBlk, svo_block* ParentBlk, svo_node* Parent)
{
    // Compute the child offset from the start of the block
    u16 ChildOffset = (u16)(Child - ChildBlk->Entries);

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


// TODO(Liam): Do we *really* need the dependency on `svo` here?
// Seems like we could just remove UsedBlockCount altogether and
// rely on relative positioning for blocks.
static svo_block*
AllocateAndLinkNewBlock(svo_block* const OldBlk, svo* const Tree)
{
    // TODO(Liam): Handle failure case
    svo_block* NewBlk = (svo_block*)calloc(1, sizeof(svo_block));
    assert(NewBlk);
    assert(OldBlk->Next == nullptr);

    NewBlk->Prev = OldBlk;
    OldBlk->Next = NewBlk;

    NewBlk->Index = OldBlk->Index + 1;

    ++Tree->UsedBlockCount;
    Tree->LastBlock = NewBlk;

    return NewBlk;
}


static svo_node*
AllocateNode(svo_block* const Blk)
{
    if (GetFreeSlotCount(Blk) > 0)
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


static void
BuildSubOctreeRecursive(svo_node* Parent, svo* Tree, svo_oct RootOct, u32 Depth, u32 Scale, svo_block* ParentBlk, vec3 Centre, intersector_fn SurfaceFn)
{
    struct node_child
    {
        svo_oct Oct;
        vec3    Centre;
        svo_node* Node;
        svo_block* Blk;
    };

    u32 NextScale = Scale >> 1;
    svo_block* CurrentBlk = Tree->LastBlock;

    node_child Children[8];
    u32 LastChildIndex = 0;

    vec3 Radius = vec3(NextScale >> 1);
    for (u32 Oct = 0; Oct < 8; ++Oct)
    {
        vec3 OctCentre = GetOctantCentre((svo_oct)Oct, NextScale, Centre);
        vec3 OctMin = OctCentre - Radius;
        vec3 OctMax = OctCentre + Radius;

        if (SurfaceFn(OctMin, OctMax))
        {
            if (Depth < Tree->MaxDepth - 1)
            {
                // Need to subdivide
                SetOctantOccupied((svo_oct)Oct, VOXEL_PARENT, Parent);

                // Allocate a new child node for this octant.
                // First, attempt to allocate within the same block
                svo_node* Child = AllocateNode(Tree->LastBlock);

                if (nullptr == Child)
                {
                    svo_block* NewBlk = AllocateAndLinkNewBlock(Tree->LastBlock, Tree);
                    Child = AllocateNode(NewBlk);
                    CurrentBlk = NewBlk;
                }

                if (0x0000 == Parent->ChildPtr)
                {
                    SetNodeChildPointer(Child, Tree->LastBlock, ParentBlk, Parent);
                }

                if (Child)
                {
                    Children[LastChildIndex] = { (svo_oct)Oct, OctCentre, Child, Tree->LastBlock };
                    ++LastChildIndex;
                }
            }
            else
            {
                SetOctantOccupied((svo_oct)Oct, VOXEL_LEAF, Parent);
            }
        }
    }

    for (u32 ChildIndex = 0; ChildIndex < LastChildIndex; ++ChildIndex)
    {
        node_child Child = Children[ChildIndex];

        BuildSubOctreeRecursive(Child.Node,
                                Tree,
                                Child.Oct,
                                Depth + 1,
                                NextScale,
                                Child.Blk,
                                Child.Centre,
                                SurfaceFn);
    }
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
        // No need to initialise indices, nodes, etc. to zero
        // here because blocks are designed to be fully initialised
        // and working with zeroed elements.
        svo_block* RootBlk = (svo_block*)calloc(1, sizeof(svo_block));
        assert(RootBlk);
        Tree->RootBlock = RootBlk;
        Tree->LastBlock = RootBlk;
        Tree->UsedBlockCount = 1;

        // Allocate the root node
        svo_node* RootNode = &RootBlk->Entries[RootBlk->NextFreeSlot];
        ++RootBlk->NextFreeSlot;
        
        // Begin building tree
        u32  RootScale = 1 << ScaleExponent;
        vec3 RootCentre = vec3(RootScale >> 1);

        // Initiate the recursive construction process
        BuildSubOctreeRecursive(RootNode,
                                Tree,
                                OCT_C000,
                                0,
                                RootScale,
                                RootBlk,
                                RootCentre,
                                SurfaceFn);

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

                //ParentNode = GetNodeChild(ParentNode, , (svo_oct)CurrentOct);

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

static svo_node*
InsertChild(svo_node* Parent, svo_oct ChildOct, svo_block* ParentBlk, svo* Tree, svo_voxel_type Type)
{
    svo_node PCopy = *Parent;
    SetOctantOccupied(ChildOct, Type, &PCopy);
    
    // Gather siblings of the newborn child
    u32 NonLeafChildMsk = PCopy.OccupiedMask & (~PCopy.LeafMask);
    u32 ChildCount = CountSetBits(NonLeafChildMsk);

    svo_block* CurrentBlk = ParentBlk;
    svo_node* CreatedParent = nullptr;
    svo_node* FirstChild = nullptr;
    svo_block* FirstChildBlk = nullptr;

    // Process each child (max 8) of the parent node, including the one
    // we just inserted.
    for (u32 ChildIndex = 0; ChildIndex < ChildCount; ++ChildIndex)
    {
        // Octant of the child we are processing
        svo_oct Oct = (svo_oct)FindLowestSetBit(NonLeafChildMsk);

        // If the current octant we're processing is the one we're inserting, 
        // we need to allocate a child svo_node for this octant.
        if (ChildOct == Oct && Type == VOXEL_PARENT)
        {
            svo_node* NewParent = AllocateNode(CurrentBlk);
            if (nullptr == NewParent)
            {
                CurrentBlk = AllocateAndLinkNewBlock(CurrentBlk, Tree);
                NewParent = AllocateNode(CurrentBlk);
            }

            CreatedParent = NewParent;

            if (nullptr == FirstChild)
            {
                FirstChild = CreatedParent;
                FirstChildBlk = CurrentBlk;
            }
        }
        else
        {
            svo_node* Child = GetNodeChild(Parent, ParentBlk, Oct);
            svo_node* RelocatedChild = PushNode(CurrentBlk, *Child);

            if (nullptr == RelocatedChild)
            {
                CurrentBlk = AllocateAndLinkNewBlock(CurrentBlk, Tree);
                RelocatedChild = PushNode(CurrentBlk, *Child);
            }

            if (nullptr == FirstChild)
            {
                FirstChild = RelocatedChild;
                FirstChildBlk = CurrentBlk;
            }
        }

        // Clear the processed oct from the mask
        NonLeafChildMsk &= ~(1U << Oct);

    }

    assert(FirstChild);

    *Parent = PCopy;
    SetNodeChildPointer(FirstChild, FirstChildBlk, ParentBlk, Parent);

    return CreatedParent;
}

extern "C" void
InsertVoxel(svo* Tree, vec3 P, u32 VoxelScale)
{
    svo_node* Root = &Tree->RootBlock->Entries[0];
    svo_node* C0 = GetNodeChild(Root, Tree->RootBlock, OCT_C001);
    svo_node* C1 = GetNodeChild(C0, Tree->RootBlock, OCT_C001);
    svo_node* C2 = GetNodeChild(C1, Tree->RootBlock, OCT_C001);
    //SetOctantOccupied(OCT_C000, VOXEL_LEAF, &Tree->RootBlock->Entries[0]);
    InsertChild(&Tree->RootBlock->Entries[0], OCT_C000, Tree->RootBlock, Tree, VOXEL_LEAF);

    svo_node* D0 = GetNodeChild(Root, Tree->RootBlock, OCT_C001);
    svo_node* D1 = GetNodeChild(D0, Tree->RootBlock, OCT_C001);
    svo_node* D2 = GetNodeChild(D1, Tree->RootBlock, OCT_C001);

    assert(C0->Packed == D0->Packed);
    assert(C1->Packed == D1->Packed);
    assert(C2->Packed == D2->Packed);

    return;
#if 0
    u32 RootScale = 1 << (Tree->ScaleExponent - 1);

    vec3 ParentCentreP = vec3(RootScale);
    svo_oct CurrentOct = GetOctantForPosition(P, ParentCentreP);

    bool AllocatedParent = false;

    svo_node* ParentNode = &Tree->RootBlock->Entries[0];

    // Beginning at the root scale, descend the tree until we get
    // to the desired scale, or we hit a leaf octant (which means
    // we can't go any further).
    for (u32 Scale = RootScale; Scale >= VoxelScale; Scale >>= 1)
    {
        // Check if we've reached the desired scale; if so, set
        // the current octant as a leaf and break out.
        if (Scale <= VoxelScale)
        {
            SetOctantOccupied(CurrentOct, VOXEL_LEAF, ParentNode);
            return; // TODO(Liam): Check if it's safe to early return here
        }

        // If the current octant is occupied, we can either
        // descend further or break out (if it's a leaf)
        if (IsOctantOccupied(ParentNode, CurrentOct))
        {
            if (IsOctantLeaf(ParentNode, CurrentOct))
            {
                return;
            }
            else
            {
                // Get a new parent  and loop back to the beginning
                ParentCentreP = GetOctantCentre(CurrentOct, Scale >> 1, ParentCentreP);
                CurrentOct = GetOctantForPosition(P, ParentCentreP);
                ParentNode = GetNodeChild(ParentNode, Tree->RootBlock, CurrentOct);

                continue;
            }
        }
        else
        {
            // If the current octant isn't occupied, we will need to build a new subtree
            // containing our inserted voxel.

            // First, mark this octant as a subdivision container
            SetOctantOccupied(CurrentOct, VOXEL_PARENT, ParentNode);

            // Check if we have previously copied out the parent nodes for this 
            // node
            if (AllocatedParent)
            {
                ParentNode = AllocateNode(Tree->LastBlock); 
                if (nullptr == ParentNode)
                {
                    svo_block* NewBlk = AllocateAndLinkNewBlock(Tree->LastBlock, Tree);
                    ParentNode = AllocateNode(NewBlk);
                }
            }
            else
            {
                u32 NonLeafChildMsk = ParentNode->OccupiedMask & (~ParentNode->LeafMask);
                svo_node* NewParent = nullptr;
                svo_node* FirstChild = nullptr;
                svo_block* CurrentBlk = Tree->LastBlock;

                u32 ChildCount = CountSetBits(NonLeafChildMsk);

                // Copy all of the children (including the new one) onto the end
                // of the tree
                for (u32 ChildIndex = 0; ChildIndex < ChildCount; ++ChildIndex)
                {
                    u32 ChildOct = FindHighestSetBit(NonLeafChildMsk);

                    if (ChildOct == CurrentOct)
                    {
                        NewParent = AllocateNode(CurrentBlk);
                        if (nullptr == NewParent)
                        {
                            svo_block* NewBlk = AllocateAndLinkNewBlock(CurrentBlk, Tree);
                            NewParent = AllocateNode(NewBlk);
                            CurrentBlk = NewBlk;

                        }

                        if (nullptr == FirstChild) FirstChild = NewParent;
                    }
                    else
                    {
                        u32 SetBitsBehindOctIdx = (1 << ChildOct) - 1;
                        u32 ChildOffset = CountSetBits(NonLeafChildMsk & SetBitsBehindOctIdx);


                        svo_node* SiblingNode = GetNodeChild(ParentNode, Tree->RootBlock, (svo_oct)ChildOct);
                        // Copy the old node to a new memory region
                        if (false == PushNode(Tree->RootBlock, *SiblingNode))
                        {
                            // TODO(Liam): Handle block allocation!!
                        }

                        if (! FirstChild) FirstChild = SiblingNode;
                    }

                    // Clear this oct from the mask
                    NonLeafChildMsk &= ~(1U << ChildOct);
                }

                if (FirstChild)
                {
                   u16 ChildPtr = (u16)(FirstChild - ParentNode); 
                   SetNodeChildPointer(FirstChild, Tree->RootBlock, Tree->RootBlock, ParentNode);
                }

                if (NewParent)
                {
                    ParentNode = NewParent;
                }

                AllocatedParent = true;
            }

        }


    }
#endif
}

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

