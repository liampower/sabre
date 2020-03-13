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

struct node_ref
{
    svo_block* Blk;
    svo_node*  Node;
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
GetTreeMaxScaleBiased(const svo* const Tree)
{
    return 1U << Tree->ScaleExponent << Tree->Bias;
}

static inline bool
PointInCube(vec3 P, vec3 Min, vec3 Max)
{
    return All(GreaterThanEqual(P, Min) && LessThanEqual(P, Max));
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

static inline bool
HasFarChildren(svo_node* Node)
{
    return Node->ChildPtr & SVO_FAR_PTR_BIT_MASK;
}

static inline far_ptr*
PushFarPtr(far_ptr FarPtr, svo_block* Blk)
{
    Blk->FarPtrs[Blk->NextFarPtrSlot] = FarPtr;
    far_ptr* AllocatedFarPtr = &Blk->FarPtrs[Blk->NextFarPtrSlot];
    ++Blk->NextFarPtrSlot;

    return AllocatedFarPtr;
}

static inline far_ptr*
GetFarPointer(svo_node* Node, svo_block* Blk)
{
    if (HasFarChildren(Node))
    {
        u16 FarPtrIndex = Node->ChildPtr & SVO_NODE_CHILD_PTR_MSK;

        return &Blk->FarPtrs[FarPtrIndex];
    }
    else
    {
        return nullptr;
    }
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
   u32 LeafBit = 1U << SubOctant;

   OutEntry->OccupiedMask |= ValidBit;

   if (VOXEL_LEAF == Type)
   {
       OutEntry->LeafMask |= LeafBit;
   }
   else
   {
       OutEntry->LeafMask &= ~LeafBit;
   }
}


static void
SetOctreeScaleBias(svo* const Tree)
{
    if (Tree->MaxDepth > Tree->ScaleExponent)
    {
        Tree->Bias = (Tree->MaxDepth - Tree->ScaleExponent) + 1;
        Tree->InvBias = (1.0f / (f32)(1U << Tree->Bias));
    }
    else
    {
        Tree->Bias = 0;
        Tree->InvBias = 1.0f;
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


static inline u32
GetNodeChildOffset(svo_node* Parent, svo_oct ChildOct)
{
    u32 NonLeafChildren = Parent->OccupiedMask & (~Parent->LeafMask);
    u32 SetBitsBehindOctIdx = (1U << (u32)ChildOct) - 1;
    u32 ChildOffset = CountSetBits(NonLeafChildren & SetBitsBehindOctIdx); 

    return ChildOffset;
}


static node_ref
GetNodeChildWithBlock(svo_node* Parent, svo_block* ParentBlk, svo_oct ChildOct)
{
    node_ref Result = { };

    u32 ChildOffset = GetNodeChildOffset(Parent, ChildOct);

    u32 ChildPtrBase = Parent->ChildPtr & SVO_NODE_CHILD_PTR_MSK;

    if (HasFarChildren(Parent))
    {
        far_ptr* ParentFarPtr = &ParentBlk->FarPtrs[ChildPtrBase];
        svo_block* ChildBlk = ParentBlk;

        i32 BlksJumped = 0;
        // TODO(Liam): Clean up signed/unsigned mismatches here
        i32 BlksToJump = (i32)(ParentFarPtr->BlkIndex - ParentBlk->Index) + (i32)((ChildOffset + ParentFarPtr->NodeOffset) / SVO_ENTRIES_PER_BLOCK);

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

        Result.Node = &ChildBlk->Entries[ParentFarPtr->NodeOffset + ChildOffset];
        Result.Blk = ChildBlk;

        return Result;
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

        Result.Node = &ChildBlk->Entries[ChildIndex];
        Result.Blk = ChildBlk;

        return Result;
    }
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
        i32 BlksToJump = (i32)(ParentFarPtr->BlkIndex - ParentBlk->Index) + (i32)((ChildOffset + ParentFarPtr->NodeOffset) / SVO_ENTRIES_PER_BLOCK);

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
    assert(NextFarPtrSlot < SVO_FAR_PTRS_PER_BLOCK);
    far_ptr* Ptr = &ContainingBlk->FarPtrs[NextFarPtrSlot];
    ++ContainingBlk->NextFarPtrSlot;

    assert(Ptr->NodeOffset == 0);
    assert(Ptr->BlkIndex == 0);

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
        FarPtr->BlkIndex = (u32)ChildBlk->Index;
        FarPtr->NodeOffset = ChildPtrBits;

        // Set the child ptr value to the far bit with the remaining 15 bits
        // set as the index into the far ptrs storage block.
        Parent->ChildPtr = SVO_FAR_PTR_BIT_MASK | u16(ParentBlk->NextFarPtrSlot - 1);
    }
}



static inline vec3
GetOctantCentre(svo_oct Octant, u32 Scale, vec3 ParentCentreP)
{
    assert(Scale > 0);
    u32 Oct = (u32) Octant;

    f32 Rad = (f32)(Scale >> 1U);
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
    u32 NextDepth = Depth + 1;

    svo_block* CurrentBlk = Tree->LastBlock;

    node_child Children[8];
    u32 LastChildIndex = 0;

    // TODO(Liam): Try to eliminate the need for a +1 bias; we would like to *not* have
    // the bias lie about what the actual difference in levels is!
    vec3 Radius = vec3(Scale >> 1);
    
    for (u32 Oct = 0; Oct < 8; ++Oct)
    {
        // NOTE(Liam): Multiplying by the InvBias here transforms the octant cubes back 
        // into "real" space from the scaled space we operate in when constructing the
        // tree.
        vec3 OctCentre = GetOctantCentre((svo_oct)Oct, Scale, Centre);
        vec3 OctMin = (OctCentre - Radius) * Tree->InvBias;
        vec3 OctMax = (OctCentre + Radius) * Tree->InvBias;

        if (SurfaceFn(OctMin, OctMax, Tree))
        {
            // Check if the NEXT depth is less than the tree max
            // depth.
            if (Depth < Tree->MaxDepth)
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
                                NextDepth,
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
        u32 RootScale = (1U << ScaleExponent);
        
        SetOctreeScaleBias(Tree);

        // Scale up by the bias
        RootScale <<= Tree->Bias;

        printf("ROOTSCALE %u\n", RootScale);

        vec3 RootCentre = vec3(RootScale >> 1);

        // Initiate the recursive construction process
        // The root depth is initialised to 1 because we are
        // technically beginning at the *second* tree level
        // when constructing the tree. The first level exists
        // implicitly in the allocated root node, the root
        // "parent".
        BuildSubOctreeRecursive(RootNode,
                                Tree,
                                OCT_C000,
                                1,
                                RootScale >> 1,
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

static inline node_ref
ReAllocateNode(svo_node* Node, svo_block* NodeBlk, svo* Tree)
{
    node_ref Result = { };
    // The block that the reallocated node will reside in.
    svo_block* NewBlk = Tree->LastBlock;

    // Attempt to allocate in the same block
    svo_node* NewNode = PushNode(NewBlk, *Node);

    if (nullptr == NewNode)
    {
        NewBlk = AllocateAndLinkNewBlock(NewBlk, Tree);
        NewNode = PushNode(NewBlk, *Node);
    }

    // If this node has any local pointers, and we moved to a new block, we need
    // to change those local pointers into far pointers.
    if (NewBlk != NodeBlk)
    {
        // Moved to a new blk and our child pointer is a far pointer.
        // Need to copy this far pointer into the new child block.
        if (HasFarChildren(Node))
        {
            // Get this node's far ptr
            far_ptr* FarPtr = GetFarPointer(Node, NodeBlk);

            // Copy the far ptr into the new block
            PushFarPtr(*FarPtr, NewBlk);

            // Set the node's far pointer index to the new far pointer.
            NewNode->ChildPtr = SVO_FAR_PTR_BIT_MASK | u16(NewBlk->NextFarPtrSlot - 1);
        }
        else
        {
            // Need to convert local pointer into far pointer.
            u16 OldChildPtr = Node->ChildPtr;
            far_ptr* NewFarPtr = AllocateFarPtr(NewBlk);

            NewFarPtr->BlkIndex = NodeBlk->Index;
            NewFarPtr->NodeOffset = OldChildPtr;
            NewNode->ChildPtr = SVO_FAR_PTR_BIT_MASK | u16(NewBlk->NextFarPtrSlot - 1);
        }
    }

    Result.Node = NewNode;
    Result.Blk = NewBlk;

    return Result;
}

static void
DeleteChild(svo_node* Parent, svo_oct ChildOct, svo_block* ParentBlk, svo* Tree)
{
    svo_node PCopy = *Parent;

    // Mask out the dead child
    u32 ClearMsk = ~(1U << (u32)ChildOct);
    PCopy.LeafMask &= ClearMsk;
    PCopy.OccupiedMask &= ClearMsk;

    // Compute the new sibling count
    u32 NonLeafChildMsk = PCopy.OccupiedMask & (~PCopy.LeafMask);
    u32 ChildCount = CountSetBits(NonLeafChildMsk);

    svo_block* CurrentChildBlk = ParentBlk;
    svo_node* FirstChild = nullptr;
    svo_block* FirstChildBlk = nullptr;

    for (u32 ChildIndex = 0; ChildIndex < ChildCount; ++ChildIndex)
    {
        svo_oct Oct = (svo_oct)FindLowestSetBit(NonLeafChildMsk);

        node_ref ChildRef = GetNodeChildWithBlock(Parent, ParentBlk, Oct);
        CurrentChildBlk = ChildRef.Blk;
        svo_node* Child = ChildRef.Node;
        node_ref MovedChild = ReAllocateNode(Child, CurrentChildBlk, Tree);

        if (nullptr == FirstChild)
        {
            FirstChildBlk = MovedChild.Blk;
            FirstChild = MovedChild.Node;
        }

        NonLeafChildMsk &= ~(1U << Oct);
    }

    *Parent = PCopy;
    SetNodeChildPointer(FirstChild, FirstChildBlk, ParentBlk, Parent);
}

static node_ref
InsertChild(svo_node* Parent, svo_oct ChildOct, svo_block* ParentBlk, svo* Tree, svo_voxel_type Type)
{
    svo_node PCopy = *Parent;
    SetOctantOccupied(ChildOct, Type, &PCopy);
    
    // Gather siblings of the newborn child
    u32 NonLeafChildMsk = PCopy.OccupiedMask & (~PCopy.LeafMask);
    u32 ChildCount = CountSetBits(NonLeafChildMsk);

    svo_block* FirstChildBlk = nullptr;
    svo_block* CurrentChildBlk = ParentBlk;
    svo_node* FirstChild = nullptr;

    node_ref CreatedNode = { };

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
            svo_node* NewChild = AllocateNode(CurrentChildBlk);

            if (nullptr == NewChild)
            {
                CurrentChildBlk = AllocateAndLinkNewBlock(Tree->LastBlock, Tree);
                NewChild = AllocateNode(CurrentChildBlk);
            }

            if (nullptr == FirstChild)
            {
                FirstChild = NewChild;
                FirstChildBlk = CurrentChildBlk;
            }

            CreatedNode.Node = NewChild;
            CreatedNode.Blk = CurrentChildBlk;
        }
        else
        {
            node_ref ChildRef = GetNodeChildWithBlock(Parent, ParentBlk, Oct);
            CurrentChildBlk = ChildRef.Blk;
            svo_node* Child = ChildRef.Node;
            node_ref MovedChild = ReAllocateNode(Child, CurrentChildBlk, Tree);
            if (nullptr == FirstChild)
            {
                FirstChild = MovedChild.Node;
                FirstChildBlk = MovedChild.Blk;
            }
        }

        // Clear the processed oct from the mask
        NonLeafChildMsk &= ~(1U << Oct);
    }

    assert(FirstChild);

    *Parent = PCopy;
    SetNodeChildPointer(FirstChild, FirstChildBlk, ParentBlk, Parent);

    return CreatedNode;
}

extern "C" void
InsertVoxel(svo* Tree, vec3 P, u32 VoxelScale)
{
    // Obtain the original root scale of the tree, though
    // this may need to be biased further if the inserted
    // voxel scale is smaller than the tree minimum scale.
    u32 RootScale = 1U << (Tree->ScaleExponent) << Tree->Bias;

    // Scale of the smallest voxel in the SVO. For biased
    // trees, this will always be 1. For non-biased trees,
    // this will be the scale at the tree's MaxDepth.
    u32 TreeMinScale = (Tree->MaxDepth > Tree->ScaleExponent) ? 1U : 1U << (Tree->ScaleExponent - Tree->MaxDepth);

    if (VoxelScale < TreeMinScale)
    {
       Tree->MaxDepth += TreeMinScale - VoxelScale; 

       // Re-scale the tree if the requested voxel scale is smaller than
       // the tree minimum scale.
       SetOctreeScaleBias(Tree);
    }

    // Inserted voxel position, scaled by the tree bias.
    vec3 InsertP = P * (1U << Tree->Bias);

    RootScale = 1U << (Tree->ScaleExponent) << Tree->Bias;
    const vec3 TreeMax = vec3(RootScale);
    const vec3 TreeMin = vec3(0);

    // TODO(Liam): Expand tree?
    if (false == PointInCube(InsertP, TreeMin, TreeMax))
    {
        return;
    }

    vec3 ParentCentreP = vec3(RootScale >> 1);
    svo_oct CurrentOct = GetOctantForPosition(InsertP, ParentCentreP);

    bool AllocatedParent = false;

    svo_node* ParentNode = &Tree->RootBlock->Entries[0];
    svo_block* ParentBlk = Tree->RootBlock;

    // Need to bias the voxel scale in case of upscaled
    // trees.
    u32 EditScale = VoxelScale << Tree->Bias;

    // Beginning at the root scale, descend the tree until we get
    // to the desired scale, or we hit a leaf octant (which means
    // we can't go any further).
    for (u32 Scale = RootScale; Scale >= EditScale; Scale >>= 1)
    {
        // Check if we've reached the desired scale; if so, set
        // the current octant as a leaf and break out.
        if (Scale <= EditScale)
        {
            if (AllocatedParent)
            {
                SetOctantOccupied(CurrentOct, VOXEL_LEAF, ParentNode);
            }
            else
            {
                InsertChild(ParentNode, CurrentOct, ParentBlk, Tree, VOXEL_LEAF);
            }

            return;
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
                node_ref ParentRef = GetNodeChildWithBlock(ParentNode, ParentBlk, CurrentOct);
                CurrentOct = GetOctantForPosition(InsertP, ParentCentreP);
                ParentNode = ParentRef.Node;
                ParentBlk = ParentRef.Blk;
                //ParentNode = GetNodeChildWithBlock(ParentNode, ParentBlk, CurrentOct);

                continue;
            }
        }
        else
        {
            // If the current octant isn't occupied, we will need to build a new subtree
            // containing our inserted voxel.

            // First, mark this octant as a subdivision container
            node_ref ParentNodeRef = InsertChild(ParentNode, CurrentOct, ParentBlk, Tree, VOXEL_PARENT);
            ParentNode = ParentNodeRef.Node;
            ParentBlk = ParentNodeRef.Blk;
            AllocatedParent = true;
        }
    }
}

extern "C" void
DeleteVoxel(svo* Tree, vec3 P)
{
    // Always go down to leaf scale (cheat at mem. mgmt!)
    u32 TreeMaxScale = GetTreeMaxScaleBiased(Tree);

    svo_node* ParentNode = &Tree->RootBlock->Entries[0];
    vec3 ParentCentre = vec3(TreeMaxScale >> 1U);
    svo_oct CurrentOct = GetOctantForPosition(P, ParentCentre);
    svo_block* ParentBlk = Tree->RootBlock;

    bool CreatedChild = false;

    // For configurations where MaxDepth > ScaleExponent, this will always
    // be 1.
    //
    // For configurations where MaxDepth < ScaleExponent, this will always
    // be 1 << (ScaleExponent - MaxDepth)
    u32 MinScale = (Tree->MaxDepth > Tree->ScaleExponent) ? 1U : 1U << (Tree->ScaleExponent - Tree->MaxDepth);
    u32 CurrentScale = TreeMaxScale;

    // Descend the tree until we get to the minium scale.
    //
    while (CurrentScale > MinScale)
    {
        CurrentScale >>= 1;

        if (CreatedChild && CurrentScale != MinScale)
        {
            svo_node* NewParent = AllocateNode(ParentBlk);    
            svo_block* NewParentBlk = ParentBlk;
            if (nullptr == NewParent)
            {
                NewParentBlk = AllocateAndLinkNewBlock(Tree->LastBlock, Tree);
                NewParent = AllocateNode(NewParentBlk);
            }

            u32 OctMsk = ~(1U << (u32)CurrentOct);
            NewParent->OccupiedMask = 0xFF;   // All octants occupied
            NewParent->LeafMask = (u8)OctMsk; // All octants except current leaves

            SetNodeChildPointer(NewParent, NewParentBlk, ParentBlk, ParentNode);
            ParentNode = NewParent;
            ParentBlk = NewParentBlk;
        }
        else if (IsOctantOccupied(ParentNode, CurrentOct))
        {
            if (CurrentScale == MinScale)
            {
                u32 ClearMsk = ~(1U << (u32)CurrentOct);
                ParentNode->LeafMask &= ClearMsk;
                ParentNode->OccupiedMask &= ClearMsk;

                return;
            }

            if (IsOctantLeaf(ParentNode, CurrentOct))
            {
                // Insert a child node into the parent
                node_ref ChildRef = InsertChild(ParentNode, 
                                                CurrentOct,
                                                ParentBlk, 
                                                Tree, 
                                                VOXEL_PARENT);
                ParentNode = ChildRef.Node;
                ParentBlk = ChildRef.Blk;

                ParentCentre = GetOctantCentre(CurrentOct, CurrentScale >> 1, ParentCentre);
                CurrentOct = GetOctantForPosition(P, ParentCentre);

                // Set all octants *except* the current one to leaves
                u32 OctMsk = ~(1U << (u32)CurrentOct);
                ParentNode->OccupiedMask = 0xFF;   // All octants occupied
                ParentNode->LeafMask = (u8)OctMsk; // All octants except current leaves

                CreatedChild = true;
            }
            else
            {
                // Traverse deeper into the tree
                ParentCentre = GetOctantCentre(CurrentOct, CurrentScale >> 1, ParentCentre);
                CurrentOct = GetOctantForPosition(P, ParentCentre);
                node_ref ParentRef = GetNodeChildWithBlock(ParentNode, ParentBlk, CurrentOct);
                ParentNode = ParentRef.Node;
                ParentBlk = ParentRef.Blk;
            }
        }
        else
        {
            return;
        }
    }

#if 0
    // Descend through the tree until we reach a voxel scale of 1
    for (u32 VoxelScale = RootScale; VoxelScale >= 1; VoxelScale >>= 1)
    {
        // If we have previously created a parent node (by splitting a larger
        // leaf node into smaller leaf nodes) then we need to continue building
        // the tree down through this 
        if (CreatedChild && VoxelScale != 1)
        {
            // Allocate a new parent node for this octant
            svo_node* NewParent = AllocateNode(ParentBlk);    
            svo_block* NewParentBlk = ParentBlk;
            if (nullptr == NewParent)
            {
                NewParentBlk = AllocateAndLinkNewBlock(Tree->LastBlock, Tree);
                NewParent = AllocateNode(NewParentBlk);
            }

            u32 OctMsk = ~(1U << (u32)CurrentOct);
            NewParent->OccupiedMask = 0xFF;   // All octants occupied
            NewParent->LeafMask = (u8)OctMsk; // All octants except current leaves

            SetNodeChildPointer(NewParent, NewParentBlk, ParentBlk, ParentNode);
            ParentNode = NewParent;
            ParentBlk = NewParentBlk;

            continue;
        }

        if (IsOctantOccupied(ParentNode, CurrentOct))
        {
            if (IsOctantLeaf(ParentNode, CurrentOct))
            {
                if (VoxelScale == 1)
                {
                    // Remove the deleted child from the parent
                    u32 ClearMsk = ~(1U << (u32)CurrentOct);
                    ParentNode->LeafMask &= ClearMsk;
                    ParentNode->OccupiedMask |= ~ClearMsk;

                    return;
                }
                else
                {
                    // Insert a child node into the parent
                    node_ref ChildRef = InsertChild(ParentNode, 
                                                    CurrentOct,
                                                    ParentBlk, 
                                                    Tree, 
                                                    VOXEL_PARENT);
                    ParentNode = ChildRef.Node;
                    ParentBlk = ChildRef.Blk;

                    ParentCentre = GetOctantCentre(CurrentOct, VoxelScale >> 1, ParentCentre);
                    CurrentOct = GetOctantForPosition(P, ParentCentre);

                    // Set all octants *except* the current one to leaves
                    u32 OctMsk = ~(1U << (u32)CurrentOct);
                    ParentNode->OccupiedMask = 0xFF;   // All octants occupied
                    ParentNode->LeafMask = (u8)OctMsk; // All octants except current leaves

                    CreatedChild = true;
                }
            }
            else
            {
                // Traverse deeper into the tree
                ParentCentre = GetOctantCentre(CurrentOct, VoxelScale >> 1, ParentCentre);
                CurrentOct = GetOctantForPosition(P, ParentCentre);
                node_ref ParentRef = GetNodeChildWithBlock(ParentNode, ParentBlk, CurrentOct);
                ParentNode = ParentRef.Node;
                ParentBlk = ParentRef.Blk;
            }
        }
        else
        {
            return;
        }
    }
#endif
}

extern "C" void
OutputSvoToFile(const svo* const Svo, FILE* FileOut)
{
    // First, write the header
    fwrite(Svo, sizeof(svo), 1, FileOut);

    // Traverse tree and write blocks
    svo_block* CurrentBlk = Svo->RootBlock;
    while (CurrentBlk)
    {
        fwrite(CurrentBlk, sizeof(svo_block), 1, FileOut);
        CurrentBlk = CurrentBlk->Next;
    }
}


extern "C" svo*
LoadSvoFromFile(FILE* FileIn)
{
    // Allocate a buffer to load into
    svo* Svo = (svo*)calloc(1, sizeof(svo));

    // Read the header
    if (0 == fread(Svo, sizeof(svo), 1, FileIn))
    {
        fprintf(stderr, "IO Error\n");
        return nullptr;
    }

    // Need to invalidate all pointers
    Svo->RootBlock = nullptr;
    Svo->LastBlock = nullptr;

    // Start reading the blocks 
    for (u32 BlkIndex = 0; BlkIndex < Svo->UsedBlockCount; ++BlkIndex)
    {
        // Read the block
        svo_block* CurrentBlk = (svo_block*)calloc(1, sizeof(svo_block));

        if (Svo->RootBlock == nullptr)
        {
            Svo->RootBlock = CurrentBlk;
        }

        CurrentBlk->Prev = Svo->LastBlock;
        if (Svo->LastBlock) Svo->LastBlock->Next = CurrentBlk;

        // TODO(Liam): Check failure
        if (0 == fread(CurrentBlk, sizeof(svo_block), 1, FileIn))
        {
            // TODO(Liam): Free? (ugh)
            fprintf(stderr, "IO Error\n");
            return nullptr;
        }
        // Need to link prev block to this one
        Svo->LastBlock = CurrentBlk;
    }

    return Svo;
}

extern "C" void
DeleteSparseVoxelOctree(svo* Tree)
{
    svo_block* CurrentBlk = Tree->RootBlock;

    while (CurrentBlk)
    {
        svo_block* NextBlk = CurrentBlk->Next;
        free(CurrentBlk);
        CurrentBlk = NextBlk;
    }

    free(Tree);
}

