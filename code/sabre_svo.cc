#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <intrin.h>

#include "sabre.h"
#include "sabre_math.h"
#include "sabre_svo.h"

static constexpr u16 CHILD_PTR_MSK = 0x7FFFU;


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

static inline u32
GetTreeMinScaleBiased(const svo* const Tree)
{
    return (Tree->MaxDepth > Tree->ScaleExponent) ? 
            1U : 
            1U << (Tree->ScaleExponent - Tree->MaxDepth);
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

static inline node_ref
GetTreeRootNodeRef(const svo* const Tree)
{
    return node_ref{ Tree->RootBlock, &Tree->RootBlock->Entries[0] };
}

static inline far_ptr*
GetFarPointer(svo_node* Node, svo_block* Blk)
{
    if (HasFarChildren(Node))
    {
        u16 FarPtrIndex = Node->ChildPtr & CHILD_PTR_MSK;

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
GetNodeChild(node_ref ParentRef, svo_oct ChildOct)
{
    // Offset value from 0 - 7 of the particular child we are
    // interested in from the first child.
    u32 ChildOffset = GetNodeChildOffset(ParentRef.Node, ChildOct);

    // How many blocks (forwards or backwards) we need to go
    // from the parent block to reach the block containing the
    // child.
    i32 BlksToJump = 0;

    u32 FirstChildIndex = 0;

    if (HasFarChildren(ParentRef.Node))
    {
        u32 FarPtrIndex = ParentRef.Node->ChildPtr & CHILD_PTR_MSK;
        far_ptr* FarPtr = &ParentRef.Blk->FarPtrs[FarPtrIndex];
        FirstChildIndex = FarPtr->NodeOffset;

        i32 BlksFromParent = (i32)FarPtr->BlkIndex - (i32)ParentRef.Blk->Index;
        i32 BlksFromFirst = (i32)((FirstChildIndex + ChildOffset) / SVO_ENTRIES_PER_BLOCK);

        BlksToJump = BlksFromParent + BlksFromFirst;
    }
    else
    {
        FirstChildIndex = ParentRef.Node->ChildPtr & CHILD_PTR_MSK;
        i32 BlksFromFirst = (i32)((FirstChildIndex + ChildOffset) / SVO_ENTRIES_PER_BLOCK);

        BlksToJump = BlksFromFirst;
    }

    svo_block* ChildBlk = ParentRef.Blk;
    i32 BlksJumped = 0;
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

    u32 ChildIndex = (FirstChildIndex + ChildOffset) % SVO_ENTRIES_PER_BLOCK;

    return node_ref{ ChildBlk, &ChildBlk->Entries[ChildIndex] };
}

static node_ref
GetNodeChildWithBlock(svo_node* Parent, svo_block* ParentBlk, svo_oct ChildOct)
{
    node_ref Result = { };

    u32 ChildOffset = GetNodeChildOffset(Parent, ChildOct);

    u32 ChildPtrBase = Parent->ChildPtr & CHILD_PTR_MSK;

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
LinkParentAndChildNodes(node_ref ParentRef, node_ref ChildRef)
{
    // Compute the child offset from the start of the block
    u16 ChildOffset = (u16)(ChildRef.Node - ChildRef.Blk->Entries);

    // Extract first 15 bits
    u16 ChildPtrBits = ChildOffset & CHILD_PTR_MSK;

    // If the child node is in the same block as the parent node, all we need to do
    // is set the parent's child ptr to the child offset from the beginning of
    // their shared block.
    if (ChildRef.Blk == ParentRef.Blk)
    {
        ParentRef.Node->ChildPtr = ChildPtrBits;
    }
    else
    {
        // Allocate a new far pointer in the parent
        // TODO(Liam): Error handling
        far_ptr* FarPtr = AllocateFarPtr(ParentRef.Blk);
        FarPtr->BlkIndex = (u32)ChildRef.Blk->Index;
        FarPtr->NodeOffset = ChildPtrBits;

        // Set the child ptr value to the far bit with the remaining 15 bits
        // set as the index into the far ptrs storage block.
        ParentRef.Node->ChildPtr = SVO_FAR_PTR_BIT_MASK | u16(ParentRef.Blk->NextFarPtrSlot - 1);
    }
}

static void
SetNodeChildPointer(svo_node* Child, svo_block* ChildBlk, svo_block* ParentBlk, svo_node* Parent)
{
    // Compute the child offset from the start of the block
    u16 ChildOffset = (u16)(Child - ChildBlk->Entries);

    // Extract first 15 bits
    u16 ChildPtrBits = ChildOffset & CHILD_PTR_MSK;

    // If the child node is in the same block as the parent node, all we need to do
    // is set the parent's child ptr to the child offset from the beginning of
    // their shared block.
    if (ChildBlk == ParentBlk)
    {
        Parent->ChildPtr = ChildPtrBits;
    }
    else
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

static inline node_ref
AllocateNewNode(svo_block* const ParentBlk, svo* const Tree)
{
    // Check if there is room in the parent block;
    // If there isn't any, we need to allocate a new
    // block. Otherwise, we can just return the parent
    // block we were passed as the new parent.
    if (0 == GetFreeSlotCount(ParentBlk))
    {
        svo_block* NewBlk = AllocateAndLinkNewBlock(Tree->LastBlock, Tree);
        svo_node* NewNode = &NewBlk->Entries[NewBlk->NextFreeSlot];

        return node_ref{ NewBlk, NewNode };
    }
    else
    {
        svo_node* NewNode = &ParentBlk->Entries[ParentBlk->NextFreeSlot];
        ++ParentBlk->NextFreeSlot;

        return node_ref{ ParentBlk, NewNode };
    }
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

    node_child Children[8];
    u32 LastChildIndex = 0;

    // TODO(Liam): Try to eliminate the need for a +1 bias; we would like to *not* have
    // the bias lie about what the actual difference in levels is!
    vec3 Radius = vec3(Scale >> 1);
    
    for (u32 Oct = 0; Oct < 8; ++Oct)
    {
        // Multiplying by the InvBias here transforms the octant cubes back 
        // into "real" space from the scaled space we operate in when
        // constructing the tree.
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
                /*svo_node* Child = AllocateNode(Tree->LastBlock);

                if (nullptr == Child)
                {
                    svo_block* NewBlk = AllocateAndLinkNewBlock(Tree->LastBlock, Tree);
                    Child = AllocateNode(NewBlk);
                    CurrentBlk = NewBlk;
                }*/

                node_ref ChildRef = AllocateNewNode(Tree->LastBlock, Tree);
                svo_node* Child = ChildRef.Node;

                const node_ref ParentRef = { ParentBlk, Parent };

                // If the parent's child pointer has already been set, we do not
                // need to set it again. Child pointers should only point to the
                // first child.
                if (0x0000 == Parent->ChildPtr)
                {
                    LinkParentAndChildNodes(ParentRef, ChildRef);
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
ReAllocateNode(node_ref NodeRef, svo* const Tree)
{
    node_ref Result = { };
    // The block that the reallocated node will reside in.
    svo_block* NewBlk = Tree->LastBlock;

    // Attempt to allocate in the same block
    svo_node* NewNode = PushNode(NewBlk, *NodeRef.Node);

    if (nullptr == NewNode)
    {
        NewBlk = AllocateAndLinkNewBlock(NewBlk, Tree);
        NewNode = PushNode(NewBlk, *NodeRef.Node);
    }

    // If this node has any local pointers, and we moved to a new block, we need
    // to change those local pointers into far pointers.
    if (NewBlk != NodeRef.Blk)
    {
        // Moved to a new blk and our child pointer is a far pointer.
        // Need to copy this far pointer into the new child block.
        if (HasFarChildren(NodeRef.Node))
        {
            // Get this node's far ptr
            far_ptr* FarPtr = GetFarPointer(NodeRef.Node, NodeRef.Blk);

            // Copy the far ptr into the new block
            PushFarPtr(*FarPtr, NewBlk);

            // Set the node's far pointer index to the new far pointer.
            NewNode->ChildPtr = SVO_FAR_PTR_BIT_MASK | u16(NewBlk->NextFarPtrSlot - 1);
        }
        else
        {
            // Need to convert local pointer into far pointer.
            u16 OldChildPtr = NodeRef.Node->ChildPtr;
            far_ptr* NewFarPtr = AllocateFarPtr(NewBlk);

            NewFarPtr->BlkIndex = NodeRef.Blk->Index;
            NewFarPtr->NodeOffset = OldChildPtr;
            NewNode->ChildPtr = SVO_FAR_PTR_BIT_MASK | u16(NewBlk->NextFarPtrSlot - 1);
        }
    }

    Result.Node = NewNode;
    Result.Blk = NewBlk;

    return Result;
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

static node_ref
InsertChild(node_ref ParentRef, svo_oct ChildOct, svo* Tree, svo_voxel_type Type)
{
    svo_node PCopy = *ParentRef.Node;
    SetOctantOccupied(ChildOct, Type, &PCopy);
    
    // Gather siblings of the newborn child
    u32 NonLeafChildMsk = PCopy.OccupiedMask & (~PCopy.LeafMask);
    u32 ChildCount = CountSetBits(NonLeafChildMsk);

    node_ref FirstChildRef = { nullptr, nullptr };
    node_ref CreatedNode = { nullptr, nullptr };

    bool HaveCreatedChild = false;

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
            node_ref NewChildRef = AllocateNewNode(Tree->LastBlock, Tree);

            if (! HaveCreatedChild)
            {
                FirstChildRef = NewChildRef;
            }

            HaveCreatedChild = true;
            CreatedNode = NewChildRef;
        }
        else
        {
            node_ref ChildRef = GetNodeChild(ParentRef, Oct);
            node_ref MovedChild = ReAllocateNode(ChildRef, Tree);

            if (! HaveCreatedChild)
            {
                FirstChildRef = MovedChild;
            }

            HaveCreatedChild = true;
        }

        // Clear the processed oct from the mask
        NonLeafChildMsk &= ~(1U << Oct);
    }

    if (Type == VOXEL_PARENT) assert(FirstChildRef.Node);

    *ParentRef.Node = PCopy;
    if (HaveCreatedChild)
    {
        LinkParentAndChildNodes(ParentRef, FirstChildRef);
    }

    // TODO(Liam): Returns nullptrs when Type == VOXEL_LEAF. Should we break this
    // into a separate function for inserting leaf voxels?
    //return CreatedNode;
    if (HaveCreatedChild)
    {
        return CreatedNode;
    }
    else
    {
        return ParentRef;
    }

}


extern "C" void
InsertVoxel(svo* Tree, vec3 P, u32 VoxelScale)
{
    // Obtain the original root scale of the tree, though
    // this may need to be biased further if the inserted
    // voxel scale is smaller than the tree minimum scale.
    u32 RootScale = GetTreeMaxScaleBiased(Tree);

    // Scale of the smallest voxel in the SVO. For biased
    // trees, this will always be 1. For non-biased trees,
    // this will be the scale at the tree's MaxDepth.
    u32 TreeMinScale = GetTreeMinScaleBiased(Tree);

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
    node_ref ParentRef = GetTreeRootNodeRef(Tree);

    // Need to bias the voxel scale in case of upscaled
    // trees.
    u32 EditScale = (VoxelScale << Tree->Bias);

    // Beginning at the root scale, descend the tree until we get
    // to the desired scale, or we hit a leaf octant (which means
    // we can't go any further).
#if 0
    for (u32 Scale = RootScale; Scale >= EditScale; Scale >>= 1)
    {
        // Check if we've reached the desired scale; if so, set
        // the current octant as a leaf and break out.
        if (Scale >> 1 <= EditScale)
        {
            if (AllocatedParent)
            {
                SetOctantOccupied(CurrentOct, VOXEL_LEAF, ParentRef.Node);
            }
            else
            {
                InsertChild(ParentRef.Node, CurrentOct, ParentRef.Blk, Tree, VOXEL_LEAF);
                //InsertChild(&ParentRef, CurrentOct, Tree, VOXEL_LEAF);
            }

            return;
        }

        // If the current octant is occupied, we can either
        // descend further or break out (if it's a leaf)
        if (IsOctantOccupied(ParentRef.Node, CurrentOct))
        {
            if (IsOctantLeaf(ParentRef.Node, CurrentOct))
            {
                return;
            }
            else
            {
                // Get a new parent  and loop back to the beginning
                ParentCentreP = GetOctantCentre(CurrentOct, Scale >> 1, ParentCentreP);
                //node_ref ParentRef = GetNodeChildWithBlock(ParentNode, ParentBlk, CurrentOct);
                CurrentOct = GetOctantForPosition(InsertP, ParentCentreP);
                ParentRef = GetNodeChildWithBlock(ParentRef.Node, ParentRef.Blk, CurrentOct);
                /*ParentNode = ParentRef.Node;
                ParentBlk = ParentRef.Blk;*/

                continue;
            }
        }
        else
        {
            // If the current octant isn't occupied, we will need to build a new subtree
            // containing our inserted voxel.

            // First, mark this octant as a subdivision container
            //node_ref ParentNodeRef = InsertChild(ParentNode, CurrentOct, ParentBlk, Tree, VOXEL_PARENT);
            /*node_ref ParentNodeRef = InsertChild(Parent
            ParentNode = ParentNodeRef.Node;
            ParentBlk = ParentNodeRef.Blk;*/
            ParentRef = InsertChild(&ParentRef, CurrentOct, Tree, VOXEL_PARENT);
            AllocatedParent = true;
        }
    }
#endif

    //InsertChild(ParentRef, CurrentOct, Tree, VOXEL_LEAF);
    //InsertChild(&ParentRef, CurrentOct, Tree, VOXEL_LEAF);


    u32 CurrentScale = RootScale >> 1;
    while (CurrentScale > EditScale)
    {
        CurrentScale >>= 1;

        if (IsOctantOccupied(ParentRef.Node, CurrentOct))
        {
            if (IsOctantLeaf(ParentRef.Node, CurrentOct))
            {
                return;
            }
            else
            {
                ParentCentreP = GetOctantCentre(CurrentOct, CurrentScale, ParentCentreP);
                CurrentOct = GetOctantForPosition(InsertP, ParentCentreP);
                ParentRef = GetNodeChild(ParentRef, CurrentOct);

                continue;
            }
        }
        else
        {
            // If the current octant isn't occupied, we will need to build a new subtree
            // containing our inserted voxel.

            // First, mark this octant as a subdivision container
            //node_ref ParentNodeRef = InsertChild(ParentNode, CurrentOct, ParentBlk, Tree, VOXEL_PARENT);
            /*node_ref ParentNodeRef = InsertChild(Parent
            ParentNode = ParentNodeRef.Node;
            ParentBlk = ParentNodeRef.Blk;*/
            ParentCentreP = GetOctantCentre(CurrentOct, CurrentScale, ParentCentreP);
            CurrentOct = GetOctantForPosition(InsertP, ParentCentreP);
            ParentRef = InsertChild(ParentRef, CurrentOct, Tree, VOXEL_PARENT);
            AllocatedParent = true;
        }
    }

    if (AllocatedParent)
    {
        SetOctantOccupied(CurrentOct, VOXEL_LEAF, ParentRef.Node);
    }
    else
    {
        InsertChild(ParentRef, CurrentOct, Tree, VOXEL_LEAF);
    }
}

static node_ref
DeleteLeafChild(svo_oct ChildOct, svo* const Tree, node_ref ParentRef)
{
    // Splits a former leaf node into 7 child leaves
    // and one non-leaf child.

    // TODO(Liam): Gracefully handle case where the
    // parent is not a leaf.
    assert(IsOctantLeaf(ParentRef.Node, ChildOct));

    node_ref NewChildRef = InsertChild(ParentRef, ChildOct, Tree, VOXEL_PARENT);

    // Mask of all octants except the intended
    // child.
    u32 LeavesMsk = ~(1U << (u32)ChildOct);

    // Clear this oct's bit from the parent leaf
    // mask. If this octant was a leaf, it will
    // already be occupied.
    ParentRef.Node->LeafMask &= LeavesMsk;

    // Set all octants occupied in the newly
    // allocated child.
    NewChildRef.Node->OccupiedMask = 0xFF;

    // Set all octants inside the newly allocated
    // child to 
    NewChildRef.Node->LeafMask = (u8) LeavesMsk;

    return NewChildRef;
}


extern "C" void
DeleteVoxel(svo* Tree, vec3 VoxelP)
{
    // Scale the voxel position by the tree bias
    vec3 DeleteP = VoxelP * (1U << Tree->Bias);

    // Always go down to leaf scale (cheat at mem. mgmt!)
    u32 MaxScale = GetTreeMaxScaleBiased(Tree);
    u32 MinScale = GetTreeMinScaleBiased(Tree);

    vec3 ParentCentre = vec3(MaxScale >> 1U);
    svo_oct CurrentOct = GetOctantForPosition(DeleteP, ParentCentre);

    node_ref ParentNodeRef = GetTreeRootNodeRef(Tree);
    bool CreatedChild = false;

    // For configurations where MaxDepth > ScaleExponent, this will always
    // be 1.
    //
    // For configurations where MaxDepth < ScaleExponent, this will always
    // be 1 << (ScaleExponent - MaxDepth)
    u32 CurrentScale = MaxScale;

    // Descend the tree until we get to the minium scale.
    while (CurrentScale > MinScale)
    {
        // If we had previously created a child node, we need to
        // continue building the tree until we reach the min scale.
        if (CreatedChild && CurrentScale > (MinScale << 1))
        {
            node_ref NewParentRef = AllocateNewNode(Tree->LastBlock, Tree);

            u32 OctMsk = ~(1U << (u32)CurrentOct);
            NewParentRef.Node->OccupiedMask = 0xFF;   // All octants occupied
            NewParentRef.Node->LeafMask = (u8)OctMsk; // All octants except current leaves

            LinkParentAndChildNodes(ParentNodeRef, NewParentRef);

            ParentNodeRef = NewParentRef;
        }
        else if (IsOctantOccupied(ParentNodeRef.Node, CurrentOct))
        {
            if (CurrentScale <= (MinScale << 1))
            {
                u32 ClearMsk = ~(1U << (u32)CurrentOct);
                ParentNodeRef.Node->LeafMask &= ClearMsk;
                ParentNodeRef.Node->OccupiedMask &= ClearMsk;

                return;
            }

            if (IsOctantLeaf(ParentNodeRef.Node, CurrentOct))
            {
                node_ref ChildRef = DeleteLeafChild(CurrentOct, Tree, ParentNodeRef);

                ParentNodeRef = ChildRef;
                CreatedChild = true;
            }
            else
            {
                // Traverse deeper into the tree
                ParentNodeRef = GetNodeChild(ParentNodeRef, CurrentOct);
            }
        }
        else
        {
            return;
        }

        CurrentScale >>= 1;
        ParentCentre = GetOctantCentre(CurrentOct, CurrentScale, ParentCentre);
        CurrentOct = GetOctantForPosition(DeleteP, ParentCentre);
    }
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

extern "C" unsigned int
GetSvoUsedBlockCount(const svo* const Svo)
{
    return Svo->UsedBlockCount;
}

extern "C" unsigned int
GetSvoDepth(const svo* const Svo)
{
    return Svo->MaxDepth;
}

