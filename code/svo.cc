#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <intrin.h>
#include <vector>
#include <map>

#include "sabre.h"
#include "svo.h"
#include "vecmath.h"

using namespace vm;

static constexpr u16 CHILD_PTR_MSK = 0x7FFFU;
static constexpr u32 FAR_BIT_MSK   = 0x8000U;

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

enum voxel_type
{
    VOXEL_PARENT,
    VOXEL_LEAF
};

struct node_ref
{
    svo_block* Blk;
    svo_node*  Node;
};

struct frustum3
{
    vec3 Planes[6];
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


static inline packed_snorm3
PackVec3ToSnorm3(vec3 V)
{
    f32 Exp = 127.0f;

    i8 Sx = (i8)Round(Clamp(V.X, -1.0f, 1.0f) * Exp);
    i8 Sy = (i8)Round(Clamp(V.Y, -1.0f, 1.0f) * Exp);
    i8 Sz = (i8)Round(Clamp(V.Z, -1.0f, 1.0f) * Exp);

    packed_snorm3 Out = 0x00000000U;
    Out |= (u32)((u8)(Sz) << 16U);
    Out |= (u32)((u8)(Sy) <<  8U);
    Out |= (u32)((u8)(Sx));

    return Out;
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

static inline uvec3
FindHighestSetBit(uvec3 Msk)
{
    uvec3 Out;
    Out.X = FindHighestSetBit(Msk.X);
    Out.Y = FindHighestSetBit(Msk.Y);
    Out.Z = FindHighestSetBit(Msk.Z);

    return Out;
}

static inline u32
GetTreeMaxScaleBiased(const svo* const Tree)
{
    return (1U << Tree->ScaleExponent) << Tree->Bias.Scale;
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
    return (Node->ChildPtr & FAR_BIT_MSK) != 0;
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
    return (int)SBR_NODES_PER_BLK - (int)Blk->NextFreeSlot;
}


static inline bool
IsOctantOccupied(svo_node* ContainingNode, svo_oct Oct)
{
    return ContainingNode->OccupiedMask & (1U << Oct);
}

static inline bool
IsOctantLeaf(svo_node* ContainingNode, svo_oct Oct)
{
    return ContainingNode->LeafMask & (1U << Oct);
}

static inline void
SetOctantOccupied(svo_oct SubOctant, voxel_type Type, svo_node* OutEntry)
{
    u32 OctMsk = 1U << SubOctant;

    OutEntry->OccupiedMask |= OctMsk;

    if (VOXEL_LEAF == Type)
    {
       OutEntry->LeafMask |= OctMsk;
    }
    else
    {
       OutEntry->LeafMask &= ~OctMsk;
    }
}


extern "C" svo_bias
ComputeScaleBias(u32 MaxDepth, u32 ScaleExponent)
{
    if (MaxDepth > ScaleExponent)
    {
        u32 Bias = (MaxDepth - ScaleExponent);
        f32 InvBias = 1.0f / (static_cast<f32>(1U << Bias));

        return svo_bias{ InvBias, Bias };
    }
    else
    {
        return svo_bias{ 1.0f, 0 };
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
        i32 BlksFromFirst = (i32)((FirstChildIndex + ChildOffset) / SBR_NODES_PER_BLK);

        BlksToJump = BlksFromParent + BlksFromFirst;
    }
    else
    {
        FirstChildIndex = ParentRef.Node->ChildPtr & CHILD_PTR_MSK;
        i32 BlksFromFirst = (i32)((FirstChildIndex + ChildOffset) / SBR_NODES_PER_BLK);

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

    u32 ChildIndex = (FirstChildIndex + ChildOffset) % SBR_NODES_PER_BLK;

    return node_ref{ ChildBlk, &ChildBlk->Entries[ChildIndex] };
}


static far_ptr*
AllocateFarPtr(svo_block* const ContainingBlk)
{
    usize NextFarPtrSlot = ContainingBlk->NextFarPtrSlot;
    assert(NextFarPtrSlot < SBR_FAR_PTRS_PER_BLK);
    far_ptr* Ptr = &ContainingBlk->FarPtrs[NextFarPtrSlot];
    ++ContainingBlk->NextFarPtrSlot;

    assert(Ptr->NodeOffset == 0);
    assert(Ptr->BlkIndex == 0);

    return Ptr;
}

static inline void
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
        assert(nullptr != FarPtr);
        FarPtr->BlkIndex = (u32)ChildRef.Blk->Index;
        FarPtr->NodeOffset = ChildPtrBits;

        // Set the child ptr value to the far bit with the remaining 15 bits
        // set as the index into the far ptrs storage block.
        ParentRef.Node->ChildPtr = FAR_BIT_MSK | u16(ParentRef.Blk->NextFarPtrSlot - 1);
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
    uvec3 G = static_cast<uvec3>(GreaterThan(P, ParentCentreP));

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
        ++NewBlk->NextFreeSlot;

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
BuildSubOctreeRecursive(svo_node* Parent,
        svo* Tree,
        svo_oct RootOct,
        u32 Depth,
        u32 Scale,
        svo_block* ParentBlk,
        vec3 Centre,
        shape_sampler* ShapeSampler,
        data_sampler* NormalSampler,
        data_sampler* ColourSampler)
{
    struct node_child
    {
        svo_oct Oct;
        vec3    Centre;
        svo_node* Node;
        svo_block* Blk;
    };

    u32 NextScale = Scale >> 1U;
    u32 NextDepth = Depth + 1;

    node_child Children[8];
    u32 LastChildIndex = 0;

    uvec3 Radius = uvec3(Scale >> 1U);

    const node_ref ParentRef{ ParentBlk, Parent };

    for (u32 Oct = 0; Oct < 8; ++Oct)
    {
        // Multiplying by the InvBias here transforms the octant cubes back 
        // into "real" space from the scaled space we operate in when
        // constructing the tree.
        vec3 OctCentre = GetOctantCentre((svo_oct)Oct, Scale, Centre);
        vec3 OctMin = (OctCentre - vec3(Radius)) * Tree->Bias.InvScale;
        vec3 OctMax = (OctCentre + vec3(Radius)) * Tree->Bias.InvScale;

        bool Intersected = ShapeSampler->SamplerFn(OctMin, OctMax, Tree, ShapeSampler->UserData);
        if (Intersected)
        {
            // Check if the NEXT depth is less than the tree max
            // depth.
            if ((Depth + 1) < Tree->MaxDepth)
            {
                // Need to subdivide
                SetOctantOccupied((svo_oct)Oct, VOXEL_PARENT, Parent);

                // Allocate a new child node for this octant.
                // First, attempt to allocate within the same block
                const node_ref ChildRef = AllocateNewNode(Tree->LastBlock, Tree);

                // If the parent's child pointer has already been set, we do not
                // need to set it again. Child pointers should only point to the
                // first child.
                if (0x0000 == Parent->ChildPtr)
                {
                    LinkParentAndChildNodes(ParentRef, ChildRef);
                }

                assert(nullptr != ChildRef.Node);
                assert(nullptr != ChildRef.Blk);

                Children[LastChildIndex] = { (svo_oct)Oct, OctCentre, ChildRef.Node, ChildRef.Blk };
                ++LastChildIndex;
            }
            else
            {
                SetOctantOccupied((svo_oct)Oct, VOXEL_LEAF, Parent);

                vec3 Normal = NormalSampler->SamplerFn(OctCentre, Tree, NormalSampler->UserData);
                vec3 Colour = ColourSampler->SamplerFn(OctCentre, Tree, ColourSampler->UserData);
                
                Tree->AttribData.emplace_back(HashVec3(uvec3(OctCentre)), PackVec3ToSnorm3(Normal),  PackVec3ToSnorm3(Colour));
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
                                ShapeSampler,
                                NormalSampler,
                                ColourSampler);
    }
}

extern "C" svo*
CreateScene(u32 ScaleExp,
            u32 MaxDepth,
            shape_sampler* ShapeSampler,
            data_sampler* NormalSampler,
            data_sampler* ColourSampler)

{
    svo* Tree = (svo*)calloc(1, sizeof(svo));
    
    if (Tree)
    {
        Tree->ScaleExponent = ScaleExp;
        Tree->MaxDepth = MaxDepth;

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
        u32 RootScale = (1U << ScaleExp);
        
        Tree->Bias = ComputeScaleBias(MaxDepth, ScaleExp);
        Tree->AttribData = std::vector<attrib_data>();

        // Scale up by the bias
        RootScale <<= Tree->Bias.Scale;

        printf("ROOTSCALE %u\n", RootScale);

        vec3 RootCentre = vec3(RootScale >> 1U);

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
                                ShapeSampler,
                                NormalSampler,
                                ColourSampler);

        printf("Completed tree\n");
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
            NewNode->ChildPtr = FAR_BIT_MSK | u16(NewBlk->NextFarPtrSlot - 1);
        }
        else
        {
            // Need to convert local pointer into far pointer.
            u16 OldChildPtr = NodeRef.Node->ChildPtr;
            far_ptr* NewFarPtr = AllocateFarPtr(NewBlk);

            NewFarPtr->BlkIndex = NodeRef.Blk->Index;
            NewFarPtr->NodeOffset = OldChildPtr;
            NewNode->ChildPtr = FAR_BIT_MSK | u16(NewBlk->NextFarPtrSlot - 1);
        }
    }

    Result.Node = NewNode;
    Result.Blk = NewBlk;

    return Result;
}


static node_ref
InsertChild(node_ref ParentRef, svo_oct ChildOct, svo* Tree, voxel_type Type)
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
    if (nullptr != CreatedNode.Node)
    {
        return CreatedNode;
    }
    else
    {
        return ParentRef;
    }

}


extern "C" void
InsertVoxel(svo* Tree, vec3 P)
{
    // Scale of the smallest voxel in the SVO. For biased
    // trees, this will always be 1. For non-biased trees,
    // this will be the scale at the tree's MaxDepth.
    u32 TreeMinScale = GetTreeMinScaleBiased(Tree);
    u32 VoxelScale = TreeMinScale;

    // Obtain the original root scale of the tree, though
    // this may need to be biased further if the inserted
    // voxel scale is smaller than the tree minimum scale.
    u32 RootScale = GetTreeMaxScaleBiased(Tree);

    // Inserted voxel position, scaled by the tree bias.
    vec3 InsertP = P * static_cast<f32>(1U << Tree->Bias.Scale);

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
    u32 EditScale = (VoxelScale << 1);

    // Beginning at the root scale, descend the tree until we get
    // to the desired scale, or we hit a leaf octant (which means
    // we can't go any further).
    u32 CurrentScale = RootScale >> 1;

    while (CurrentScale > EditScale)
    {
        if (IsOctantOccupied(ParentRef.Node, CurrentOct))
        {
            if (IsOctantLeaf(ParentRef.Node, CurrentOct))
            {
                return;
            }
            else
            {
                ParentRef = GetNodeChild(ParentRef, CurrentOct);
            }
        }
        else
        {
            // If the current octant isn't occupied, we will need to build a new subtree
            // containing our inserted voxel.
            ParentRef = InsertChild(ParentRef, CurrentOct, Tree, VOXEL_PARENT);
            AllocatedParent = true;
        }

        ParentCentreP = GetOctantCentre(CurrentOct, CurrentScale, ParentCentreP);
        CurrentOct = GetOctantForPosition(InsertP, ParentCentreP);
        CurrentScale >>= 1;
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
    u32 LeavesMsk = ~(1U << ChildOct);

    // Clear this oct's bit from the parent leaf
    // mask. If this octant was a leaf, it will
    // already be occupied.
    ParentRef.Node->LeafMask &= LeavesMsk;

    // Set all octants occupied in the newly
    // allocated child.
    NewChildRef.Node->OccupiedMask = 0xFF;

    // Set all octants inside the newly allocated
    // child to 
    NewChildRef.Node->LeafMask = (u8)LeavesMsk;

    return NewChildRef;
}

struct ray_intersection
{
    float tMin;
    float tMax;
    vec3 tMaxV;
    vec3 tMinV;
};

static ray_intersection
ComputeRayBoxIntersection(vec3 ROrigin, vec3 RInvDir, vec3 vMin, vec3 vMax)
{
    vec3 t0 = (vMin - ROrigin) * RInvDir; // Distance along ray to vmin planes
    vec3 t1 = (vMax - ROrigin) * RInvDir; // Distance along ray to vmax planes

    vec3 tMin = Minimum(t0, t1); // Minimums of all distances
    vec3 tMax = Maximum(t0, t1); // Maximums of all distances

    float ttMin = HorzMax(tMin); // Largest of the min distances (closest to box)
    float ttMax = HorzMin(tMax); // Smallest of max distances (closest to box)

    ray_intersection Result = { ttMin, ttMax, tMax, tMin };
    return Result;
}

struct st_frame
{
    node_ref Node;
    uint Scale;
    vec3 ParentCentre;
};

static bool
IsAdvanceValid(svo_oct NewOct, svo_oct OldOct, vec3 RaySgn)
{
    ivec3 NewOctBits = ivec3(uvec3{1, 2, 4} & static_cast<u32>(NewOct));
    ivec3 OldOctBits = ivec3(uvec3{1, 2, 4} & static_cast<u32>(OldOct));

    vec3 OctSgn = Sign(vec3(NewOctBits - OldOctBits));

    return Any(Equal(RaySgn, OctSgn));
}

static svo_oct
GetNextOctant(float tMax, vec3 tMaxV, svo_oct CurrentOct)
{
    uvec3 XorMsk3 = uvec3(GreaterThanEqual(vec3(tMax), tMaxV)) * uvec3(1, 2, 4);

    uint XorMsk = XorMsk3.X + XorMsk3.Y + XorMsk3.Z;

    return svo_oct(CurrentOct ^ XorMsk);
}

extern vec3
GetNearestFreeSlot(vec3 RayOrigin, vec3 RayDir, const svo* const Tree)
{
    u32 MaxScale = GetTreeMaxScaleBiased(Tree);

    vec3 RaySgn = Sign(RayDir);
    st_frame Stack[64 + 1];
    uint Scale = MaxScale;
    vec3 RayInvDir = Reciprocal(RayDir);
    vec3 RootMin = vec3(0);
    vec3 RootMax = vec3(Scale) * Tree->Bias.InvScale;
    const float BiasScale = (1.0f / Tree->Bias.InvScale);
    vec3 LastCentre = vec3(0);

    ray_intersection CurrentIntersection = ComputeRayBoxIntersection(RayOrigin, RayInvDir, RootMin, RootMax);
    if (CurrentIntersection.tMin <= CurrentIntersection.tMax)    // Raycast to find voxel position
    {
        vec3 RayP = (CurrentIntersection.tMin >= 0) ? RayOrigin + (RayDir *CurrentIntersection.tMin) : RayOrigin;
        vec3 ParentCentre = vec3(Scale >> 1);
        svo_oct CurrentOct = GetOctantForPosition(RayP, ParentCentre*Tree->Bias.InvScale);
        node_ref ParentNodeRef = GetTreeRootNodeRef(Tree);
        uint CurrentDepth = 1;
        Scale >>= 1;

        Stack[CurrentDepth] = { ParentNodeRef, Scale, ParentCentre };

        for (int Step = 0; Step < 64; ++Step)
        {
            vec3 Rad = vec3(Scale >> 1);
            vec3 NodeCentre = GetOctantCentre(CurrentOct, Scale, ParentCentre);
            vec3 NodeMin = (NodeCentre - Rad) * Tree->Bias.InvScale;
            vec3 NodeMax = (NodeCentre + Rad) * Tree->Bias.InvScale;

            CurrentIntersection = ComputeRayBoxIntersection(RayOrigin, RayInvDir, NodeMin, NodeMax);

            if (CurrentIntersection.tMin <= CurrentIntersection.tMax)
            {
                if (IsOctantOccupied(ParentNodeRef.Node, CurrentOct))
                {
                    // {{{ 
                    if (IsOctantLeaf(ParentNodeRef.Node, CurrentOct))
                    {
                        // Leaf hit; return nodecentre.
                        // Return centre of the *previous* oct we visited
                        //return NodeCentre * Tree->Bias.InvScale;
                        return LastCentre * Tree->Bias.InvScale;
                    }
                    else
                    {
                        Stack[CurrentDepth] = { ParentNodeRef, Scale, ParentCentre };

                        ParentNodeRef = GetNodeChild(ParentNodeRef, CurrentOct);
                        CurrentOct = GetOctantForPosition(RayP, NodeCentre*Tree->Bias.InvScale);
                        ParentCentre = NodeCentre;
                        Scale >>= 1;
                        ++CurrentDepth;

                        continue;
                    }
                    // }}}
                }

                RayP = RayOrigin + RayDir*(CurrentIntersection.tMax + 0.015625f);
                const svo_oct NextOct = GetNextOctant(CurrentIntersection.tMax, CurrentIntersection.tMaxV, CurrentOct);

                if (IsAdvanceValid(NextOct, CurrentOct, RaySgn))
                {
                    CurrentOct = NextOct;
                    LastCentre = NodeCentre;
                }
                else
                {
                    // {{{ 
                    // Determined that NodeCentre is never < 0
                    uvec3 NodeCentreBits = uvec3(NodeCentre);
                    uvec3 RayPBits = uvec3(RayP * BiasScale);

                    // NOTE(Liam): It is **okay** to have negative values here
                    // because the HDB will end up being equal to ScaleExponentUniform.
                    //
                    // Find the highest differing bit
                    uvec3 HDB = FindHighestSetBit(NodeCentreBits ^ RayPBits);
                    bvec3 B = LessThan(HDB, uvec3(Tree->ScaleExponent + Tree->Bias.Scale));

                    uint M = HorzMax(Select(HDB, uvec3(0), B));

                    uint NextDepth = ((Tree->ScaleExponent + Tree->Bias.Scale) - M);

                    if (NextDepth <= 64 && NextDepth < CurrentDepth)
                    {
                        CurrentDepth = NextDepth;
                        Scale = Stack[CurrentDepth].Scale;
                        ParentCentre = Stack[CurrentDepth].ParentCentre;
                        ParentNodeRef = Stack[CurrentDepth].Node;

                        CurrentOct = GetOctantForPosition(RayP, ParentCentre*Tree->Bias.InvScale);
                    }
                    else
                    {
                        break;
                    }
                    /// }}}
                }
            }
        }

    }

    printf("Position not found\n");

    return vec3(FLT_MAX);
}

extern vec3
GetNearestLeafSlot(vec3 RayOrigin, vec3 RayDir, const svo* const Tree)
{
    u32 MaxScale = GetTreeMaxScaleBiased(Tree);

    vec3 RaySgn = Sign(RayDir);
    st_frame Stack[64 + 1];
    uint Scale = MaxScale;
    vec3 RayInvDir = Reciprocal(RayDir);
    vec3 RootMin = vec3(0);
    vec3 RootMax = vec3(Scale) * Tree->Bias.InvScale;
    const float BiasScale = (1.0f / Tree->Bias.InvScale);

    ray_intersection CurrentIntersection = ComputeRayBoxIntersection(RayOrigin, RayInvDir, RootMin, RootMax);
    if (CurrentIntersection.tMin <= CurrentIntersection.tMax)    // Raycast to find voxel position
    {
        vec3 RayP = (CurrentIntersection.tMin >= 0) ? RayOrigin + (RayDir * CurrentIntersection.tMin) : RayOrigin;
        vec3 ParentCentre = vec3(Scale >> 1);
        svo_oct CurrentOct = GetOctantForPosition(RayP, ParentCentre*Tree->Bias.InvScale);
        node_ref ParentNodeRef = GetTreeRootNodeRef(Tree);
        uint CurrentDepth = 1;
        Scale >>= 1;

        Stack[CurrentDepth] = { ParentNodeRef, Scale, ParentCentre };

        for (int Step = 0; Step < 64; ++Step)
        {
            vec3 Rad = vec3(Scale >> 1);
            vec3 NodeCentre = GetOctantCentre(CurrentOct, Scale, ParentCentre);
            vec3 NodeMin = (NodeCentre - Rad) * Tree->Bias.InvScale;
            vec3 NodeMax = (NodeCentre + Rad) * Tree->Bias.InvScale;

            CurrentIntersection = ComputeRayBoxIntersection(RayOrigin, RayInvDir, NodeMin, NodeMax);

            if (CurrentIntersection.tMin <= CurrentIntersection.tMax)
            {
                if (IsOctantOccupied(ParentNodeRef.Node, CurrentOct))
                {
                    // {{{ 
                    if (IsOctantLeaf(ParentNodeRef.Node, CurrentOct))
                    {
                        // Leaf hit; return nodecentre.
                        return NodeCentre * Tree->Bias.InvScale;
                    }
                    else
                    {
                        Stack[CurrentDepth] = { ParentNodeRef, Scale, ParentCentre };

                        ParentNodeRef = GetNodeChild(ParentNodeRef, CurrentOct);
                        CurrentOct = GetOctantForPosition(RayP, NodeCentre*Tree->Bias.InvScale);
                        ParentCentre = NodeCentre;
                        Scale >>= 1;
                        ++CurrentDepth;

                        continue;
                    }
                    // }}}
                }

                RayP = RayOrigin + RayDir*(CurrentIntersection.tMax + 0.015625f);
                const svo_oct NextOct = GetNextOctant(CurrentIntersection.tMax, CurrentIntersection.tMaxV, CurrentOct);

                if (IsAdvanceValid(NextOct, CurrentOct, RaySgn))
                {
                    CurrentOct = NextOct;
                }
                else
                {
                    // {{{ 
                    // Determined that NodeCentre is never < 0
                    uvec3 NodeCentreBits = uvec3(NodeCentre);
                    uvec3 RayPBits = uvec3(RayP * BiasScale);

                    // NOTE(Liam): It is **okay** to have negative values here
                    // because the HDB will end up being equal to ScaleExponentUniform.
                    //
                    // Find the highest differing bit
                    uvec3 HDB = FindHighestSetBit(NodeCentreBits ^ RayPBits);
                    bvec3 B = LessThan(HDB, uvec3(Tree->ScaleExponent + Tree->Bias.Scale));

                    uint M = HorzMax(Select(HDB, uvec3(0), B));

                    uint NextDepth = ((Tree->ScaleExponent + Tree->Bias.Scale) - M);

                    if (NextDepth <= 64 && NextDepth < CurrentDepth)
                    {
                        CurrentDepth = NextDepth;
                        Scale = Stack[CurrentDepth].Scale;
                        ParentCentre = Stack[CurrentDepth].ParentCentre;
                        ParentNodeRef = Stack[CurrentDepth].Node;

                        CurrentOct = GetOctantForPosition(RayP, ParentCentre*Tree->Bias.InvScale);
                    }
                    else
                    {
                        break;
                    }
                    /// }}}
                }
            }
        }

    }

    printf("Position not found\n");

    return vec3(FLT_MAX);
}


extern "C" void
DeleteVoxel(svo* Tree, vec3 VoxelP)
{
    // Scale the voxel position by the tree bias
    vec3 DeleteP = VoxelP * (float)((1U << Tree->Bias.Scale));

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
    u32 LeafScale = MinScale << 1;

    // Descend the tree until we get to the minium scale.
    while (CurrentScale >= LeafScale)
    {
        CurrentScale >>= 1;
        // If we had previously created a child node, we need to
        // continue building the tree until we reach the min scale.
        if (CreatedChild && CurrentScale > LeafScale)
        {
            node_ref NewParentRef = AllocateNewNode(Tree->LastBlock, Tree);

            u32 OctMsk = ~(1U << CurrentOct);
            NewParentRef.Node->OccupiedMask = 0xFF;   // All octants occupied
            NewParentRef.Node->LeafMask = (u8)OctMsk; // All octants except current leaves

            LinkParentAndChildNodes(ParentNodeRef, NewParentRef);

            ParentNodeRef = NewParentRef;
        }
        else if (IsOctantOccupied(ParentNodeRef.Node, CurrentOct))
        {
            if (CurrentScale <= LeafScale)
            {
                u32 ClearMsk = ~(1U << CurrentOct);
                ParentNodeRef.Node->LeafMask &= ClearMsk;
                ParentNodeRef.Node->OccupiedMask &= ClearMsk;

                return;
            }

            if (IsOctantLeaf(ParentNodeRef.Node, CurrentOct))
            {
                ParentNodeRef = DeleteLeafChild(CurrentOct, Tree, ParentNodeRef);
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

        ParentCentre = GetOctantCentre(CurrentOct, CurrentScale, ParentCentre);
        CurrentOct = GetOctantForPosition(DeleteP, ParentCentre);
    }
}

extern "C" void
DeleteScene(svo* Tree)
{
    if (nullptr != Tree)
    {
        svo_block* CurrentBlk = Tree->RootBlock;

        while (nullptr != CurrentBlk)
        {
            svo_block* NextBlk = CurrentBlk->Next;
            free(CurrentBlk);
            CurrentBlk = NextBlk;
        }

        free(Tree);
    }
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

static inline bool
BoxFrustumIntersection(const frustum3* const F, vec3 Min, vec3 Max)
{
    for( int Plane = 0; Plane < 6; Plane++ )
    {
        int Out = 0;
        Out += ((Dot(F->Planes[Plane], vec3(Min.X, Min.Y, Min.Z)) < 0.0f)?1:0);
        Out += ((Dot(F->Planes[Plane], vec3(Max.X, Min.Y, Min.Z)) < 0.0f)?1:0);
        Out += ((Dot(F->Planes[Plane], vec3(Min.X, Max.Y, Min.Z)) < 0.0f)?1:0);
        Out += ((Dot(F->Planes[Plane], vec3(Max.X, Max.Y, Min.Z)) < 0.0f)?1:0);
        Out += ((Dot(F->Planes[Plane], vec3(Min.X, Min.Y, Max.Z)) < 0.0f)?1:0);
        Out += ((Dot(F->Planes[Plane], vec3(Max.X, Min.Y, Max.Z)) < 0.0f)?1:0);
        Out += ((Dot(F->Planes[Plane], vec3(Min.X, Max.Y, Max.Z)) < 0.0f)?1:0);
        Out += ((Dot(F->Planes[Plane], vec3(Max.X, Max.Y, Max.Z)) < 0.0f)?1:0);
        if(8 == Out) return false;
    }

    return true;
}

#if 0
static inline void
ComputeFrustumFromEyeAndDir(vec3 Eye, vec3 Dir, frustum3* const FOut)
{
    vec3 TopRightNear = vec3(Eye.X + 256, Eye.Y + 256, 
}
#endif

#if 0
extern "C" void
HighestVisibleAncestor(const svo* const Svo, vec3 EyePos, vec3 EyeDir)
{
    frustum3 ViewFrustum;
    ComputeFrustumFromEyeAndDir(EyePos, EyeDir, &ViewFrustum);
}
#endif


