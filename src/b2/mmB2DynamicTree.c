/*
-----------------------------------------------------------------------------
MIT License

Copyright (c) 2019 Erin Catto
Copyright (c) 2023-2023 mm_longcheng@icloud.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
*/

#include "mmB2DynamicTree.h"
#include "mmB2GrowableStack.h"
#include "mmB2Collision.h"

#include <assert.h>
#include <string.h>
#include <stdlib.h>

// Allocate a node from the pool. Grow the pool if necessary.
static
int32 
b2DynamicTreeAllocateNode(
    struct b2DynamicTree* p)
{
    int32 nodeId;

    // Expand the node pool as needed.
    if (p->m_freeList == b2_nullNode)
    {
        int32 i;

        struct b2TreeNode* oldNodes;

        b2Assert(p->m_nodeCount == p->m_nodeCapacity);

        // The free list is empty. Rebuild a bigger pool.
        oldNodes = p->m_nodes;
        p->m_nodeCapacity *= 2;
        p->m_nodes = (struct b2TreeNode*)b2Alloc(p->m_nodeCapacity * sizeof(struct b2TreeNode));
        memcpy(p->m_nodes, oldNodes, p->m_nodeCount * sizeof(struct b2TreeNode));
        b2Free(oldNodes);

        // Build a linked list for the free list. The parent
        // pointer becomes the "next" pointer.
        for (i = p->m_nodeCount; i < p->m_nodeCapacity - 1; ++i)
        {
            p->m_nodes[i].next = i + 1;
            p->m_nodes[i].height = -1;
        }
        p->m_nodes[p->m_nodeCapacity - 1].next = b2_nullNode;
        p->m_nodes[p->m_nodeCapacity - 1].height = -1;
        p->m_freeList = p->m_nodeCount;
    }

    // Peel a node off the free list.
    nodeId = p->m_freeList;
    p->m_freeList = p->m_nodes[nodeId].next;
    p->m_nodes[nodeId].parent = b2_nullNode;
    p->m_nodes[nodeId].child1 = b2_nullNode;
    p->m_nodes[nodeId].child2 = b2_nullNode;
    p->m_nodes[nodeId].height = 0;
    p->m_nodes[nodeId].userData = NULL;
    p->m_nodes[nodeId].moved = b2False;
    ++p->m_nodeCount;
    return nodeId;
}

// Return a node to the pool.
static
void 
b2DynamicTreeFreeNode(
    struct b2DynamicTree* p,
    int32 nodeId)
{
    b2Assert(0 <= nodeId && nodeId < p->m_nodeCapacity);
    b2Assert(0 < p->m_nodeCount);
    p->m_nodes[nodeId].next = p->m_freeList;
    p->m_nodes[nodeId].height = -1;
    p->m_freeList = nodeId;
    --p->m_nodeCount;
}

// Perform a left or right rotation if node A is imbalanced.
// Returns the new root index.
static
int32
b2DynamicTreeBalance(
    struct b2DynamicTree* p,
    int32 iA)
{
    struct b2TreeNode* A;

    int32 iB;
    int32 iC;

    struct b2TreeNode* B;
    struct b2TreeNode* C;

    int32 balance;

    b2Assert(iA != b2_nullNode);

    A = p->m_nodes + iA;
    if (b2TreeNodeIsLeaf(A) || A->height < 2)
    {
        return iA;
    }

    iB = A->child1;
    iC = A->child2;
    b2Assert(0 <= iB && iB < p->m_nodeCapacity);
    b2Assert(0 <= iC && iC < p->m_nodeCapacity);

    B = p->m_nodes + iB;
    C = p->m_nodes + iC;

    balance = C->height - B->height;

    // Rotate C up
    if (balance > 1)
    {
        int32 iF = C->child1;
        int32 iG = C->child2;
        struct b2TreeNode* F = p->m_nodes + iF;
        struct b2TreeNode* G = p->m_nodes + iG;
        b2Assert(0 <= iF && iF < p->m_nodeCapacity);
        b2Assert(0 <= iG && iG < p->m_nodeCapacity);

        // Swap A and C
        C->child1 = iA;
        C->parent = A->parent;
        A->parent = iC;

        // A's old parent should point to C
        if (C->parent != b2_nullNode)
        {
            if (p->m_nodes[C->parent].child1 == iA)
            {
                p->m_nodes[C->parent].child1 = iC;
            }
            else
            {
                b2Assert(p->m_nodes[C->parent].child2 == iA);
                p->m_nodes[C->parent].child2 = iC;
            }
        }
        else
        {
            p->m_root = iC;
        }

        // Rotate
        if (F->height > G->height)
        {
            C->child2 = iF;
            A->child2 = iG;
            G->parent = iA;
            b2AABBCombine2(&A->aabb, &B->aabb, &G->aabb);
            b2AABBCombine2(&C->aabb, &A->aabb, &F->aabb);

            A->height = 1 + b2MaxInt32(B->height, G->height);
            C->height = 1 + b2MaxInt32(A->height, F->height);
        }
        else
        {
            C->child2 = iG;
            A->child2 = iF;
            F->parent = iA;
            b2AABBCombine2(&A->aabb, &B->aabb, &F->aabb);
            b2AABBCombine2(&C->aabb, &A->aabb, &G->aabb);

            A->height = 1 + b2MaxInt32(B->height, F->height);
            C->height = 1 + b2MaxInt32(A->height, G->height);
        }

        return iC;
    }

    // Rotate B up
    if (balance < -1)
    {
        int32 iD = B->child1;
        int32 iE = B->child2;
        struct b2TreeNode* D = p->m_nodes + iD;
        struct b2TreeNode* E = p->m_nodes + iE;
        b2Assert(0 <= iD && iD < p->m_nodeCapacity);
        b2Assert(0 <= iE && iE < p->m_nodeCapacity);

        // Swap A and B
        B->child1 = iA;
        B->parent = A->parent;
        A->parent = iB;

        // A's old parent should point to B
        if (B->parent != b2_nullNode)
        {
            if (p->m_nodes[B->parent].child1 == iA)
            {
                p->m_nodes[B->parent].child1 = iB;
            }
            else
            {
                b2Assert(p->m_nodes[B->parent].child2 == iA);
                p->m_nodes[B->parent].child2 = iB;
            }
        }
        else
        {
            p->m_root = iB;
        }

        // Rotate
        if (D->height > E->height)
        {
            B->child2 = iD;
            A->child1 = iE;
            E->parent = iA;
            b2AABBCombine2(&A->aabb, &C->aabb, &E->aabb);
            b2AABBCombine2(&B->aabb, &A->aabb, &D->aabb);

            A->height = 1 + b2MaxInt32(C->height, E->height);
            B->height = 1 + b2MaxInt32(A->height, D->height);
        }
        else
        {
            B->child2 = iE;
            A->child1 = iD;
            D->parent = iA;
            b2AABBCombine2(&A->aabb, &C->aabb, &D->aabb);
            b2AABBCombine2(&B->aabb, &A->aabb, &E->aabb);

            A->height = 1 + b2MaxInt32(C->height, D->height);
            B->height = 1 + b2MaxInt32(A->height, E->height);
        }

        return iB;
    }

    return iA;
}

static
void 
b2DynamicTreeInsertLeaf(
    struct b2DynamicTree* p,
    int32 leaf)
{
    const struct b2AABB* leafAABB;
    int32 index;

    int32 sibling;
    int32 oldParent;
    int32 newParent;

    ++p->m_insertionCount;

    if (p->m_root == b2_nullNode)
    {
        p->m_root = leaf;
        p->m_nodes[p->m_root].parent = b2_nullNode;
        return;
    }

    // Find the best sibling for this node
    leafAABB = &p->m_nodes[leaf].aabb;
    index = p->m_root;
    while (b2TreeNodeIsLeaf(&p->m_nodes[index]) == b2False)
    {
        int32 child1;
        int32 child2;

        float area;

        struct b2AABB combinedAABB;

        float combinedArea;
        float cost;
        float inheritanceCost;
        float cost1;
        float cost2;

        child1 = p->m_nodes[index].child1;
        child2 = p->m_nodes[index].child2;

        area = b2AABBGetPerimeter(&p->m_nodes[index].aabb);

        b2AABBCombine2(&combinedAABB, &p->m_nodes[index].aabb, leafAABB);
        combinedArea = b2AABBGetPerimeter(&combinedAABB);

        // Cost of creating a new parent for this node and the new leaf
        cost = 2.0f * combinedArea;

        // Minimum cost of pushing the leaf further down the tree
        inheritanceCost = 2.0f * (combinedArea - area);

        // Cost of descending into child1
        if (b2TreeNodeIsLeaf(&p->m_nodes[child1]))
        {
            struct b2AABB aabb;
            b2AABBCombine2(&aabb, leafAABB, &p->m_nodes[child1].aabb);
            cost1 = b2AABBGetPerimeter(&aabb) + inheritanceCost;
        }
        else
        {
            float oldArea;
            float newArea;
            struct b2AABB aabb;
            b2AABBCombine2(&aabb, leafAABB, &p->m_nodes[child1].aabb);
            oldArea = b2AABBGetPerimeter(&p->m_nodes[child1].aabb);
            newArea = b2AABBGetPerimeter(&aabb);
            cost1 = (newArea - oldArea) + inheritanceCost;
        }

        // Cost of descending into child2
        if (b2TreeNodeIsLeaf(&p->m_nodes[child2]))
        {
            struct b2AABB aabb;
            b2AABBCombine2(&aabb, leafAABB, &p->m_nodes[child2].aabb);
            cost2 = b2AABBGetPerimeter(&aabb) + inheritanceCost;
        }
        else
        {
            float oldArea;
            float newArea;
            struct b2AABB aabb;
            b2AABBCombine2(&aabb, leafAABB, &p->m_nodes[child2].aabb);
            oldArea = b2AABBGetPerimeter(&p->m_nodes[child2].aabb);
            newArea = b2AABBGetPerimeter(&aabb);
            cost2 = newArea - oldArea + inheritanceCost;
        }

        // Descend according to the minimum cost.
        if (cost < cost1 && cost < cost2)
        {
            break;
        }

        // Descend
        if (cost1 < cost2)
        {
            index = child1;
        }
        else
        {
            index = child2;
        }
    }

    sibling = index;

    // Create a new parent.
    oldParent = p->m_nodes[sibling].parent;
    newParent = b2DynamicTreeAllocateNode(p);
    p->m_nodes[newParent].parent = oldParent;
    p->m_nodes[newParent].userData = NULL;
    b2AABBCombine2(&p->m_nodes[newParent].aabb, leafAABB, &p->m_nodes[sibling].aabb);
    p->m_nodes[newParent].height = p->m_nodes[sibling].height + 1;

    if (oldParent != b2_nullNode)
    {
        // The sibling was not the root.
        if (p->m_nodes[oldParent].child1 == sibling)
        {
            p->m_nodes[oldParent].child1 = newParent;
        }
        else
        {
            p->m_nodes[oldParent].child2 = newParent;
        }

        p->m_nodes[newParent].child1 = sibling;
        p->m_nodes[newParent].child2 = leaf;
        p->m_nodes[sibling].parent = newParent;
        p->m_nodes[leaf].parent = newParent;
    }
    else
    {
        // The sibling was the root.
        p->m_nodes[newParent].child1 = sibling;
        p->m_nodes[newParent].child2 = leaf;
        p->m_nodes[sibling].parent = newParent;
        p->m_nodes[leaf].parent = newParent;
        p->m_root = newParent;
    }

    // Walk back up the tree fixing heights and AABBs
    index = p->m_nodes[leaf].parent;
    while (index != b2_nullNode)
    {
        int32 child1;
        int32 child2;

        index = b2DynamicTreeBalance(p, index);

        child1 = p->m_nodes[index].child1;
        child2 = p->m_nodes[index].child2;

        b2Assert(child1 != b2_nullNode);
        b2Assert(child2 != b2_nullNode);

        p->m_nodes[index].height = 1 + b2MaxInt32(p->m_nodes[child1].height, p->m_nodes[child2].height);
        b2AABBCombine2(&p->m_nodes[index].aabb, &p->m_nodes[child1].aabb, &p->m_nodes[child2].aabb);

        index = p->m_nodes[index].parent;
    }

    //b2DynamicTreeValidate(p);
}

static
void 
b2DynamicTreeRemoveLeaf(
    struct b2DynamicTree* p,
    int32 leaf)
{
    int32 parent;
    int32 grandParent;
    int32 sibling;

    if (leaf == p->m_root)
    {
        p->m_root = b2_nullNode;
        return;
    }

    parent = p->m_nodes[leaf].parent;
    grandParent = p->m_nodes[parent].parent;
    if (p->m_nodes[parent].child1 == leaf)
    {
        sibling = p->m_nodes[parent].child2;
    }
    else
    {
        sibling = p->m_nodes[parent].child1;
    }

    if (grandParent != b2_nullNode)
    {
        int32 index;

        // Destroy parent and connect sibling to grandParent.
        if (p->m_nodes[grandParent].child1 == parent)
        {
            p->m_nodes[grandParent].child1 = sibling;
        }
        else
        {
            p->m_nodes[grandParent].child2 = sibling;
        }
        p->m_nodes[sibling].parent = grandParent;
        b2DynamicTreeFreeNode(p, parent);

        // Adjust ancestor bounds.
        index = grandParent;
        while (index != b2_nullNode)
        {
            int32 child1;
            int32 child2;

            index = b2DynamicTreeBalance(p, index);

            child1 = p->m_nodes[index].child1;
            child2 = p->m_nodes[index].child2;

            b2AABBCombine2(&p->m_nodes[index].aabb, &p->m_nodes[child1].aabb, &p->m_nodes[child2].aabb);
            p->m_nodes[index].height = 1 + b2MaxInt32(p->m_nodes[child1].height, p->m_nodes[child2].height);

            index = p->m_nodes[index].parent;
        }
    }
    else
    {
        p->m_root = sibling;
        p->m_nodes[sibling].parent = b2_nullNode;
        b2DynamicTreeFreeNode(p, parent);
    }

    //b2DynamicTreeValidate(p);
}

// Compute the height of a sub-tree.
static
int32 
b2DynamicTreeComputeHeightById(
    const struct b2DynamicTree* p,
    int32 nodeId)
{
    struct b2TreeNode* node;

    b2Assert(0 <= nodeId && nodeId < p->m_nodeCapacity);
    node = p->m_nodes + nodeId;

    if (b2TreeNodeIsLeaf(node))
    {
        return 0;
    }
    else
    {
        int32 height1;
        int32 height2;
        height1 = b2DynamicTreeComputeHeightById(p, node->child1);
        height2 = b2DynamicTreeComputeHeightById(p, node->child2);
        return 1 + b2MaxInt32(height1, height2);
    }
}

int32
b2DynamicTreeComputeHeight(
    const struct b2DynamicTree* p)
{
    int32 height = b2DynamicTreeComputeHeightById(p, p->m_root);
    return height;
}

void 
b2DynamicTreeValidateStructure(
    const struct b2DynamicTree* p,
    int32 index)
{
    const struct b2TreeNode* node;

    int32 child1;
    int32 child2;

    if (index == b2_nullNode)
    {
        return;
    }

    if (index == p->m_root)
    {
        b2Assert(p->m_nodes[index].parent == b2_nullNode);
    }

    node = p->m_nodes + index;

    child1 = node->child1;
    child2 = node->child2;

    if (b2TreeNodeIsLeaf(node))
    {
        b2Assert(child1 == b2_nullNode);
        b2Assert(child2 == b2_nullNode);
        b2Assert(node->height == 0);
        return;
    }

    b2Assert(0 <= child1 && child1 < p->m_nodeCapacity);
    b2Assert(0 <= child2 && child2 < p->m_nodeCapacity);

    b2Assert(p->m_nodes[child1].parent == index);
    b2Assert(p->m_nodes[child2].parent == index);

    b2DynamicTreeValidateStructure(p, child1);
    b2DynamicTreeValidateStructure(p, child2);
}

void 
b2DynamicTreeValidateMetrics(
    const struct b2DynamicTree* p,
    int32 index)
{
    const struct b2TreeNode* node;

    int32 child1;
    int32 child2;

    int32 height1;
    int32 height2;
    int32 height;

    struct b2AABB aabb;

    if (index == b2_nullNode)
    {
        return;
    }

    node = p->m_nodes + index;

    child1 = node->child1;
    child2 = node->child2;

    if (b2TreeNodeIsLeaf(node))
    {
        b2Assert(child1 == b2_nullNode);
        b2Assert(child2 == b2_nullNode);
        b2Assert(node->height == 0);
        return;
    }

    b2Assert(0 <= child1 && child1 < p->m_nodeCapacity);
    b2Assert(0 <= child2 && child2 < p->m_nodeCapacity);

    height1 = p->m_nodes[child1].height;
    height2 = p->m_nodes[child2].height;
    height = 1 + b2MaxInt32(height1, height2);
    b2Assert(node->height == height);

    b2AABBCombine2(&aabb, &p->m_nodes[child1].aabb, &p->m_nodes[child2].aabb);

    b2Assert(aabb.lowerBound == node->aabb.lowerBound);
    b2Assert(aabb.upperBound == node->aabb.upperBound);

    b2DynamicTreeValidateMetrics(p, child1);
    b2DynamicTreeValidateMetrics(p, child2);
}

B2_API
void
b2DynamicTreeInit(
    struct b2DynamicTree* p)
{
    int32 i;

    p->m_root = b2_nullNode;

    p->m_nodeCapacity = 16;
    p->m_nodeCount = 0;
    p->m_nodes = (struct b2TreeNode*)b2Alloc(p->m_nodeCapacity * sizeof(struct b2TreeNode));
    memset(p->m_nodes, 0, p->m_nodeCapacity * sizeof(struct b2TreeNode));

    // Build a linked list for the free list.
    for (i = 0; i < p->m_nodeCapacity - 1; ++i)
    {
        p->m_nodes[i].next = i + 1;
        p->m_nodes[i].height = -1;
    }
    p->m_nodes[p->m_nodeCapacity - 1].next = b2_nullNode;
    p->m_nodes[p->m_nodeCapacity - 1].height = -1;
    p->m_freeList = 0;

    p->m_insertionCount = 0;
}

B2_API
void
b2DynamicTreeDestroy(
    struct b2DynamicTree* p)
{
    // This frees the entire tree in one shot.
    b2Free(p->m_nodes);
}

// Create a proxy in the tree as a leaf node. We return the index
// of the node instead of a pointer so that we can grow
// the node pool.
B2_API
int32
b2DynamicTreeCreateProxy(
    struct b2DynamicTree* p,
    const struct b2AABB* aabb,
    void* userData)
{
    int32 proxyId = b2DynamicTreeAllocateNode(p);

    // Fatten the aabb.
    b2Vec2 r = { b2_aabbExtension, b2_aabbExtension };
    b2Vec2Sub(p->m_nodes[proxyId].aabb.lowerBound, aabb->lowerBound, r);
    b2Vec2Add(p->m_nodes[proxyId].aabb.upperBound, aabb->upperBound, r);
    p->m_nodes[proxyId].userData = userData;
    p->m_nodes[proxyId].height = 0;
    p->m_nodes[proxyId].moved = b2True;

    b2DynamicTreeInsertLeaf(p, proxyId);

    return proxyId;
}

B2_API
void
b2DynamicTreeDeleteProxy(
    struct b2DynamicTree* p,
    int32 proxyId)
{
    b2Assert(0 <= proxyId && proxyId < p->m_nodeCapacity);
    b2Assert(b2TreeNodeIsLeaf(&p->m_nodes[proxyId]));

    b2DynamicTreeRemoveLeaf(p, proxyId);
    b2DynamicTreeFreeNode(p, proxyId);
}

B2_API
int
b2DynamicTreeMoveProxy(
    struct b2DynamicTree* p,
    int32 proxyId,
    const struct b2AABB* aabb,
    const b2Vec2 displacement)
{
    b2Vec2 d;
    struct b2AABB fatAABB;
    b2Vec2 r = { b2_aabbExtension, b2_aabbExtension };

    const struct b2AABB* treeAABB;

    b2Assert(0 <= proxyId && proxyId < p->m_nodeCapacity);

    b2Assert(b2TreeNodeIsLeaf(&p->m_nodes[proxyId]));

    // Extend AABB
    b2Vec2Sub(fatAABB.lowerBound, aabb->lowerBound, r);
    b2Vec2Add(fatAABB.upperBound, aabb->upperBound, r);

    // Predict AABB movement
    b2Vec2Scale(d, displacement, b2_aabbMultiplier);

    if (d[0] < 0.0f)
    {
        fatAABB.lowerBound[0] += d[0];
    }
    else
    {
        fatAABB.upperBound[0] += d[0];
    }

    if (d[1] < 0.0f)
    {
        fatAABB.lowerBound[1] += d[1];
    }
    else
    {
        fatAABB.upperBound[1] += d[1];
    }

    treeAABB = &p->m_nodes[proxyId].aabb;
    if (b2AABBContains(treeAABB, aabb))
    {
        // The tree AABB still contains the object, but it might be too large.
        // Perhaps the object was moving fast but has since gone to sleep.
        // The huge AABB is larger than the new fat AABB.
        b2Vec2 v;
        struct b2AABB hugeAABB;
        b2Vec2Scale(v, r, 4.0f);
        b2Vec2Sub(hugeAABB.lowerBound, fatAABB.lowerBound, v);
        b2Vec2Add(hugeAABB.upperBound, fatAABB.upperBound, v);

        if (b2AABBContains(&hugeAABB, treeAABB))
        {
            // The tree AABB contains the object AABB and the tree AABB is
            // not too large. No tree update needed.
            return b2False;
        }

        // Otherwise the tree AABB is huge and needs to be shrunk
    }

    b2DynamicTreeRemoveLeaf(p, proxyId);

    p->m_nodes[proxyId].aabb = fatAABB;

    b2DynamicTreeInsertLeaf(p, proxyId);

    p->m_nodes[proxyId].moved = b2True;

    return b2True;
}

B2_API
void*
b2DynamicTreeGetUserData(
    const struct b2DynamicTree* p,
    int32 proxyId)
{
    b2Assert(0 <= proxyId && proxyId < p->m_nodeCapacity);
    return p->m_nodes[proxyId].userData;
}

B2_API
int
b2DynamicTreeWasMoved(
    const struct b2DynamicTree* p,
    int32 proxyId)
{
    b2Assert(0 <= proxyId && proxyId < p->m_nodeCapacity);
    return p->m_nodes[proxyId].moved;
}

B2_API
void
b2DynamicTreeClearMoved(
    struct b2DynamicTree* p,
    int32 proxyId)
{
    b2Assert(0 <= proxyId && proxyId < p->m_nodeCapacity);
    p->m_nodes[proxyId].moved = b2False;
}

B2_API
const struct b2AABB*
b2DynamicTreeGetFatAABB(
    const struct b2DynamicTree* p,
    int32 proxyId)
{
    b2Assert(0 <= proxyId && proxyId < p->m_nodeCapacity);
    return &p->m_nodes[proxyId].aabb;
}

B2_API
void
b2DynamicTreeValidate(
    const struct b2DynamicTree* p)
{
#if defined(b2DEBUG)
    int32 freeCount;
    int32 freeIndex;

    b2DynamicTreeValidateStructure(p, p->m_root);
    b2DynamicTreeValidateMetrics(p, p->m_root);

    freeCount = 0;
    freeIndex = p->m_freeList;
    while (freeIndex != b2_nullNode)
    {
        b2Assert(0 <= freeIndex && freeIndex < p->m_nodeCapacity);
        freeIndex = p->m_nodes[freeIndex].next;
        ++freeCount;
    }

    b2Assert(b2DynamicTreeGetHeight(p) == b2DynamicTreeComputeHeight(p));

    b2Assert(p->m_nodeCount + freeCount == p->m_nodeCapacity);
#endif
}

B2_API
int32
b2DynamicTreeGetHeight(
    const struct b2DynamicTree* p)
{
    if (p->m_root == b2_nullNode)
    {
        return 0;
    }

    return p->m_nodes[p->m_root].height;
}

B2_API
int32
b2DynamicTreeGetMaxBalance(
    const struct b2DynamicTree* p)
{
    int32 i;
    int32 maxBalance = 0;
    for (i = 0; i < p->m_nodeCapacity; ++i)
    {
        int32 child1;
        int32 child2;
        int32 balance;

        const struct b2TreeNode* node = p->m_nodes + i;
        if (node->height <= 1)
        {
            continue;
        }

        b2Assert(b2TreeNodeIsLeaf(node) == b2False);

        child1 = node->child1;
        child2 = node->child2;
        balance = b2AbsInt32(p->m_nodes[child2].height - p->m_nodes[child1].height);
        maxBalance = b2MaxInt32(maxBalance, balance);
    }

    return maxBalance;
}

B2_API
float
b2DynamicTreeGetAreaRatio(
    const struct b2DynamicTree* p)
{
    int32 i;

    const struct b2TreeNode* root;
    float rootArea;
    float totalArea;

    if (p->m_root == b2_nullNode)
    {
        return 0.0f;
    }

    root = p->m_nodes + p->m_root;
    rootArea = b2AABBGetPerimeter(&root->aabb);

    totalArea = 0.0f;
    for (i = 0; i < p->m_nodeCapacity; ++i)
    {
        const struct b2TreeNode* node = p->m_nodes + i;
        if (node->height < 0)
        {
            // Free node in pool
            continue;
        }

        totalArea += b2AABBGetPerimeter(&node->aabb);
    }

    return totalArea / rootArea;
}

B2_API
void
b2DynamicTreeRebuildBottomUp(
    struct b2DynamicTree* p)
{
    int32 i;
    int32* nodes = (int32*)b2Alloc(p->m_nodeCount * sizeof(int32));
    int32 count = 0;

    // Build array of leaves. Free the rest.
    for (i = 0; i < p->m_nodeCapacity; ++i)
    {
        if (p->m_nodes[i].height < 0)
        {
            // free node in pool
            continue;
        }

        if (b2TreeNodeIsLeaf(&p->m_nodes[i]))
        {
            p->m_nodes[i].parent = b2_nullNode;
            nodes[count] = i;
            ++count;
        }
        else
        {
            b2DynamicTreeFreeNode(p, i);
        }
    }

    while (count > 1)
    {
        int32 i, j;
        float minCost = b2_maxFloat;
        int32 iMin = -1, jMin = -1;

        int32 index1;
        int32 index2;

        struct b2TreeNode* child1;
        struct b2TreeNode* child2;

        int32 parentIndex;

        struct b2TreeNode* parent;

        for (i = 0; i < count; ++i)
        {
            struct b2AABB* aabbi = &p->m_nodes[nodes[i]].aabb;

            for (j = i + 1; j < count; ++j)
            {
                float cost;
                struct b2AABB b;
                struct b2AABB* aabbj = &p->m_nodes[nodes[j]].aabb;
                b2AABBCombine2(&b, aabbi, aabbj);
                cost = b2AABBGetPerimeter(&b);
                if (cost < minCost)
                {
                    iMin = i;
                    jMin = j;
                    minCost = cost;
                }
            }
        }

        index1 = nodes[iMin];
        index2 = nodes[jMin];
        child1 = p->m_nodes + index1;
        child2 = p->m_nodes + index2;

        parentIndex = b2DynamicTreeAllocateNode(p);
        parent = p->m_nodes + parentIndex;
        parent->child1 = index1;
        parent->child2 = index2;
        parent->height = 1 + b2MaxInt32(child1->height, child2->height);
        b2AABBCombine2(&parent->aabb, &child1->aabb, &child2->aabb);
        parent->parent = b2_nullNode;

        child1->parent = parentIndex;
        child2->parent = parentIndex;

        nodes[jMin] = nodes[count - 1];
        nodes[iMin] = parentIndex;
        --count;
    }

    p->m_root = nodes[0];
    b2Free(nodes);

    b2DynamicTreeValidate(p);
}

B2_API
void
b2DynamicTreeShiftOrigin(
    struct b2DynamicTree* p,
    const b2Vec2 newOrigin)
{
    int32 i;
    struct b2AABB* aabb;
    // Build array of leaves. Free the rest.
    for (i = 0; i < p->m_nodeCapacity; ++i)
    {
        aabb = &p->m_nodes[i].aabb;
        b2Vec2Sub(aabb->lowerBound, aabb->lowerBound, newOrigin);
        b2Vec2Sub(aabb->upperBound, aabb->upperBound, newOrigin);
    }
}

B2_API
void
b2DynamicTreeQuery(
    const struct b2DynamicTree* p,
    const struct b2AABB* aabb,
    void* obj,
    void* callback)
{
    typedef int(*QueryCallback)(void* obj, int32 nodeId);

    struct b2GrowableStack stack;

    b2GrowableStackInit(&stack);
    b2GrowableStackPush(&stack, p->m_root);

    while (b2GrowableStackGetCount(&stack) > 0)
    {
        int32 nodeId;
        const struct b2TreeNode* node;

        nodeId = b2GrowableStackPop(&stack);
        if (nodeId == b2_nullNode)
        {
            continue;
        }

        node = p->m_nodes + nodeId;

        if (b2AABBTestOverlap(&node->aabb, aabb))
        {
            if (b2TreeNodeIsLeaf(node))
            {
                int proceed = (*((QueryCallback)callback))(obj, nodeId);
                if (proceed == b2False)
                {
                    return;
                }
            }
            else
            {
                b2GrowableStackPush(&stack, node->child1);
                b2GrowableStackPush(&stack, node->child2);
            }
        }
    }

    b2GrowableStackDestroy(&stack);
}

B2_API
void
b2DynamicTreeRayCast(
    const struct b2DynamicTree* p,
    const struct b2RayCastInput* input,
    void* obj,
    void* callback)
{
    typedef float(*RayCast)(void* obj, struct b2RayCastInput* subInput, int32 nodeId);

    b2Vec2 v;

    b2Vec2 p1;
    b2Vec2 p2;
    b2Vec2 r;

    b2Vec2 seg_v;
    b2Vec2 abs_v;

    float maxFraction;

    struct b2AABB segmentAABB;

    struct b2GrowableStack stack;

    b2GrowableStackInit(&stack);

    b2Vec2Assign(p1, input->p1);
    b2Vec2Assign(p2, input->p2);
    b2Vec2Sub(r, p2, p1);
    b2Assert(b2Vec2SquaredLength(r) > 0.0f);
    b2Vec2Normalize(r, r);

    // sv is perpendicular to the segment.
    b2Vec2CrossProductKL(seg_v, 1.0f, r);
    b2Vec2Abs(abs_v, seg_v);

    // Separating axis for segment (Gino, p80).
    // |dot(v, p1 - c)| > dot(|v|, h)

    maxFraction = input->maxFraction;

    // Build a bounding box for the segment.
    {
        b2Vec2 t;
        b2Vec2Sub(v, p2, p1);
        b2Vec2Scale(v, v, maxFraction);
        b2Vec2Add(t, p1, v);
        b2Vec2Min(segmentAABB.lowerBound, p1, t);
        b2Vec2Max(segmentAABB.upperBound, p1, t);
    }

    b2GrowableStackPush(&stack, p->m_root);

    while (b2GrowableStackGetCount(&stack) > 0)
    {
        int32 nodeId;
        const struct b2TreeNode* node;

        b2Vec2 c;
        b2Vec2 h;
        float separation;

        nodeId = b2GrowableStackPop(&stack);
        if (nodeId == b2_nullNode)
        {
            continue;
        }

        node = p->m_nodes + nodeId;

        if (b2AABBTestOverlap(&node->aabb, &segmentAABB) == b2False)
        {
            continue;
        }

        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)
        b2AABBGetCenter(&node->aabb, c);
        b2AABBGetExtents(&node->aabb, h);
        b2Vec2Sub(v, p1, c);
        separation = b2AbsFloat(b2Vec2DotProduct(seg_v, v)) - b2Vec2DotProduct(abs_v, h);
        if (separation > 0.0f)
        {
            continue;
        }

        if (b2TreeNodeIsLeaf(node))
        {
            float value;

            struct b2RayCastInput subInput;
            b2Vec2Assign(subInput.p1, input->p1);
            b2Vec2Assign(subInput.p2, input->p2);
            subInput.maxFraction = maxFraction;

            value = (*((RayCast)callback))(obj, &subInput, nodeId);

            if (value == 0.0f)
            {
                // The client has terminated the ray cast.
                return;
            }

            if (value > 0.0f)
            {
                b2Vec2 t;

                // Update segment bounding box.
                maxFraction = value;
                b2Vec2Sub(v, p2, p1);
                b2Vec2Scale(v, v, maxFraction);
                b2Vec2Add(t, p1, v);
                b2Vec2Min(segmentAABB.lowerBound, p1, t);
                b2Vec2Max(segmentAABB.upperBound, p1, t);
            }
        }
        else
        {
            b2GrowableStackPush(&stack, node->child1);
            b2GrowableStackPush(&stack, node->child2);
        }
    }

    b2GrowableStackDestroy(&stack);
}
