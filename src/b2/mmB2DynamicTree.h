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

#ifndef __mmB2DynamicTree_h__
#define __mmB2DynamicTree_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Collision.h"

#include "b2/mmB2Prefix.h"

#define b2_nullNode (-1)

/// A node in the dynamic tree. The client does not interact with this directly.
struct b2TreeNode
{
    /// Enlarged AABB
    struct b2AABB aabb;

    void* userData;

    union
    {
        int32 parent;
        int32 next;
    };

    int32 child1;
    int32 child2;

    // leaf = 0, free node = -1
    int32 height;

    int moved;
};

static 
inline
int
b2TreeNodeIsLeaf(
    const struct b2TreeNode* p)
{
    return p->child1 == b2_nullNode;
}

/// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
/// so that the proxy AABB is bigger than the client object. This allows the client
/// object to move by small amounts without triggering a tree update.
///
/// Nodes are pooled and relocatable, so we use node indices rather than pointers.
struct b2DynamicTree
{
    int32 m_root;

    struct b2TreeNode* m_nodes;
    int32 m_nodeCount;
    int32 m_nodeCapacity;

    int32 m_freeList;

    int32 m_insertionCount;
};

/// Constructing the tree initializes the node pool.
B2_API
void
b2DynamicTreeInit(
    struct b2DynamicTree* p);

/// Destroy the tree, freeing the node pool.
B2_API
void
b2DynamicTreeDestroy(
    struct b2DynamicTree* p);

/// Create a proxy. Provide a tight fitting AABB and a userData pointer.
B2_API
int32
b2DynamicTreeCreateProxy(
    struct b2DynamicTree* p,
    const struct b2AABB* aabb, 
    void* userData);

/// Delete a proxy. This asserts if the id is invalid.
B2_API
void
b2DynamicTreeDeleteProxy(
    struct b2DynamicTree* p,
    int32 proxyId);

/// Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
/// then the proxy is removed from the tree and re-inserted. Otherwise
/// the function returns immediately.
/// @return true if the proxy was re-inserted.
B2_API
int
b2DynamicTreeMoveProxy(
    struct b2DynamicTree* p,
    int32 proxyId, 
    const struct b2AABB* aabb1,
    const b2Vec2 displacement);

/// Get proxy user data.
/// @return the proxy user data or 0 if the id is invalid.
B2_API
void*
b2DynamicTreeGetUserData(
    const struct b2DynamicTree* p,
    int32 proxyId);

B2_API
int
b2DynamicTreeWasMoved(
    const struct b2DynamicTree* p,
    int32 proxyId);

B2_API
void
b2DynamicTreeClearMoved(
    struct b2DynamicTree* p,
    int32 proxyId);

/// Get the fat AABB for a proxy.
B2_API
const struct b2AABB*
b2DynamicTreeGetFatAABB(
    const struct b2DynamicTree* p,
    int32 proxyId);

/// Validate this tree. For testing.
B2_API
void
b2DynamicTreeValidate(
    const struct b2DynamicTree* p);

/// Compute the height of the binary tree in O(N) time. Should not be
/// called often.
B2_API
int32
b2DynamicTreeGetHeight(
    const struct b2DynamicTree* p);

/// Get the maximum balance of an node in the tree. The balance is the difference
/// in height of the two children of a node.
B2_API
int32
b2DynamicTreeGetMaxBalance(
    const struct b2DynamicTree* p);

/// Get the ratio of the sum of the node areas to the root area.
B2_API
float
b2DynamicTreeGetAreaRatio(
    const struct b2DynamicTree* p);

/// Build an optimal tree. Very expensive. For testing.
B2_API
void
b2DynamicTreeRebuildBottomUp(
    struct b2DynamicTree* p);

/// Shift the world origin. Useful for large worlds.
/// The shift formula is: position -= newOrigin
/// @param newOrigin the new origin with respect to the old origin
B2_API
void
b2DynamicTreeShiftOrigin(
    struct b2DynamicTree* p,
    const b2Vec2 newOrigin);

/// Query an AABB for overlapping proxies. The callback class
/// is called for each proxy that overlaps the supplied AABB.
/// typedef int(*QueryCallback)(void* obj, int32 nodeId);
B2_API
void
b2DynamicTreeQuery(
    const struct b2DynamicTree* p,
    const struct b2AABB* aabb,
    void* obj,
    void* callback);

/// Ray-cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray-cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
/// @param callback a callback class that is called for each proxy that is hit by the ray.
/// typedef float(*RayCast)(void* obj, struct b2RayCastInput* subInput, int32 nodeId);
B2_API
void
b2DynamicTreeRayCast(
    const struct b2DynamicTree* p,
    const struct b2RayCastInput* input,
    void* obj,
    void* callback);

#include "b2/mmB2Suffix.h"

#endif//__mmB2DynamicTree_h__
