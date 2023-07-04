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

#ifndef __mmB2ShapeChain_h__
#define __mmB2ShapeChain_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Shape.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

struct b2BlockAllocator;
struct b2ShapeEdge;

B2_API extern const struct b2MetaAllocator b2MetaAllocatorShapeChain;
B2_API extern const struct b2ShapeMeta b2MetaShapeChain;

/// A chain shape is a free form sequence of line segments.
/// The chain has one-sided collision, with the surface normal pointing to the right of the edge.
/// This provides a counter-clockwise winding like the polygon shape.
/// Connectivity information is used to create smooth collisions.
/// @warning the chain will not collide properly if there are self-intersections.
struct b2ShapeChain
{
    /// Super
    b2ShapeSuper;

    /// The vertices. Owned by this class.
    b2Vec2* m_vertices;

    /// The vertex count.
    int32 m_count;

    b2Vec2 m_prevVertex, m_nextVertex;
};

B2_API
void
b2ShapeChainInit(
    struct b2ShapeChain* p);

B2_API
void
b2ShapeChainDestroy(
    struct b2ShapeChain* p);

B2_API
void
b2ShapeChainReset(
    struct b2ShapeChain* p);

/// Implement b2Shape. Vertices are cloned using b2Alloc.
B2_API
struct b2Shape*
b2ShapeChainClone(
    const struct b2ShapeChain* p,
    struct b2BlockAllocator* allocator);

/// @see b2Shape::GetChildCount
B2_API
int32
b2ShapeChainGetChildCount(
    const struct b2ShapeChain* p);

/// This always return false.
/// @see b2Shape::TestPoint
B2_API
int
b2ShapeChainTestPoint(
    const struct b2ShapeChain* p,
    const b2Transform transform, 
    const b2Vec2 point);

/// Implement b2Shape.
B2_API
int
b2ShapeChainRayCast(
    const struct b2ShapeChain* p,
    struct b2RayCastOutput* output, 
    const struct b2RayCastInput* input,
    const b2Transform transform, 
    int32 childIndex);

/// @see b2Shape::ComputeAABB
B2_API
void
b2ShapeChainComputeAABB(
    const struct b2ShapeChain* p,
    struct b2AABB* aabb, 
    const b2Transform transform, 
    int32 childIndex);

/// Chains have zero mass.
/// @see b2Shape::ComputeMass
B2_API
void
b2ShapeChainComputeMass(
    const struct b2ShapeChain* p,
    struct b2MassData* massData, 
    float density);

/// Clear all data.
B2_API
void
b2ShapeChainClear(
    struct b2ShapeChain* p);

/// Create a loop. This automatically adjusts connectivity.
/// @param vertices an array of vertices, these are copied
/// @param count the vertex count
B2_API
void
b2ShapeChainCreateLoop(
    struct b2ShapeChain* p,
    const b2Vec2* vertices, 
    int32 count);

/// Create a chain with ghost vertices to connect multiple chains together.
/// @param vertices an array of vertices, these are copied
/// @param count the vertex count
/// @param prevVertex previous vertex from chain that connects to the start
/// @param nextVertex next vertex from chain that connects to the end
B2_API
void
b2ShapeChainCreateChain(
    struct b2ShapeChain* p,
    const b2Vec2* vertices, 
    int32 count,
    const b2Vec2 prevVertex, 
    const b2Vec2 nextVertex);

/// Get a child edge.
B2_API
void
b2ShapeChainGetChildEdge(
    const struct b2ShapeChain* p,
    struct b2ShapeEdge* edge,
    int32 index);

#include "b2/mmB2Suffix.h"

#endif//__mmB2ShapeChain_h__
