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

#ifndef __mmB2ShapeEdge_h__
#define __mmB2ShapeEdge_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Shape.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

struct b2BlockAllocator;

B2_API extern const struct b2MetaAllocator b2MetaAllocatorShapeEdge;
B2_API extern const struct b2ShapeMeta b2MetaShapeEdge;

/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. Edges created independently are two-sided and do
/// no provide smooth movement across junctions.
struct b2ShapeEdge
{
    /// Super
    b2ShapeSuper;

    /// These are the edge vertices
    b2Vec2 m_vertex1, m_vertex2;

    /// Optional adjacent vertices. These are used for smooth collision.
    b2Vec2 m_vertex0, m_vertex3;

    /// Uses m_vertex0 and m_vertex3 to create smooth collision.
    int m_oneSided;
};

B2_API
void
b2ShapeEdgeInit(
    struct b2ShapeEdge* p);

B2_API
void
b2ShapeEdgeDestroy(
    struct b2ShapeEdge* p);

B2_API
void
b2ShapeEdgeReset(
    struct b2ShapeEdge* p);

/// Implement b2Shape.
B2_API
struct b2Shape*
b2ShapeEdgeClone(
    const struct b2ShapeEdge* p,
    struct b2BlockAllocator* allocator);

/// @see b2Shape::GetChildCount
B2_API
int32
b2ShapeEdgeGetChildCount(
    const struct b2ShapeEdge* p);

/// @see b2Shape::TestPoint
B2_API
int
b2ShapeEdgeTestPoint(
    const struct b2ShapeEdge* p,
    const b2Transform transform, 
    const b2Vec2 point);

/// Implement b2Shape.
B2_API
int
b2ShapeEdgeRayCast(
    const struct b2ShapeEdge* p,
    struct b2RayCastOutput* output, 
    const struct b2RayCastInput* input,
    const b2Transform transform, 
    int32 childIndex);

/// @see b2Shape::ComputeAABB
B2_API
void
b2ShapeEdgeComputeAABB(
    const struct b2ShapeEdge* p,
    struct b2AABB* aabb, 
    const b2Transform transform, 
    int32 childIndex);

/// @see b2Shape::ComputeMass
B2_API
void
b2ShapeEdgeComputeMass(
    const struct b2ShapeEdge* p,
    struct b2MassData* massData, 
    float density);

/// Set this as a part of a sequence. Vertex v0 precedes the edge and vertex v3
/// follows. These extra vertices are used to provide smooth movement
/// across junctions. This also makes the collision one-sided. The edge
/// normal points to the right looking from v1 to v2.
B2_API
void
b2ShapeEdgeSetOneSided(
    struct b2ShapeEdge* p,
    const b2Vec2 v0, 
    const b2Vec2 v1, 
    const b2Vec2 v2, 
    const b2Vec2 v3);

/// Set this as an isolated edge. Collision is two-sided.
B2_API
void
b2ShapeEdgeSetTwoSided(
    struct b2ShapeEdge* p,
    const b2Vec2 v1, 
    const b2Vec2 v2);

#include "b2/mmB2Suffix.h"

#endif//__mmB2ShapeEdge_h__
