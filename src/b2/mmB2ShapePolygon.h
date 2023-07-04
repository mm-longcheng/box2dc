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

#ifndef __mmB2ShapePolygon_h__
#define __mmB2ShapePolygon_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Shape.h"
#include "b2/mmB2MetaAllocator.h"
#include "b2/mmB2Settings.h"

#include "b2/mmB2Prefix.h"

struct b2BlockAllocator;
struct b2Hull;

B2_API extern const struct b2MetaAllocator b2MetaAllocatorShapePolygon;
B2_API extern const struct b2ShapeMeta b2MetaShapePolygon;

/// A solid convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
struct b2ShapePolygon
{
    /// Super
    b2ShapeSuper;

    b2Vec2 m_centroid;
    b2Vec2 m_vertices[b2_maxPolygonVertices];
    b2Vec2 m_normals[b2_maxPolygonVertices];
    int32 m_count;
};

B2_API
void
b2ShapePolygonInit(
    struct b2ShapePolygon* p);

B2_API
void
b2ShapePolygonDestroy(
    struct b2ShapePolygon* p);

B2_API
void
b2ShapePolygonReset(
    struct b2ShapePolygon* p);

/// Implement b2Shape.
B2_API
struct b2Shape*
b2ShapePolygonClone(
    const struct b2ShapePolygon* p,
    struct b2BlockAllocator* allocator);

/// @see b2Shape::GetChildCount
B2_API
int32
b2ShapePolygonGetChildCount(
    const struct b2ShapePolygon* p);

/// @see b2Shape::TestPoint
B2_API
int
b2ShapePolygonTestPoint(
    const struct b2ShapePolygon* p,
    const b2Transform transform, 
    const b2Vec2 point);

/// Implement b2Shape.
/// @note because the polygon is solid, rays that start inside do not hit because the normal is
/// not defined.
B2_API
int
b2ShapePolygonRayCast(
    const struct b2ShapePolygon* p,
    struct b2RayCastOutput* output, 
    const struct b2RayCastInput* input,
    const b2Transform transform, 
    int32 childIndex);

/// @see b2Shape::ComputeAABB
B2_API
void
b2ShapePolygonComputeAABB(
    const struct b2ShapePolygon* p,
    struct b2AABB* aabb, 
    const b2Transform transform, 
    int32 childIndex);

/// @see b2Shape::ComputeMass
B2_API
void
b2ShapePolygonComputeMass(
    const struct b2ShapePolygon* p,
    struct b2MassData* massData, 
    float density);

/// Create a convex hull from the given array of local points.
/// The count must be in the range [3, b2_maxPolygonVertices].
/// @warning the points may be re-ordered, even if they form a convex polygon
/// @warning if this fails then the polygon is invalid
/// @returns true if valid
B2_API
int
b2ShapePolygonSetPoints(
    struct b2ShapePolygon* p,
    const b2Vec2* points, 
    int32 count);

/// Create a polygon from a given convex hull (see b2ComputeHull).
/// @warning the hull must be valid or this will crash or have unexpected behavior
B2_API
void
b2ShapePolygonSetHull(
    struct b2ShapePolygon* p,
    const struct b2Hull* hull);

/// Build vertices to represent an axis-aligned box centered on the local origin.
/// @param hx the half-width.
/// @param hy the half-height.
B2_API
void
b2ShapePolygonSetAsBox(
    struct b2ShapePolygon* p,
    float hx, 
    float hy);

/// Build vertices to represent an oriented box.
/// @param hx the half-width.
/// @param hy the half-height.
/// @param center the center of the box in local coordinates.
/// @param angle the rotation of the box in local coordinates.
B2_API
void
b2ShapePolygonSetAsBoxDetail(
    struct b2ShapePolygon* p,
    float hx, 
    float hy, 
    const b2Vec2 center, 
    float angle);

/// Validate convexity. This is a very time consuming operation.
/// @returns true if valid
B2_API
int
b2ShapePolygonValidate(
    const struct b2ShapePolygon* p);

#include "b2/mmB2Suffix.h"

#endif//__mmB2ShapePolygon_h__
