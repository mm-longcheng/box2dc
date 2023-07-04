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

#ifndef __mmB2Shape_h__
#define __mmB2Shape_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"

#include "b2/mmB2Prefix.h"

struct b2BlockAllocator;
struct b2RayCastOutput;
struct b2RayCastInput;
struct b2AABB;

/// This holds the mass data computed for a shape.
struct b2MassData
{
    /// The mass of the shape, usually in kilograms.
    float mass;

    /// The position of the shape's centroid relative to the shape's origin.
    b2Vec2 center;

    /// The rotational inertia of the shape about the local origin.
    float I;
};

enum b2ShapeType
{
    b2ShapeTypeCircle    = 0,
    b2ShapeTypeEdge      = 1,
    b2ShapeTypePolygon   = 2,
    b2ShapeTypeChain     = 3,
    b2ShapeTypeCount     = 4,
};

struct b2ShapeMeta
{
    /// Clone the concrete shape using the provided allocator.
    /// struct b2Shape* 
    /// (*Clone)(
    ///     const void* obj,
    ///     struct b2BlockAllocator* allocator);
    void* Clone;

    /// Get the number of child primitives.
    /// int32
    /// (*GetChildCount)(
    ///     const void* obj);
    void* GetChildCount;

    /// Test a point for containment in this shape. This only works for convex shapes.
    /// @param xf the shape world transform.
    /// @param p a point in world coordinates.
    /// int
    /// (*TestPoint)(
    ///     const void* obj,
    ///     const b2Transform xf, 
    ///     const b2Vec2 point);
    void* TestPoint;

    /// Cast a ray against a child shape.
    /// @param output the ray-cast results.
    /// @param input the ray-cast input parameters.
    /// @param transform the transform to be applied to the shape.
    /// @param childIndex the child shape index
    /// int
    /// (*RayCast)(
    ///     const void* obj, 
    ///     struct b2RayCastOutput* output, 
    ///     const struct b2RayCastInput* input,
    ///     const b2Transform transform, 
    ///     int32 childIndex);
    void* RayCast;

    /// Given a transform, compute the associated axis aligned bounding box for a child shape.
    /// @param aabb returns the axis aligned box.
    /// @param xf the world transform of the shape.
    /// @param childIndex the child shape
    /// void
    /// (*ComputeAABB)(
    ///     const void* obj, 
    ///     struct b2AABB* aabb, 
    ///     const b2Transform xf, 
    ///     int32 childIndex);
    void* ComputeAABB;

    /// Compute the mass properties of this shape using its dimensions and density.
    /// The inertia tensor is computed about the local origin.
    /// @param massData returns the mass data for this shape.
    /// @param density the density in kilograms per meter squared.
    /// void
    /// (*ComputeMass)(
    ///     const void* obj, 
    ///     struct b2MassData* massData, 
    ///     float density);
    void* ComputeMass;
};

/// A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in b2World are created automatically when a b2Fixture
/// is created. Shapes may encapsulate a one or more child shapes.
struct b2Shape
{
    /// The shape type.
    enum b2ShapeType m_type;

    /// Radius of a shape. For polygonal shapes this must be b2_polygonRadius. There is no support for
    /// making rounded polygons.
    float m_radius;

    /// The shape meta.
    const struct b2ShapeMeta* Meta;
};

/// shape super member.
#define b2ShapeSuper            \
enum b2ShapeType m_type;        \
float m_radius;                 \
const struct b2ShapeMeta* Meta

static
inline
enum b2ShapeType 
b2ShapeGetType(
    const struct b2Shape* p)
{
    return p->m_type;
}

B2_API
struct b2Shape*
b2ShapeClone(
    const struct b2Shape* obj,
    struct b2BlockAllocator* allocator);

B2_API
int32
b2ShapeGetChildCount(
    const struct b2Shape* obj);

B2_API
int
b2ShapeTestPoint(
    const struct b2Shape* obj,
    const b2Transform xf, 
    const b2Vec2 point);

B2_API
int
b2ShapeRayCast(
    const struct b2Shape* obj,
    struct b2RayCastOutput* output, 
    const struct b2RayCastInput* input,
    const b2Transform transform, 
    int32 childIndex);

B2_API
void
b2ShapeComputeAABB(
    const struct b2Shape* obj,
    struct b2AABB* aabb, 
    const b2Transform xf, 
    int32 childIndex);

B2_API
void
b2ShapeComputeMass(
    const struct b2Shape* obj,
    struct b2MassData* massData, 
    float density);

#include "b2/mmB2Suffix.h"

#endif//__mmB2Shape_h__
