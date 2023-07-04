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

#ifndef __mmB2ShapeCircle_h__
#define __mmB2ShapeCircle_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Shape.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

struct b2BlockAllocator;

B2_API extern const struct b2MetaAllocator b2MetaAllocatorShapeCircle;
B2_API extern const struct b2ShapeMeta b2MetaShapeCircle;

/// A solid circle shape
struct b2ShapeCircle
{
    /// Super
    b2ShapeSuper;

    /// Position
    b2Vec2 m_p;
};

B2_API
void
b2ShapeCircleInit(
    struct b2ShapeCircle* p);

B2_API
void
b2ShapeCircleDestroy(
    struct b2ShapeCircle* p);

B2_API
void
b2ShapeCircleReset(
    struct b2ShapeCircle* p);

/// Implement b2Shape.
B2_API
struct b2Shape*
b2ShapeCircleClone(
    const struct b2ShapeCircle* p,
    struct b2BlockAllocator* allocator);

/// @see b2Shape::GetChildCount
B2_API
int32
b2ShapeCircleGetChildCount(
    const struct b2ShapeCircle* p);

/// Implement b2Shape.
B2_API
int
b2ShapeCircleTestPoint(
    const struct b2ShapeCircle* p,
    const b2Transform transform, 
    const b2Vec2 point);

/// Implement b2Shape.
/// @note because the circle is solid, rays that start inside do not hit because the normal is
/// not defined.
B2_API
int
b2ShapeCircleRayCast(
    const struct b2ShapeCircle* p,
    struct b2RayCastOutput* output, 
    const struct b2RayCastInput* input,
    const b2Transform transform, 
    int32 childIndex);

/// @see b2Shape::ComputeAABB
B2_API
void
b2ShapeCircleComputeAABB(
    const struct b2ShapeCircle* p,
    struct b2AABB* aabb, 
    const b2Transform transform, 
    int32 childIndex);

/// @see b2Shape::ComputeMass
B2_API
void
b2ShapeCircleComputeMass(
    const struct b2ShapeCircle* p,
    struct b2MassData* massData, 
    float density);

#include "b2/mmB2Suffix.h"

#endif//__mmB2ShapeCircle_h__
