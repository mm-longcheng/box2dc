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

#include "mmB2Shape.h"

B2_API
struct b2Shape*
b2ShapeClone(
    const struct b2Shape* obj,
    struct b2BlockAllocator* allocator)
{
    typedef
    struct b2Shape* 
    (*Clone)(
        const void* obj,
        struct b2BlockAllocator* allocator);

    return (*((Clone)obj->Meta->Clone))(
        obj,
        allocator);
}

B2_API
int32
b2ShapeGetChildCount(
    const struct b2Shape* obj)
{
    typedef
    int32
    (*GetChildCount)(
        const void* obj);

    return (*((GetChildCount)obj->Meta->GetChildCount))(
        obj);
}

B2_API
int
b2ShapeTestPoint(
    const struct b2Shape* obj,
    const b2Transform xf,
    const b2Vec2 point)
{
    typedef
    int
    (*TestPoint)(
        const void* obj,
        const b2Transform xf, 
        const b2Vec2 point);

    return (*((TestPoint)obj->Meta->TestPoint))(
        obj,
        xf,
        point);
}

B2_API
int
b2ShapeRayCast(
    const struct b2Shape* obj,
    struct b2RayCastOutput* output,
    const struct b2RayCastInput* input,
    const b2Transform transform,
    int32 childIndex)
{
    typedef
    int
    (*RayCast)(
        const void* obj, 
        struct b2RayCastOutput* output, 
        const struct b2RayCastInput* input,
        const b2Transform transform, 
        int32 childIndex);

    return (*((RayCast)obj->Meta->RayCast))(
        obj,
        output,
        input,
        transform,
        childIndex);
}

B2_API
void
b2ShapeComputeAABB(
    const struct b2Shape* obj,
    struct b2AABB* aabb,
    const b2Transform xf,
    int32 childIndex)
{
    typedef
    void
    (*ComputeAABB)(
        const void* obj, 
        struct b2AABB* aabb, 
        const b2Transform xf, 
        int32 childIndex);

    (*((ComputeAABB)obj->Meta->ComputeAABB))(
        obj,
        aabb,
        xf,
        childIndex);
}

B2_API
void
b2ShapeComputeMass(
    const struct b2Shape* obj,
    struct b2MassData* massData,
    float density)
{
    typedef
    void
    (*ComputeMass)(
        const void* obj, 
        struct b2MassData* massData, 
        float density);

    (*((ComputeMass)obj->Meta->ComputeMass))(
        obj,
        massData,
        density);
}

