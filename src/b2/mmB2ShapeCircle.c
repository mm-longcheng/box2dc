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

#include "mmB2ShapeCircle.h"
#include "mmB2BlockAllocator.h"
#include "mmB2Common.h"
#include "mmB2Collision.h"

B2_API const struct b2MetaAllocator b2MetaAllocatorShapeCircle =
{
    "b2ShapeCircle",
    sizeof(struct b2ShapeCircle),
    &b2ShapeCircleInit,
    &b2ShapeCircleDestroy,
};

B2_API const struct b2ShapeMeta b2MetaShapeCircle =
{
    &b2ShapeCircleClone,
    &b2ShapeCircleGetChildCount,
    &b2ShapeCircleTestPoint,
    &b2ShapeCircleRayCast,
    &b2ShapeCircleComputeAABB,
    &b2ShapeCircleComputeMass,
};

B2_API
void
b2ShapeCircleInit(
    struct b2ShapeCircle* p)
{
    p->Meta = &b2MetaShapeCircle;
    p->m_type = b2ShapeTypeCircle;
    p->m_radius = 0.0f;
    b2Vec2SetZero(p->m_p);
}

B2_API
void
b2ShapeCircleDestroy(
    struct b2ShapeCircle* p)
{
    b2Vec2SetZero(p->m_p);
    p->m_radius = 0.0f;
    p->m_type = b2ShapeTypeCircle;
    p->Meta = &b2MetaShapeCircle;
}

B2_API
void
b2ShapeCircleReset(
    struct b2ShapeCircle* p)
{
    p->Meta = &b2MetaShapeCircle;
    p->m_type = b2ShapeTypeCircle;
    p->m_radius = 0.0f;
    b2Vec2SetZero(p->m_p);
}

B2_API
struct b2Shape*
b2ShapeCircleClone(
    const struct b2ShapeCircle* p,
    struct b2BlockAllocator* allocator)
{
    void* mem;
    struct b2ShapeCircle* clone;
    mem = b2BlockAllocatorAllocate(allocator, sizeof(struct b2ShapeCircle));
    clone = (struct b2ShapeCircle*)mem;
    *clone = *p;
    return (struct b2Shape*)clone;
}

B2_API
int32
b2ShapeCircleGetChildCount(
    const struct b2ShapeCircle* p)
{
    return 1;
}

B2_API
int
b2ShapeCircleTestPoint(
    const struct b2ShapeCircle* p,
    const b2Transform transform, 
    const b2Vec2 point)
{
    b2Vec2 v;
    b2Vec2 center;
    b2Vec2 d;

    b2RotMulVec2(v, transform[1], p->m_p);
    b2Vec2Add(center, transform[0], v);
    b2Vec2Sub(d, point, center);
    return b2Vec2DotProduct(d, d) <= p->m_radius * p->m_radius;
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
B2_API
int
b2ShapeCircleRayCast(
    const struct b2ShapeCircle* p,
    struct b2RayCastOutput* output,
    const struct b2RayCastInput* input,
    const b2Transform transform,
    int32 childIndex)
{
    b2Vec2 v;

    b2Vec2 position;
    b2Vec2 s;
    float b;

    b2Vec2 r;
    float c;
    float rr;
    float sigma;

    float a;

    B2_NOT_USED(childIndex);

    b2RotMulVec2(v, transform[1], p->m_p);
    b2Vec2Add(position, transform[0], v);
    b2Vec2Sub(s, input->p1, position);
    b = b2Vec2DotProduct(s, s) - p->m_radius * p->m_radius;

    // Solve quadratic equation.
    b2Vec2Sub(r, input->p2, input->p1);
    c = b2Vec2DotProduct(s, r);
    rr = b2Vec2DotProduct(r, r);
    sigma = c * c - rr * b;

    // Check for negative discriminant and short segment.
    if (sigma < 0.0f || rr < b2_epsilon)
    {
        return b2False;
    }

    // Find the point of intersection of the line with the circle.
    a = -(c + b2Sqrt(sigma));

    // Is the intersection point on the segment?
    if (0.0f <= a && a <= input->maxFraction * rr)
    {
        a /= rr;
        output->fraction = a;

        b2Vec2Scale(v, r, a);
        b2Vec2Add(output->normal, s, v);
        b2Vec2Normalize(output->normal, output->normal);
        return b2True;
    }

    return b2False;
}

B2_API
void
b2ShapeCircleComputeAABB(
    const struct b2ShapeCircle* p,
    struct b2AABB* aabb,
    const b2Transform transform,
    int32 childIndex)
{
    b2Vec2 v;
    b2Vec2 a;

    B2_NOT_USED(childIndex);

    b2RotMulVec2(v, transform[1], p->m_p);
    b2Vec2Add(a, transform[0], v);
    b2Vec2Make(aabb->lowerBound, a[0] - p->m_radius, a[1] - p->m_radius);
    b2Vec2Make(aabb->upperBound, a[0] + p->m_radius, a[1] + p->m_radius);
}

B2_API
void
b2ShapeCircleComputeMass(
    const struct b2ShapeCircle* p,
    struct b2MassData* massData,
    float density)
{
    massData->mass = density * b2_pi * p->m_radius * p->m_radius;
    b2Vec2Assign(massData->center, p->m_p);

    // inertia about the local origin
    massData->I = massData->mass * (0.5f * p->m_radius * p->m_radius + b2Vec2DotProduct(p->m_p, p->m_p));
}
