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

#include "mmB2ShapeEdge.h"
#include "mmB2BlockAllocator.h"
#include "mmB2Common.h"
#include "mmB2Collision.h"

B2_API const struct b2MetaAllocator b2MetaAllocatorShapeEdge =
{
    "b2ShapeEdge",
    sizeof(struct b2ShapeEdge),
    &b2ShapeEdgeInit,
    &b2ShapeEdgeDestroy,
};

B2_API const struct b2ShapeMeta b2MetaShapeEdge =
{
    &b2ShapeEdgeClone,
    &b2ShapeEdgeGetChildCount,
    &b2ShapeEdgeTestPoint,
    &b2ShapeEdgeRayCast,
    &b2ShapeEdgeComputeAABB,
    &b2ShapeEdgeComputeMass,
};

B2_API
void
b2ShapeEdgeInit(
    struct b2ShapeEdge* p)
{
    p->Meta = &b2MetaShapeEdge;
    p->m_type = b2ShapeTypeEdge;
    p->m_radius = b2_polygonRadius;
    p->m_vertex0[0] = 0.0f;
    p->m_vertex0[1] = 0.0f;
    p->m_vertex3[0] = 0.0f;
    p->m_vertex3[1] = 0.0f;
    p->m_oneSided = b2False;
}

B2_API
void
b2ShapeEdgeDestroy(
    struct b2ShapeEdge* p)
{
    p->m_oneSided = b2False;
    p->m_vertex3[1] = 0.0f;
    p->m_vertex3[0] = 0.0f;
    p->m_vertex0[1] = 0.0f;
    p->m_vertex0[0] = 0.0f;
    p->m_radius = b2_polygonRadius;
    p->m_type = b2ShapeTypeEdge;
    p->Meta = &b2MetaShapeEdge;
}

B2_API
void
b2ShapeEdgeReset(
    struct b2ShapeEdge* p)
{
    p->Meta = &b2MetaShapeEdge;
    p->m_type = b2ShapeTypeEdge;
    p->m_radius = b2_polygonRadius;
    p->m_vertex0[0] = 0.0f;
    p->m_vertex0[1] = 0.0f;
    p->m_vertex3[0] = 0.0f;
    p->m_vertex3[1] = 0.0f;
    p->m_oneSided = b2False;
}

B2_API
struct b2Shape*
b2ShapeEdgeClone(
    const struct b2ShapeEdge* p,
    struct b2BlockAllocator* allocator)
{
    void* mem;
    struct b2ShapeEdge* clone;
    mem = b2BlockAllocatorAllocate(allocator, sizeof(struct b2ShapeEdge));
    clone = (struct b2ShapeEdge*)mem;
    *clone = *p;
    return (struct b2Shape*)clone;
}

B2_API
int32
b2ShapeEdgeGetChildCount(
    const struct b2ShapeEdge* p)
{
    return 1;
}

B2_API
int
b2ShapeEdgeTestPoint(
    const struct b2ShapeEdge* p,
    const b2Transform transform, 
    const b2Vec2 point)
{
    B2_NOT_USED(transform);
    B2_NOT_USED(point);
    return b2False;
}

// p = p1 + t * d
// v = v1 + s * e
// p1 + t * d = v1 + s * e
// s * e - t * d = p1 - v1
B2_API
int
b2ShapeEdgeRayCast(
    const struct b2ShapeEdge* p,
    struct b2RayCastOutput* output,
    const struct b2RayCastInput* input,
    const b2Transform transform,
    int32 childIndex)
{
    b2Vec2 v;

    b2Vec2 p1;
    b2Vec2 p2;
    b2Vec2 d;

    b2Vec2 v1;
    b2Vec2 v2;
    b2Vec2 e;

    b2Vec2 normal;

    float numerator;
    float denominator;
    float t;

    b2Vec2 q;

    b2Vec2 r;
    float rr;

    float s;

    B2_NOT_USED(childIndex);

    // Put the ray into the edge's frame of reference.
    b2Vec2Sub(v, input->p1, transform[0]);
    b2RotMulTVec2(p1, transform[1], v);
    b2Vec2Sub(v, input->p2, transform[0]);
    b2RotMulTVec2(p2, transform[1], v);
    b2Vec2Sub(d, p2, p1);

    b2Vec2Assign(v1, p->m_vertex1);
    b2Vec2Assign(v2, p->m_vertex2);
    b2Vec2Sub(e, v2, v1);

    // Normal points to the right, looking from v1 at v2
    normal[0] = +e[1];
    normal[1] = -e[0];
    b2Vec2Normalize(normal, normal);

    // q = p1 + t * d
    // dot(normal, q - v1) = 0
    // dot(normal, p1 - v1) + t * dot(normal, d) = 0
    b2Vec2Sub(v, v1, p1);
    numerator = b2Vec2DotProduct(normal, v);
    if (p->m_oneSided && numerator > 0.0f)
    {
        return b2False;
    }

    denominator = b2Vec2DotProduct(normal, d);
    if (denominator == 0.0f)
    {
        return b2False;
    }

    t = numerator / denominator;
    if (t < 0.0f || input->maxFraction < t)
    {
        return b2False;
    }

    b2Vec2Scale(v, d, t);
    b2Vec2Add(q, p1, v);

    // q = v1 + s * r
    // s = dot(q - v1, r) / dot(r, r)
    b2Vec2Sub(r, v2, v1);
    rr = b2Vec2DotProduct(r, r);
    if (rr == 0.0f)
    {
        return b2False;
    }

    b2Vec2Sub(v, q, v1);
    s = b2Vec2DotProduct(v, r) / rr;
    if (s < 0.0f || 1.0f < s)
    {
        return b2False;
    }

    output->fraction = t;
    if (numerator > 0.0f)
    {
        b2RotMulVec2(v, transform[1], normal);
        b2Vec2Negate(output->normal, v);
    }
    else
    {
        b2RotMulVec2(v, transform[1], normal);
        b2Vec2Assign(output->normal, v);
    }
    return b2True;
}

B2_API
void 
b2ShapeEdgeComputeAABB(
    const struct b2ShapeEdge* p,
    struct b2AABB* aabb,
    const b2Transform transform,
    int32 childIndex)
{
    b2Vec2 v1;
    b2Vec2 v2;

    b2Vec2 lower;
    b2Vec2 upper;

    b2Vec2 r;

    B2_NOT_USED(childIndex);

    b2TransformMulVec2(v1, transform, p->m_vertex1);
    b2TransformMulVec2(v2, transform, p->m_vertex2);

    b2Vec2Min(lower, v1, v2);
    b2Vec2Max(upper, v1, v2);

    r[0] = p->m_radius; r[1] = p->m_radius;
    b2Vec2Sub(aabb->lowerBound, lower, r);
    b2Vec2Add(aabb->upperBound, upper, r);
}

B2_API
void
b2ShapeEdgeComputeMass(
    const struct b2ShapeEdge* p,
    struct b2MassData* massData,
    float density)
{
    b2Vec2 v;

    B2_NOT_USED(density);

    massData->mass = 0.0f;
    b2Vec2Add(v, p->m_vertex1, p->m_vertex2);
    b2Vec2Scale(massData->center, v, 0.5f);
    massData->I = 0.0f;
}

B2_API
void
b2ShapeEdgeSetOneSided(
    struct b2ShapeEdge* p,
    const b2Vec2 v0,
    const b2Vec2 v1,
    const b2Vec2 v2,
    const b2Vec2 v3)
{
    b2Vec2Assign(p->m_vertex0, v0);
    b2Vec2Assign(p->m_vertex1, v1);
    b2Vec2Assign(p->m_vertex2, v2);
    b2Vec2Assign(p->m_vertex3, v3);
    p->m_oneSided = b2True;
}

B2_API
void
b2ShapeEdgeSetTwoSided(
    struct b2ShapeEdge* p,
    const b2Vec2 v1,
    const b2Vec2 v2)
{
    b2Vec2Assign(p->m_vertex1, v1);
    b2Vec2Assign(p->m_vertex2, v2);
    p->m_oneSided = b2False;
}
