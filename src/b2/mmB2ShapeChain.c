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

#include "mmB2ShapeChain.h"
#include "mmB2BlockAllocator.h"
#include "mmB2Common.h"
#include "mmB2Collision.h"
#include "mmB2ShapeEdge.h"

#include <assert.h>
#include <string.h>

B2_API const struct b2MetaAllocator b2MetaAllocatorShapeChain =
{
    "b2ShapeChain",
    sizeof(struct b2ShapeChain),
    &b2ShapeChainInit,
    &b2ShapeChainDestroy,
};

B2_API const struct b2ShapeMeta b2MetaShapeChain =
{
    &b2ShapeChainClone,
    &b2ShapeChainGetChildCount,
    &b2ShapeChainTestPoint,
    &b2ShapeChainRayCast,
    &b2ShapeChainComputeAABB,
    &b2ShapeChainComputeMass,
};

B2_API
void
b2ShapeChainInit(
    struct b2ShapeChain* p)
{
    p->Meta = &b2MetaShapeChain;
    p->m_type = b2ShapeTypeChain;
    p->m_radius = b2_polygonRadius;
    p->m_vertices = NULL;
    p->m_count = 0;
}

B2_API
void
b2ShapeChainDestroy(
    struct b2ShapeChain* p)
{
    b2ShapeChainClear(p);

    p->m_count = 0;
    p->m_vertices = NULL;
    p->m_radius = b2_polygonRadius;
    p->m_type = b2ShapeTypeChain;
    p->Meta = &b2MetaShapeChain;
}

B2_API
void
b2ShapeChainReset(
    struct b2ShapeChain* p)
{
    p->Meta = &b2MetaShapeChain;
    p->m_type = b2ShapeTypeChain;
    p->m_radius = b2_polygonRadius;
    p->m_vertices = NULL;
    p->m_count = 0;
}

B2_API
struct b2Shape*
b2ShapeChainClone(
    const struct b2ShapeChain* p,
    struct b2BlockAllocator* allocator)
{
    void* mem;
    struct b2ShapeChain* clone;
    mem = b2BlockAllocatorAllocate(allocator, sizeof(struct b2ShapeChain));
    clone = (struct b2ShapeChain*)mem;
    b2ShapeChainInit(clone);
    b2ShapeChainCreateChain(clone, p->m_vertices, p->m_count, p->m_prevVertex, p->m_nextVertex);
    return (struct b2Shape*)clone;
}

B2_API
int32
b2ShapeChainGetChildCount(
    const struct b2ShapeChain* p)
{
    // edge count = vertex count - 1
    return p->m_count - 1;
}

B2_API
int
b2ShapeChainTestPoint(
    const struct b2ShapeChain* p,
    const b2Transform transform, 
    const b2Vec2 point)
{
    B2_NOT_USED(transform);
    B2_NOT_USED(point);
    return b2False;
}

B2_API
int
b2ShapeChainRayCast(
    const struct b2ShapeChain* p,
    struct b2RayCastOutput* output,
    const struct b2RayCastInput* input,
    const b2Transform transform,
    int32 childIndex)
{
    struct b2ShapeEdge shapeEdge;

    int32 i1;
    int32 i2;

    b2Assert(childIndex < p->m_count);

    i1 = childIndex;
    i2 = childIndex + 1;
    if (i2 == p->m_count)
    {
        i2 = 0;
    }

    b2Vec2Assign(shapeEdge.m_vertex1, p->m_vertices[i1]);
    b2Vec2Assign(shapeEdge.m_vertex2, p->m_vertices[i2]);

    return b2ShapeEdgeRayCast(&shapeEdge, output, input, transform, 0);
}

B2_API
void
b2ShapeChainComputeAABB(
    const struct b2ShapeChain* p,
    struct b2AABB* aabb,
    const b2Transform transform,
    int32 childIndex)
{
    int32 i1;
    int32 i2;

    b2Vec2 v1;
    b2Vec2 v2;

    b2Vec2 lower;
    b2Vec2 upper;

    b2Vec2 r;

    b2Assert(childIndex < p->m_count);

    i1 = childIndex;
    i2 = childIndex + 1;
    if (i2 == p->m_count)
    {
        i2 = 0;
    }

    b2TransformMulVec2(v1, transform, p->m_vertices[i1]);
    b2TransformMulVec2(v2, transform, p->m_vertices[i2]);

    b2Vec2Min(lower, v1, v2);
    b2Vec2Max(upper, v1, v2);

    r[0] = p->m_radius;
    r[1] = p->m_radius;
    b2Vec2Sub(aabb->lowerBound, lower, r);
    b2Vec2Add(aabb->upperBound, upper, r);
}

B2_API
void
b2ShapeChainComputeMass(
    const struct b2ShapeChain* p,
    struct b2MassData* massData,
    float density)
{
    B2_NOT_USED(density);

    massData->mass = 0.0f;
    b2Vec2SetZero(massData->center);
    massData->I = 0.0f;
}

B2_API
void
b2ShapeChainClear(
    struct b2ShapeChain* p)
{
    b2Free(p->m_vertices);
    p->m_vertices = NULL;
    p->m_count = 0;
}

B2_API
void
b2ShapeChainCreateLoop(
    struct b2ShapeChain* p,
    const b2Vec2* vertices,
    int32 count)
{
#if defined(b2DEBUG)
    int32 i;
#endif

    b2Assert(p->m_vertices == NULL && p->m_count == 0);
    b2Assert(count >= 3);
    if (count < 3)
    {
        return;
    }

#if defined(b2DEBUG)
    for (i = 1; i < count; ++i)
    {
        b2Vec2ConstRef v1 = vertices[i - 1];
        b2Vec2ConstRef v2 = vertices[i];
        // If the code crashes here, it means your vertices are too close together.
        b2Assert(b2Vec2SquaredDistance(v1, v2) > b2_linearSlop * b2_linearSlop);
    }
#endif

    p->m_count = count + 1;
    p->m_vertices = (b2Vec2*)b2Alloc(p->m_count * sizeof(b2Vec2));
    memcpy(p->m_vertices, vertices, count * sizeof(b2Vec2));
    b2Vec2Assign(p->m_vertices[count], p->m_vertices[0]);
    b2Vec2Assign(p->m_prevVertex, p->m_vertices[p->m_count - 2]);
    b2Vec2Assign(p->m_nextVertex, p->m_vertices[1]);
}

B2_API
void
b2ShapeChainCreateChain(
    struct b2ShapeChain* p,
    const b2Vec2* vertices,
    int32 count,
    const b2Vec2 prevVertex,
    const b2Vec2 nextVertex)
{
#if defined(b2DEBUG)
    int32 i;
#endif

    b2Assert(p->m_vertices == NULL && p->m_count == 0);
    b2Assert(count >= 2);
#if defined(b2DEBUG)
    for (i = 1; i < count; ++i)
    {
        b2Vec2ConstRef v1 = vertices[i - 1];
        b2Vec2ConstRef v2 = vertices[i];
        // If the code crashes here, it means your vertices are too close together.
        b2Assert(b2Vec2SquaredDistance(v1, v2) > b2_linearSlop * b2_linearSlop);
    }
#endif

    p->m_count = count;
    p->m_vertices = (b2Vec2*)b2Alloc(count * sizeof(b2Vec2));
    memcpy(p->m_vertices, vertices, p->m_count * sizeof(b2Vec2));
    b2Vec2Assign(p->m_prevVertex, prevVertex);
    b2Vec2Assign(p->m_nextVertex, nextVertex);
}

B2_API
void
b2ShapeChainGetChildEdge(
    const struct b2ShapeChain* p,
    struct b2ShapeEdge* edge,
    int32 index)
{
    b2Assert(0 <= index && index < p->m_count - 1);
    edge->m_type = b2ShapeTypeEdge;
    edge->m_radius = p->m_radius;

    b2Vec2Assign(edge->m_vertex1, p->m_vertices[index + 0]);
    b2Vec2Assign(edge->m_vertex2, p->m_vertices[index + 1]);
    edge->m_oneSided = b2True;

    if (index > 0)
    {
        b2Vec2Assign(edge->m_vertex0, p->m_vertices[index - 1]);
    }
    else
    {
        b2Vec2Assign(edge->m_vertex0, p->m_prevVertex);
    }

    if (index < p->m_count - 2)
    {
        b2Vec2Assign(edge->m_vertex3, p->m_vertices[index + 2]);
    }
    else
    {
        b2Vec2Assign(edge->m_vertex3, p->m_nextVertex);
    }
}
