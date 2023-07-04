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

#include "mmB2Distance.h"
#include "mmB2Common.h"
#include "mmB2Shape.h"

#include "mmB2ShapeCircle.h"
#include "mmB2ShapePolygon.h"
#include "mmB2ShapeChain.h"
#include "mmB2ShapePolygon.h"
#include "mmB2ShapeEdge.h"

#include <assert.h>
#include <string.h>

// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
B2_API int32 b2_gjkCalls;
B2_API int32 b2_gjkIters;
B2_API int32 b2_gjkMaxIters;

struct b2SimplexVertex
{
    b2Vec2 wA;		// support point in proxyA
    b2Vec2 wB;		// support point in proxyB
    b2Vec2 w;		// wB - wA
    float a;		// barycentric coordinate for closest point
    int32 indexA;	// wA index
    int32 indexB;	// wB index
};

struct b2Simplex
{
    struct b2SimplexVertex m_v1;
    struct b2SimplexVertex m_v2;
    struct b2SimplexVertex m_v3;
    int32 m_count;
};

static
float
b2SimplexGetMetric(
    const struct b2Simplex* p)
{
    switch (p->m_count)
    {
    case 0:
        b2Assert(b2False);
        return 0.0f;

    case 1:
        return 0.0f;

    case 2:
        return b2Vec2Distance(p->m_v1.w, p->m_v2.w);

    case 3:
    {
        b2Vec2 v0, v1;
        b2Vec2Sub(v0, p->m_v2.w, p->m_v1.w);
        b2Vec2Sub(v1, p->m_v3.w, p->m_v1.w);
        return b2Vec2CrossProduct(v0, v1);
    }

    default:
        b2Assert(b2False);
        return 0.0f;
    }
}

static
void
b2SimplexReadCache(
    struct b2Simplex* p,
    const struct b2SimplexCache* cache,
    const struct b2DistanceProxy* proxyA, const b2Transform transformA,
    const struct b2DistanceProxy* proxyB, const b2Transform transformB)
{
    int32 i;

    struct b2SimplexVertex* vertices;

    b2Assert(cache->count <= 3);

    // Copy data from cache.
    p->m_count = cache->count;
    vertices = &p->m_v1;
    for (i = 0; i < p->m_count; ++i)
    {
        b2Vec2ConstRef wALocal;
        b2Vec2ConstRef wBLocal;

        struct b2SimplexVertex* v = vertices + i;
        v->indexA = cache->indexA[i];
        v->indexB = cache->indexB[i];
        wALocal = b2DistanceProxyGetVertex(proxyA, v->indexA);
        wBLocal = b2DistanceProxyGetVertex(proxyB, v->indexB);
        b2TransformMulVec2(v->wA, transformA, wALocal);
        b2TransformMulVec2(v->wB, transformB, wBLocal);
        b2Vec2Sub(v->w, v->wB, v->wA);
        v->a = 0.0f;
    }

    // Compute the new simplex metric, if it is substantially different than
    // old metric then flush the simplex.
    if (p->m_count > 1)
    {
        float metric1 = cache->metric;
        float metric2 = b2SimplexGetMetric(p);
        if (metric2 < 0.5f * metric1 || 
            2.0f * metric1 < metric2 || 
            metric2 < b2_epsilon)
        {
            // Reset the simplex.
            p->m_count = 0;
        }
    }

    // If the cache is empty or invalid ...
    if (p->m_count == 0)
    {
        b2Vec2ConstRef wALocal;
        b2Vec2ConstRef wBLocal;

        struct b2SimplexVertex* v = vertices + 0;
        v->indexA = 0;
        v->indexB = 0;
        wALocal = b2DistanceProxyGetVertex(proxyA, 0);
        wBLocal = b2DistanceProxyGetVertex(proxyB, 0);
        b2TransformMulVec2(v->wA, transformA, wALocal);
        b2TransformMulVec2(v->wB, transformB, wBLocal);
        b2Vec2Sub(v->w, v->wB, v->wA);
        v->a = 1.0f;
        p->m_count = 1;
    }
}

static
void
b2SimplexWriteCache(
    const struct b2Simplex* p,
    struct b2SimplexCache* cache)
{
    int32 i;
    const struct b2SimplexVertex* vertices;
    cache->metric = b2SimplexGetMetric(p);
    cache->count = (uint16)(p->m_count);
    vertices = &p->m_v1;
    for (i = 0; i < p->m_count; ++i)
    {
        cache->indexA[i] = (uint8)(vertices[i].indexA);
        cache->indexB[i] = (uint8)(vertices[i].indexB);
    }
}

static
void
b2SimplexGetSearchDirection(
    const struct b2Simplex* p,
    b2Vec2 d)
{
    switch (p->m_count)
    {
    case 1:
        b2Vec2Negate(d, p->m_v1.w);
        break;
    case 2:
    {
        b2Vec2 v;
        b2Vec2 e12;
        float sgn;

        b2Vec2Sub(e12, p->m_v2.w, p->m_v1.w);

        b2Vec2Negate(v, p->m_v1.w);
        sgn = b2Vec2CrossProduct(e12, v);
        if (sgn > 0.0f)
        {
            // Origin is left of e12.
            b2Vec2CrossProductKL(d, 1.0f, e12);
        }
        else
        {
            // Origin is right of e12.
            b2Vec2CrossProductKR(d, e12, 1.0f);
        }
        break;
    }

    default:
        b2Assert(b2False);
        b2Vec2SetZero(d);
        break;
    }
}

static
void
b2SimplexGetClosestPoint(
    const struct b2Simplex* p,
    b2Vec2 d)
{
    switch (p->m_count)
    {
    case 0:
        b2Assert(b2False);
        b2Vec2SetZero(d);
        break;

    case 1:
        b2Vec2Assign(d, p->m_v1.w);
        break;

    case 2:
    {
        b2Vec2 v0, v1;
        b2Vec2Scale(v0, p->m_v1.w, p->m_v1.a);
        b2Vec2Scale(v1, p->m_v2.w, p->m_v2.a);
        b2Vec2Add(d, v0, v1);
        break;
    }

    case 3:
        b2Vec2SetZero(d);
        break;

    default:
        b2Assert(b2False);
        b2Vec2SetZero(d);
        break;
    }
}

static
void
b2SimplexGetWitnessPoints(
    const struct b2Simplex* p,
    b2Vec2 pA, 
    b2Vec2 pB)
{
    switch (p->m_count)
    {
    case 0:
        b2Assert(b2False);
        break;

    case 1:
        b2Vec2Assign(pA, p->m_v1.wA);
        b2Vec2Assign(pB, p->m_v1.wB);
        break;

    case 2:
    {
        b2Vec2 v0, v1;
        b2Vec2Scale(v0, p->m_v1.wA, p->m_v1.a);
        b2Vec2Scale(v1, p->m_v2.wA, p->m_v2.a);
        b2Vec2Add(pA, v0, v1);

        b2Vec2Scale(v0, p->m_v1.wB, p->m_v1.a);
        b2Vec2Scale(v1, p->m_v2.wB, p->m_v2.a);
        b2Vec2Add(pB, v0, v1);
        break;
    }

    case 3:
    {
        b2Vec2 v0, v1, v2;
        b2Vec2Scale(v0, p->m_v1.wA, p->m_v1.a);
        b2Vec2Scale(v1, p->m_v2.wA, p->m_v2.a);
        b2Vec2Scale(v2, p->m_v3.wA, p->m_v3.a);
        b2Vec2Add(pA, v0, v1);
        b2Vec2Add(pA, pA, v2);
        b2Vec2Assign(pB, pA);
        break;
    }

    default:
        b2Assert(b2False);
        break;
    }
}

// Solve a line segment using barycentric coordinates.
//
// p = a1 * w1 + a2 * w2
// a1 + a2 = 1
//
// The vector from the origin to the closest point on the line is
// perpendicular to the line.
// e12 = w2 - w1
// dot(p, e) = 0
// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
//
// 2-by-2 linear system
// [1      1     ][a1] = [1]
// [w1.e12 w2.e12][a2] = [0]
//
// Define
// d12_1 =  dot(w2, e12)
// d12_2 = -dot(w1, e12)
// d12 = d12_1 + d12_2
//
// Solution
// a1 = d12_1 / d12
// a2 = d12_2 / d12
static
void
b2SimplexSolve2(
    struct b2Simplex* p)
{
    b2Vec2 w1;
    b2Vec2 w2;
    b2Vec2 e12;

    float d12_2;
    float d12_1;
    float inv_d12;

    b2Vec2Assign(w1, p->m_v1.w);
    b2Vec2Assign(w2, p->m_v2.w);
    b2Vec2Sub(e12, w2, w1);

    // w1 region
    d12_2 = -b2Vec2DotProduct(w1, e12);
    if (d12_2 <= 0.0f)
    {
        // a2 <= 0, so we clamp it to 0
        p->m_v1.a = 1.0f;
        p->m_count = 1;
        return;
    }

    // w2 region
    d12_1 = b2Vec2DotProduct(w2, e12);
    if (d12_1 <= 0.0f)
    {
        // a1 <= 0, so we clamp it to 0
        p->m_v2.a = 1.0f;
        p->m_count = 1;
        p->m_v1 = p->m_v2;
        return;
    }

    // Must be in e12 region.
    inv_d12 = 1.0f / (d12_1 + d12_2);
    p->m_v1.a = d12_1 * inv_d12;
    p->m_v2.a = d12_2 * inv_d12;
    p->m_count = 2;
}

// Possible regions:
// - points[2]
// - edge points[0]-points[2]
// - edge points[1]-points[2]
// - inside the triangle
static
void
b2SimplexSolve3(
    struct b2Simplex* p)
{
    b2Vec2 w1;
    b2Vec2 w2;
    b2Vec2 w3;

    b2Vec2 e12;
    float w1e12;
    float w2e12;
    float d12_1;
    float d12_2;

    b2Vec2 e13;
    float w1e13;
    float w3e13;
    float d13_1;
    float d13_2;

    b2Vec2 e23;
    float w2e23;
    float w3e23;
    float d23_1;
    float d23_2;

    float n123;

    float d123_1;
    float d123_2;
    float d123_3;

    float inv_d123;

    b2Vec2Assign(w1, p->m_v1.w);
    b2Vec2Assign(w2, p->m_v2.w);
    b2Vec2Assign(w3, p->m_v3.w);

    // Edge12
    // [1      1     ][a1] = [1]
    // [w1.e12 w2.e12][a2] = [0]
    // a3 = 0
    b2Vec2Sub(e12, w2, w1);
    w1e12 = b2Vec2DotProduct(w1, e12);
    w2e12 = b2Vec2DotProduct(w2, e12);
    d12_1 = w2e12;
    d12_2 = -w1e12;

    // Edge13
    // [1      1     ][a1] = [1]
    // [w1.e13 w3.e13][a3] = [0]
    // a2 = 0
    b2Vec2Sub(e13, w3, w1);
    w1e13 = b2Vec2DotProduct(w1, e13);
    w3e13 = b2Vec2DotProduct(w3, e13);
    d13_1 = w3e13;
    d13_2 = -w1e13;

    // Edge23
    // [1      1     ][a2] = [1]
    // [w2.e23 w3.e23][a3] = [0]
    // a1 = 0
    b2Vec2Sub(e23, w3, w2);
    w2e23 = b2Vec2DotProduct(w2, e23);
    w3e23 = b2Vec2DotProduct(w3, e23);
    d23_1 = w3e23;
    d23_2 = -w2e23;

    // Triangle123
    n123 = b2Vec2CrossProduct(e12, e13);

    d123_1 = n123 * b2Vec2CrossProduct(w2, w3);
    d123_2 = n123 * b2Vec2CrossProduct(w3, w1);
    d123_3 = n123 * b2Vec2CrossProduct(w1, w2);

    // w1 region
    if (d12_2 <= 0.0f && d13_2 <= 0.0f)
    {
        p->m_v1.a = 1.0f;
        p->m_count = 1;
        return;
    }

    // e12
    if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
    {
        float inv_d12 = 1.0f / (d12_1 + d12_2);
        p->m_v1.a = d12_1 * inv_d12;
        p->m_v2.a = d12_2 * inv_d12;
        p->m_count = 2;
        return;
    }

    // e13
    if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
    {
        float inv_d13 = 1.0f / (d13_1 + d13_2);
        p->m_v1.a = d13_1 * inv_d13;
        p->m_v3.a = d13_2 * inv_d13;
        p->m_count = 2;
        p->m_v2 = p->m_v3;
        return;
    }

    // w2 region
    if (d12_1 <= 0.0f && d23_2 <= 0.0f)
    {
        p->m_v2.a = 1.0f;
        p->m_count = 1;
        p->m_v1 = p->m_v2;
        return;
    }

    // w3 region
    if (d13_1 <= 0.0f && d23_1 <= 0.0f)
    {
        p->m_v3.a = 1.0f;
        p->m_count = 1;
        p->m_v1 = p->m_v3;
        return;
    }

    // e23
    if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
    {
        float inv_d23 = 1.0f / (d23_1 + d23_2);
        p->m_v2.a = d23_1 * inv_d23;
        p->m_v3.a = d23_2 * inv_d23;
        p->m_count = 2;
        p->m_v1 = p->m_v3;
        return;
    }

    // Must be in triangle123
    inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
    p->m_v1.a = d123_1 * inv_d123;
    p->m_v2.a = d123_2 * inv_d123;
    p->m_v3.a = d123_3 * inv_d123;
    p->m_count = 3;
}

B2_API
void
b2DistanceProxyReset(
    struct b2DistanceProxy* p)
{
    p->m_vertices = NULL;
    p->m_count = 0;
    p->m_radius = 0.0f;
}

B2_API
void
b2DistanceProxySetShape(
    struct b2DistanceProxy* p,
    const struct b2Shape* shape,
    int32 index)
{
    switch (b2ShapeGetType(shape))
    {
    case b2ShapeTypeCircle:
    {
        const struct b2ShapeCircle* circle;
        circle = (const struct b2ShapeCircle*)(shape);
        p->m_vertices = &circle->m_p;
        p->m_count = 1;
        p->m_radius = circle->m_radius;
    }
    break;

    case b2ShapeTypePolygon:
    {
        const struct b2ShapePolygon* polygon = (const struct b2ShapePolygon*)(shape);
        p->m_vertices = polygon->m_vertices;
        p->m_count = polygon->m_count;
        p->m_radius = polygon->m_radius;
    }
    break;

    case b2ShapeTypeChain:
    {
        const struct b2ShapeChain* chain;
        chain = (const struct b2ShapeChain*)(shape);
        b2Assert(0 <= index && index < chain->m_count);

        b2Vec2Assign(p->m_buffer[0], chain->m_vertices[index]);
        if (index + 1 < chain->m_count)
        {
            b2Vec2Assign(p->m_buffer[1], chain->m_vertices[index + 1]);
        }
        else
        {
            b2Vec2Assign(p->m_buffer[1], chain->m_vertices[0]);
        }

        p->m_vertices = p->m_buffer;
        p->m_count = 2;
        p->m_radius = chain->m_radius;
    }
    break;

    case b2ShapeTypeEdge:
    {
        const struct b2ShapeEdge* edge;
        edge = (const struct b2ShapeEdge*)(shape);
        p->m_vertices = &edge->m_vertex1;
        p->m_count = 2;
        p->m_radius = edge->m_radius;
    }
    break;

    default:
        b2Assert(b2False);
    }
}

B2_API
void
b2DistanceProxySetVertex(
    struct b2DistanceProxy* p,
    const b2Vec2* vertices,
    int32 count,
    float radius)
{
    p->m_vertices = vertices;
    p->m_count = count;
    p->m_radius = radius;
}

B2_API
int32
b2DistanceProxyGetSupport(
    const struct b2DistanceProxy* p,
    const b2Vec2 d)
{
    int32 i;
    int32 bestIndex = 0;
    float bestValue = b2Vec2DotProduct(p->m_vertices[0], d);
    for (i = 1; i < p->m_count; ++i)
    {
        float value = b2Vec2DotProduct(p->m_vertices[i], d);
        if (value > bestValue)
        {
            bestIndex = i;
            bestValue = value;
        }
    }

    return bestIndex;
}

B2_API
b2Vec2ConstRef
b2DistanceProxyGetSupportVertex(
    const struct b2DistanceProxy* p,
    const b2Vec2 d)
{
    int32 i;
    int32 bestIndex = 0;
    float bestValue = b2Vec2DotProduct(p->m_vertices[0], d);
    for (i = 1; i < p->m_count; ++i)
    {
        float value = b2Vec2DotProduct(p->m_vertices[i], d);
        if (value > bestValue)
        {
            bestIndex = i;
            bestValue = value;
        }
    }

    return p->m_vertices[bestIndex];
}

B2_API
b2Vec2ConstRef
b2DistanceProxyGetVertex(
    const struct b2DistanceProxy* p,
    int32 index)
{
    b2Assert(0 <= index && index < p->m_count);
    return p->m_vertices[index];
}

B2_API
void
b2DistanceInputReset(
    struct b2DistanceInput* p)
{
    b2DistanceProxyReset(&p->proxyA);
    b2DistanceProxyReset(&p->proxyB);
}

B2_API
void
b2Distance(
    struct b2DistanceOutput* output,
    struct b2SimplexCache* cache,
    const struct b2DistanceInput* input)
{
    const struct b2DistanceProxy* proxyA;
    const struct b2DistanceProxy* proxyB;

    b2TransformConstRef transformA;
    b2TransformConstRef transformB;

    struct b2Simplex simplex;

    struct b2SimplexVertex* vertices;

    int32 saveA[3], saveB[3];
    int32 saveCount;

    // Main iteration loop.
    int32 iter;

    const int32 k_maxIters = 20;

    ++b2_gjkCalls;

    proxyA = &input->proxyA;
    proxyB = &input->proxyB;

    transformA = input->transformA;
    transformB = input->transformB;

    // Initialize the simplex.
    b2SimplexReadCache(&simplex, cache, proxyA, transformA, proxyB, transformB);

    // Get simplex vertices as an array.
    vertices = &simplex.m_v1;

    // These store the vertices of the last simplex so that we
    // can check for duplicates and prevent cycling.
    saveCount = 0;

    // Main iteration loop.
    iter = 0;
    while (iter < k_maxIters)
    {
        int32 i;
        b2Vec2 d;
        struct b2SimplexVertex* vertex;

        b2Vec2 v;
        int duplicate;

        // Copy simplex so we can identify duplicates.
        saveCount = simplex.m_count;
        for (i = 0; i < saveCount; ++i)
        {
            saveA[i] = vertices[i].indexA;
            saveB[i] = vertices[i].indexB;
        }

        switch (simplex.m_count)
        {
        case 1:
            break;

        case 2:
            b2SimplexSolve2(&simplex);
            break;

        case 3:
            b2SimplexSolve3(&simplex);
            break;

        default:
            b2Assert(b2False);
        }

        // If we have 3 points, then the origin is in the corresponding triangle.
        if (simplex.m_count == 3)
        {
            break;
        }

        // Get search direction.
        b2SimplexGetSearchDirection(&simplex, d);

        // Ensure the search direction is numerically fit.
        if (b2Vec2SquaredLength(d) < b2_epsilon * b2_epsilon)
        {
            // The origin is probably contained by a line segment
            // or triangle. Thus the shapes are overlapped.

            // We can't return zero here even though there may be overlap.
            // In case the simplex is a point, segment, or triangle it is difficult
            // to determine if the origin is contained in the CSO or very close to it.
            break;
        }

        // Compute a tentative new simplex vertex using support points.
        vertex = vertices + simplex.m_count;

        b2Vec2Negate(v, d);
        b2RotMulTVec2(v, transformA[1], v);
        vertex->indexA = b2DistanceProxyGetSupport(proxyA, v);
        b2TransformMulVec2(vertex->wA, transformA, b2DistanceProxyGetVertex(proxyA, vertex->indexA));
        b2RotMulTVec2(v, transformB[1], d);
        vertex->indexB = b2DistanceProxyGetSupport(proxyB, v);
        b2TransformMulVec2(vertex->wB, transformB, b2DistanceProxyGetVertex(proxyB, vertex->indexB));
        b2Vec2Sub(vertex->w, vertex->wB, vertex->wA);

        // Iteration count is equated to the number of support point calls.
        ++iter;
        ++b2_gjkIters;

        // Check for duplicate support points. This is the main termination criteria.
        duplicate = b2False;
        for (i = 0; i < saveCount; ++i)
        {
            if (vertex->indexA == saveA[i] && 
                vertex->indexB == saveB[i])
            {
                duplicate = b2True;
                break;
            }
        }

        // If we found a duplicate support point we must exit to avoid cycling.
        if (duplicate)
        {
            break;
        }

        // New vertex is ok and needed.
        ++simplex.m_count;
    }

    b2_gjkMaxIters = b2MaxInt32(b2_gjkMaxIters, iter);

    // Prepare output.
    b2SimplexGetWitnessPoints(&simplex, output->pointA, output->pointB);
    output->distance = b2Vec2Distance(output->pointA, output->pointB);
    output->iterations = iter;

    // Cache the simplex.
    b2SimplexWriteCache(&simplex, cache);

    // Apply radii if requested
    if (input->useRadii)
    {
        if (output->distance < b2_epsilon)
        {
            b2Vec2 v;
            b2Vec2 p;
            // Shapes are too close to safely compute normal
            b2Vec2Add(v, output->pointA, output->pointB);
            b2Vec2Scale(p, v, 0.5f);
            b2Vec2Assign(output->pointA, p);
            b2Vec2Assign(output->pointB, p);
            output->distance = 0.0f;
        }
        else
        {
            // Keep closest points on perimeter even if overlapped, this way
            // the points move smoothly.
            b2Vec2 v;
            b2Vec2 normal;
            float rA = proxyA->m_radius;
            float rB = proxyB->m_radius;
            b2Vec2Sub(normal, output->pointB, output->pointA);
            b2Vec2Normalize(normal, normal);
            output->distance = b2MaxFloat(0.0f, output->distance - rA - rB);
            b2Vec2Scale(v, normal, rA);
            b2Vec2Add(output->pointA, output->pointA, v);
            b2Vec2Scale(v, normal, rB);
            b2Vec2Sub(output->pointB, output->pointB, v);
        }
    }
}

B2_API
void
b2ShapeCastInputReset(
    struct b2ShapeCastInput* p)
{
    b2DistanceProxyReset(&p->proxyA);
    b2DistanceProxyReset(&p->proxyB);
}

// GJK-raycast
// Algorithm by Gino van den Bergen.
// "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010
B2_API
int
b2ShapeCast(
    struct b2ShapeCastOutput* output,
    const struct b2ShapeCastInput* input)
{
    const struct b2DistanceProxy* proxyA;
    const struct b2DistanceProxy* proxyB;

    float radiusA;
    float radiusB;
    float radius;

    b2TransformConstRef xfA;
    b2TransformConstRef xfB;

    b2Vec2 r;
    b2Vec2 n;
    float lambda;

    struct b2Simplex simplex;

    struct b2SimplexVertex* vertices;

    int32 indexA;
    b2Vec2 wA;
    int32 indexB;
    b2Vec2 wB;
    b2Vec2 v;

    float sigma;

    int32 iter;
    
    b2Vec2 pointA, pointB;

    const float tolerance = 0.5f * b2_linearSlop;

    const int32 k_maxIters = 20;

    output->iterations = 0;
    output->lambda = 1.0f;
    b2Vec2SetZero(output->normal);
    b2Vec2SetZero(output->point);

    proxyA = &input->proxyA;
    proxyB = &input->proxyB;

    radiusA = b2MaxFloat(proxyA->m_radius, b2_polygonRadius);
    radiusB = b2MaxFloat(proxyB->m_radius, b2_polygonRadius);
    radius = radiusA + radiusB;

    xfA = input->transformA;
    xfB = input->transformB;

    b2Vec2Assign(r, input->translationB);
    b2Vec2Make(n, 0.0f, 0.0f);

    lambda = 0.0f;

    // Initial simplex
    simplex.m_count = 0;

    // Get simplex vertices as an array.
    vertices = &simplex.m_v1;

    // Get support point in -r direction
    b2Vec2Negate(v, r);
    b2RotMulTVec2(v, xfA[1], v);
    indexA = b2DistanceProxyGetSupport(proxyA, v);
    b2TransformMulVec2(wA, xfA, b2DistanceProxyGetVertex(proxyA, indexA));
    b2RotMulTVec2(v, xfB[1], r);
    indexB = b2DistanceProxyGetSupport(proxyB, v);
    b2TransformMulVec2(wB, xfB, b2DistanceProxyGetVertex(proxyB, indexB));
    b2Vec2Sub(v, wA, wB);

    // Sigma is the target distance between polygons
    sigma = b2MaxFloat(b2_polygonRadius, radius - b2_polygonRadius);

    // Main iteration loop.
    iter = 0;
    while (iter < k_maxIters && b2Vec2Length(v) - sigma > tolerance)
    {
        b2Vec2 t;
        b2Vec2 p;

        float vp;
        float vr;

        struct b2SimplexVertex* vertex;

        b2Assert(simplex.m_count < 3);

        output->iterations += 1;

        // Support in direction -v (A - B)
        b2Vec2Negate(t, v);
        b2RotMulTVec2(t, xfA[1], t);
        indexA = b2DistanceProxyGetSupport(proxyA, t);
        b2TransformMulTVec2(wA, xfA, b2DistanceProxyGetVertex(proxyA, indexA));
        b2RotMulTVec2(t, xfB[1], v);
        indexB = b2DistanceProxyGetSupport(proxyB, t);
        b2TransformMulTVec2(wB, xfB, b2DistanceProxyGetVertex(proxyB, indexB));
        b2Vec2Sub(p, wA, wB);

        // -v is a normal at p
        b2Vec2Normalize(v, v);

        // Intersect ray with plane
        vp = b2Vec2DotProduct(v, p);
        vr = b2Vec2DotProduct(v, r);
        if (vp - sigma > lambda * vr)
        {
            if (vr <= 0.0f)
            {
                return b2False;
            }

            lambda = (vp - sigma) / vr;
            if (lambda > 1.0f)
            {
                return b2False;
            }

            b2Vec2Negate(n, v);
            simplex.m_count = 0;
        }

        // Reverse simplex since it works with B - A.
        // Shift by lambda * r because we want the closest point to the current clip point.
        // Note that the support point p is not shifted because we want the plane equation
        // to be formed in unshifted space.
        vertex = vertices + simplex.m_count;
        vertex->indexA = indexB;
        b2Vec2Scale(t, r, lambda);
        b2Vec2Add(vertex->wA, wB, t);
        vertex->indexB = indexA;
        b2Vec2Assign(vertex->wB, wA);
        b2Vec2Sub(vertex->w, vertex->wB, vertex->wA);
        vertex->a = 1.0f;
        simplex.m_count += 1;

        switch (simplex.m_count)
        {
        case 1:
            break;

        case 2:
            b2SimplexSolve2(&simplex);
            break;

        case 3:
            b2SimplexSolve3(&simplex);
            break;

        default:
            b2Assert(b2False);
        }

        // If we have 3 points, then the origin is in the corresponding triangle.
        if (simplex.m_count == 3)
        {
            // Overlap
            return b2False;
        }

        // Get search direction.
        b2SimplexGetClosestPoint(&simplex, v);

        // Iteration count is equated to the number of support point calls.
        ++iter;
    }

    if (iter == 0)
    {
        // Initial overlap
        return b2False;
    }

    // Prepare output.
    b2SimplexGetWitnessPoints(&simplex, pointB, pointA);

    if (b2Vec2SquaredLength(v) > 0.0f)
    {
        b2Vec2Negate(n, v);
        b2Vec2Normalize(n, n);
    }

    b2Vec2Scale(v, n, radiusA);
    b2Vec2Add(output->point, pointA, v);
    b2Vec2Assign(output->normal, n);
    output->lambda = lambda;
    output->iterations = iter;
    return b2True;
}
