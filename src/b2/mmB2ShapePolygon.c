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

#include "mmB2ShapePolygon.h"
#include "mmB2BlockAllocator.h"
#include "mmB2Common.h"
#include "mmB2Collision.h"

#include <assert.h>

static 
void 
ComputeCentroid(
    b2Vec2 c,
    const b2Vec2* vs, 
    int32 count)
{
    int32 i;

    float area;

    b2Vec2ConstRef s;

    b2Vec2 v;

    static const float inv3 = 1.0f / 3.0f;

    b2Assert(count >= 3);

    b2Vec2Make(c, 0.0f, 0.0f);
    area = 0.0f;

    // Get a reference point for forming triangles.
    // Use the first vertex to reduce round-off errors.
    s = vs[0];

    for (i = 0; i < count; ++i)
    {
        b2Vec2 p1;
        b2Vec2 p2;
        b2Vec2 p3;

        b2Vec2 e1;
        b2Vec2 e2;

        float D;

        float triangleArea;

        // Triangle vertices.
        b2Vec2Sub(p1, vs[0], s);
        b2Vec2Sub(p2, vs[i], s);
        if (i + 1 < count)
        {
            b2Vec2Sub(p3, vs[i + 1], s);
        }
        else
        {
            b2Vec2Sub(p3, vs[0], s);
        }

        b2Vec2Sub(e1, p2, p1);
        b2Vec2Sub(e2, p3, p1);

        D = b2Vec2CrossProduct(e1, e2);

        triangleArea = 0.5f * D;
        area += triangleArea;

        // Area weighted centroid
        b2Vec2Add(v, p1, p2);
        b2Vec2Add(v, v, p3);
        b2Vec2Scale(v, v, triangleArea * inv3);
        b2Vec2Add(c, c, v);
    }

    // Centroid
    b2Assert(area > b2_epsilon);
    b2Vec2Scale(v, c, 1.0f / area);
    b2Vec2Add(c, v, s);
}

B2_API const struct b2MetaAllocator b2MetaAllocatorShapePolygon =
{
    "b2ShapePolygon",
    sizeof(struct b2ShapePolygon),
    &b2ShapePolygonInit,
    &b2ShapePolygonDestroy,
};

B2_API const struct b2ShapeMeta b2MetaShapePolygon =
{
    &b2ShapePolygonClone,
    &b2ShapePolygonGetChildCount,
    &b2ShapePolygonTestPoint,
    &b2ShapePolygonRayCast,
    &b2ShapePolygonComputeAABB,
    &b2ShapePolygonComputeMass,
};

B2_API
void
b2ShapePolygonInit(
    struct b2ShapePolygon* p)
{
    p->Meta = &b2MetaShapePolygon;
    p->m_type = b2ShapeTypePolygon;
    p->m_radius = b2_polygonRadius;
    p->m_count = 0;
    b2Vec2SetZero(p->m_centroid);
}

B2_API
void
b2ShapePolygonDestroy(
    struct b2ShapePolygon* p)
{
    b2Vec2SetZero(p->m_centroid);
    p->m_count = 0;
    p->m_radius = b2_polygonRadius;
    p->m_type = b2ShapeTypePolygon;
    p->Meta = &b2MetaShapePolygon;
}

B2_API
void
b2ShapePolygonReset(
    struct b2ShapePolygon* p)
{
    p->Meta = &b2MetaShapePolygon;
    p->m_type = b2ShapeTypePolygon;
    p->m_radius = b2_polygonRadius;
    p->m_count = 0;
    b2Vec2SetZero(p->m_centroid);
}

B2_API
struct b2Shape*
b2ShapePolygonClone(
    const struct b2ShapePolygon* p,
    struct b2BlockAllocator* allocator)
{
    void* mem;
    struct b2ShapePolygon* clone;
    mem = b2BlockAllocatorAllocate(allocator, sizeof(struct b2ShapePolygon));
    clone = (struct b2ShapePolygon*)mem;
    *clone = *p;
    return (struct b2Shape*)clone;
}

B2_API
int32
b2ShapePolygonGetChildCount(
    const struct b2ShapePolygon* p)
{
    return 1;
}

B2_API
int
b2ShapePolygonTestPoint(
    const struct b2ShapePolygon* p,
    const b2Transform transform, 
    const b2Vec2 point)
{
    int32 i;

    b2Vec2 v;
    b2Vec2 pLocal;

    b2Vec2Sub(v, point, transform[0]);
    b2RotMulTVec2(pLocal, transform[1], v);

    for (i = 0; i < p->m_count; ++i)
    {
        float dot;

        b2Vec2Sub(v, pLocal, p->m_vertices[i]);
        dot = b2Vec2DotProduct(p->m_normals[i], v);
        if (dot > 0.0f)
        {
            return b2False;
        }
    }

    return b2True;
}

B2_API
int
b2ShapePolygonRayCast(
    const struct b2ShapePolygon* p,
    struct b2RayCastOutput* output,
    const struct b2RayCastInput* input,
    const b2Transform transform,
    int32 childIndex)
{
    b2Vec2 v;

    b2Vec2 p1;
    b2Vec2 p2;
    b2Vec2 d;

    float lower, upper;

    int32 index;

    int32 i;

    B2_NOT_USED(childIndex);

    // Put the ray into the polygon's frame of reference.
    b2Vec2Sub(v, input->p1, transform[0]);
    b2RotMulTVec2(p1, transform[1], v);
    b2Vec2Sub(v, input->p2, transform[0]);
    b2RotMulTVec2(p2, transform[1], v);
    b2Vec2Sub(d, p2, p1);

    lower = 0.0f; upper = input->maxFraction;

    index = -1;

    for (i = 0; i < p->m_count; ++i)
    {
        float numerator;
        float denominator;

        // p = p1 + a * d
        // dot(normal, p - v) = 0
        // dot(normal, p1 - v) + a * dot(normal, d) = 0
        b2Vec2Sub(v, p->m_vertices[i], p1);
        numerator = b2Vec2DotProduct(p->m_normals[i], v);
        denominator = b2Vec2DotProduct(p->m_normals[i], d);

        if (denominator == 0.0f)
        {
            if (numerator < 0.0f)
            {
                return b2False;
            }
        }
        else
        {
            // Note: we want this predicate without division:
            // lower < numerator / denominator, where denominator < 0
            // Since denominator < 0, we have to flip the inequality:
            // lower < numerator / denominator <==> denominator * lower > numerator.
            if (denominator < 0.0f && numerator < lower * denominator)
            {
                // Increase lower.
                // The segment enters this half-space.
                lower = numerator / denominator;
                index = i;
            }
            else if (denominator > 0.0f && numerator < upper * denominator)
            {
                // Decrease upper.
                // The segment exits this half-space.
                upper = numerator / denominator;
            }
        }

        // The use of epsilon here causes the assert on lower to trip
        // in some cases. Apparently the use of epsilon was to make edge
        // shapes work, but now those are handled separately.
        //if (upper < lower - b2_epsilon)
        if (upper < lower)
        {
            return b2False;
        }
    }

    b2Assert(0.0f <= lower && lower <= input->maxFraction);

    if (index >= 0)
    {
        output->fraction = lower;
        b2RotMulVec2(output->normal, transform[1], p->m_normals[index]);
        return b2True;
    }

    return b2False;
}

B2_API
void
b2ShapePolygonComputeAABB(
    const struct b2ShapePolygon* p,
    struct b2AABB* aabb,
    const b2Transform transform,
    int32 childIndex)
{
    int32 i;

    b2Vec2 lower;
    b2Vec2 upper;

    b2Vec2 r;

    B2_NOT_USED(childIndex);

    b2TransformMulVec2(lower, transform, p->m_vertices[0]);
    b2Vec2Assign(upper, lower);

    for (i = 1; i < p->m_count; ++i)
    {
        b2Vec2 v;
        b2TransformMulVec2(v, transform, p->m_vertices[i]);
        b2Vec2Min(lower, lower, v);
        b2Vec2Max(upper, upper, v);
    }

    r[0] = p->m_radius;
    r[1] = p->m_radius;
    b2Vec2Sub(aabb->lowerBound, lower, r);
    b2Vec2Add(aabb->upperBound, upper, r);
}

B2_API
void
b2ShapePolygonComputeMass(
    const struct b2ShapePolygon* p,
    struct b2MassData* massData,
    float density)
{
    // Polygon mass, centroid, and inertia.
    // Let rho be the polygon density in mass per unit area.
    // Then:
    // mass = rho * int(dA)
    // centroid.x = (1/mass) * rho * int(x * dA)
    // centroid.y = (1/mass) * rho * int(y * dA)
    // I = rho * int((x*x + y*y) * dA)
    //
    // We can compute these integrals by summing all the integrals
    // for each triangle of the polygon. To evaluate the integral
    // for a single triangle, we make a change of variables to
    // the (u,v) coordinates of the triangle:
    // x = x0 + e1x * u + e2x * v
    // y = y0 + e1y * u + e2y * v
    // where 0 <= u && 0 <= v && u + v <= 1.
    //
    // We integrate u from [0,1-v] and then v from [0,1].
    // We also need to use the Jacobian of the transformation:
    // D = cross(e1, e2)
    //
    // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
    //
    // The rest of the derivation is handled by computer algebra.

    b2Vec2 center;
    float area;
    float I;

    b2Vec2ConstRef s;

    int32 i;

    b2Vec2 v;

    float d0, d1;

    static const float k_inv3 = 1.0f / 3.0f;

    b2Assert(p->m_count >= 3);

    b2Vec2Make(center, 0.0f, 0.0f);
    area = 0.0f;
    I = 0.0f;

    // Get a reference point for forming triangles.
    // Use the first vertex to reduce round-off errors.
    s = p->m_vertices[0];

    for (i = 0; i < p->m_count; ++i)
    {
        b2Vec2 e1;
        b2Vec2 e2;

        float D;

        float triangleArea;

        float ex1, ey1;
        float ex2, ey2;

        float intx2;
        float inty2;

        // Triangle vertices.
        b2Vec2Sub(e1, p->m_vertices[i], s);
        if (i + 1 < p->m_count)
        {
            b2Vec2Sub(e2, p->m_vertices[i + 1], s);
        }
        else
        {
            b2Vec2Sub(e2, p->m_vertices[0], s);
        }

        D = b2Vec2CrossProduct(e1, e2);

        triangleArea = 0.5f * D;
        area += triangleArea;

        // Area weighted centroid
        b2Vec2Add(v, e1, e2);
        b2Vec2Scale(v, v, triangleArea * k_inv3);
        b2Vec2Add(center, center, v);

        ex1 = e1[0]; ey1 = e1[1];
        ex2 = e2[0]; ey2 = e2[1];

        intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
        inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

        I += (0.25f * k_inv3 * D) * (intx2 + inty2);
    }

    // Total mass
    massData->mass = density * area;

    // Center of mass
    b2Assert(area > b2_epsilon);
    b2Vec2Scale(center, center, 1.0f / area);
    b2Vec2Add(massData->center, center, s);

    // Inertia tensor relative to the local origin (point s).
    massData->I = density * I;

    d0 = b2Vec2DotProduct(massData->center, massData->center);
    d1 = b2Vec2DotProduct(center, center);
    // Shift to center of mass then to original body origin.
    massData->I += massData->mass * (d0 - d1);
}

B2_API
int
b2ShapePolygonSetPoints(
    struct b2ShapePolygon* p,
    const b2Vec2* points,
    int32 count)
{
    struct b2Hull hull;

    b2ComputeHull(&hull, points, count);

    if (hull.count < 3)
    {
        return b2False;
    }

    b2ShapePolygonSetHull(p, &hull);

    return b2True;
}

B2_API
void
b2ShapePolygonSetHull(
    struct b2ShapePolygon* p,
    const struct b2Hull* hull)
{
    int32 i;

    b2Assert(hull->count >= 3);

    p->m_count = hull->count;

    // Copy vertices
    //mmMemcpy(p->m_vertices, hull->points, sizeof(b2Vec2) * p->m_count);
    for (i = 0; i < hull->count; ++i)
    {
        b2Vec2Assign(p->m_vertices[i], hull->points[i]);
    }

    // Compute normals. Ensure the edges have non-zero length.
    for (i = 0; i < p->m_count; ++i)
    {
        b2Vec2 edge;
        int32 i1 = i;
        int32 i2 = i + 1 < p->m_count ? i + 1 : 0;
        b2Vec2Sub(edge, p->m_vertices[i2], p->m_vertices[i1]);
        b2Assert(b2Vec2SquaredLength(edge) > b2_epsilon * b2_epsilon);
        b2Vec2CrossProductKR(p->m_normals[i], edge, 1.0f);
        b2Vec2Normalize(p->m_normals[i], p->m_normals[i]);
    }

    // Compute the polygon centroid.
    ComputeCentroid(p->m_centroid, p->m_vertices, p->m_count);
}

B2_API
void
b2ShapePolygonSetAsBox(
    struct b2ShapePolygon* p,
    float hx,
    float hy)
{
    p->m_count = 4;
    b2Vec2Make(p->m_vertices[0], -hx, -hy);
    b2Vec2Make(p->m_vertices[1], +hx, -hy);
    b2Vec2Make(p->m_vertices[2], +hx, +hy);
    b2Vec2Make(p->m_vertices[3], -hx, +hy);
    b2Vec2Make(p->m_normals[0], +0.0f, -1.0f);
    b2Vec2Make(p->m_normals[1], +1.0f, +0.0f);
    b2Vec2Make(p->m_normals[2], +0.0f, +1.0f);
    b2Vec2Make(p->m_normals[3], -1.0f, +0.0f);
    b2Vec2SetZero(p->m_centroid);
}

B2_API
void
b2ShapePolygonSetAsBoxDetail(
    struct b2ShapePolygon* p,
    float hx,
    float hy,
    const b2Vec2 center,
    float angle)
{
    b2Transform xf;

    p->m_count = 4;
    b2Vec2Make(p->m_vertices[0], -hx, -hy);
    b2Vec2Make(p->m_vertices[1], +hx, -hy);
    b2Vec2Make(p->m_vertices[2], +hx, +hy);
    b2Vec2Make(p->m_vertices[3], -hx, +hy);
    b2Vec2Make(p->m_normals[0], +0.0f, -1.0f);
    b2Vec2Make(p->m_normals[1], +1.0f, +0.0f);
    b2Vec2Make(p->m_normals[2], +0.0f, +1.0f);
    b2Vec2Make(p->m_normals[3], -1.0f, +0.0f);
    b2Vec2Assign(p->m_centroid, center);

    b2TransformSet(xf, center, angle);

    // Transform vertices and normals.
    b2TransformMulVec2(p->m_vertices[0], xf, p->m_vertices[0]);
    b2RotMulVec2(p->m_normals[0], xf[1], p->m_normals[0]);

    b2TransformMulVec2(p->m_vertices[1], xf, p->m_vertices[1]);
    b2RotMulVec2(p->m_normals[1], xf[1], p->m_normals[1]);

    b2TransformMulVec2(p->m_vertices[2], xf, p->m_vertices[2]);
    b2RotMulVec2(p->m_normals[2], xf[1], p->m_normals[2]);

    b2TransformMulVec2(p->m_vertices[3], xf, p->m_vertices[3]);
    b2RotMulVec2(p->m_normals[3], xf[1], p->m_normals[3]);
}

B2_API
int
b2ShapePolygonValidate(
    const struct b2ShapePolygon* p)
{
    int32 i;
    struct b2Hull hull;

    if (p->m_count < 3 || b2_maxPolygonVertices < p->m_count)
    {
        return b2False;
    }

    //mmMemcpy(hull.points, p->m_vertices, sizeof(b2Vec2) * p->m_count);
    //hull.count = p->m_count;
    for (i = 0; i < p->m_count; ++i)
    {
        b2Vec2Assign(hull.points[i], p->m_vertices[i]);
    }

    hull.count = p->m_count;

    return b2ValidateHull(&hull);
}
