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

#include "mmB2Collision.h"
#include "mmB2ShapeEdge.h"
#include "mmB2ShapeCircle.h"
#include "mmB2ShapePolygon.h"

#include <assert.h>

// Compute contact points for edge versus circle.
// This accounts for edge connectivity.
B2_API
void
b2CollideEdgeAndCircle(
    struct b2Manifold* manifold,
    const struct b2ShapeEdge* edgeA, const b2Transform xfA,
    const struct b2ShapeCircle* circleB, const b2Transform xfB)
{
    b2Vec2 t;
    b2Vec2 t0, t1;

    b2Vec2 Q;

    b2Vec2ConstRef A, B;
    b2Vec2 e;

    // Normal points to the right for a CCW winding
    b2Vec2 n;
    float offset;

    int oneSided;

    float u;
    float v;

    float radius;

    struct b2ContactFeature cf;

    float den;
    b2Vec2 P;
    b2Vec2 d;
    float dd;

    manifold->pointCount = 0;

    // Compute circle in frame of edge
    b2TransformMulVec2(t, xfB, circleB->m_p);
    b2TransformMulTVec2(Q, xfA, t);

    A = edgeA->m_vertex1; B = edgeA->m_vertex2;
    b2Vec2Sub(e, B, A);

    // Normal points to the right for a CCW winding
    n[0] = +e[1]; n[1] = -e[0];
    b2Vec2Sub(t, Q, A);
    offset = b2Vec2DotProduct(n, t);

    oneSided = edgeA->m_oneSided;
    if (oneSided && offset < 0.0f)
    {
        return;
    }

    // Barycentric coordinates
    b2Vec2Sub(t, B, Q);
    u = b2Vec2DotProduct(e, t);
    b2Vec2Sub(t, Q, A);
    v = b2Vec2DotProduct(e, t);

    radius = edgeA->m_radius + circleB->m_radius;

    cf.indexB = 0;
    cf.typeB = b2ContactFeatureTypeVertex;

    // Region A
    if (v <= 0.0f)
    {
        b2Vec2 P;
        b2Vec2 d;
        float dd;

        b2Vec2Assign(P, A);
        b2Vec2Sub(d, Q, P);
        dd = b2Vec2DotProduct(d, d);
        if (dd > radius * radius)
        {
            return;
        }

        // Is there an edge connected to A?
        if (edgeA->m_oneSided)
        {
            b2Vec2 A1;
            b2Vec2 B1;
            b2Vec2 e1;

            float u1;

            b2Vec2Assign(A1, edgeA->m_vertex0);
            b2Vec2Assign(B1, A);
            b2Vec2Sub(e1, B1, A1);
            b2Vec2Sub(t, B1, Q);
            u1 = b2Vec2DotProduct(e1, t);

            // Is the circle in Region AB of the previous edge?
            if (u1 > 0.0f)
            {
                return;
            }
        }

        cf.indexA = 0;
        cf.typeA = b2ContactFeatureTypeVertex;
        manifold->pointCount = 1;
        manifold->type = b2ManifoldTypeCircles;
        b2Vec2SetZero(manifold->localNormal);
        b2Vec2Assign(manifold->localPoint, P);
        manifold->points[0].id.key = 0;
        manifold->points[0].id.cf = cf;
        b2Vec2Assign(manifold->points[0].localPoint, circleB->m_p);
        return;
    }

    // Region B
    if (u <= 0.0f)
    {
        b2Vec2 P;
        b2Vec2 d;
        float dd;

        b2Vec2Assign(P, B);
        b2Vec2Sub(d, Q, P);
        dd = b2Vec2DotProduct(d, d);
        if (dd > radius * radius)
        {
            return;
        }

        // Is there an edge connected to B?
        if (edgeA->m_oneSided)
        {
            b2Vec2 B2;
            b2Vec2 A2;
            b2Vec2 e2;
            float v2;

            b2Vec2Assign(B2, edgeA->m_vertex3);
            b2Vec2Assign(A2, B);
            b2Vec2Sub(e2, B2, A2);
            b2Vec2Sub(t, Q, A2);
            v2 = b2Vec2DotProduct(e2, t);

            // Is the circle in Region AB of the next edge?
            if (v2 > 0.0f)
            {
                return;
            }
        }

        cf.indexA = 1;
        cf.typeA = b2ContactFeatureTypeVertex;
        manifold->pointCount = 1;
        manifold->type = b2ManifoldTypeCircles;
        b2Vec2SetZero(manifold->localNormal);
        b2Vec2Assign(manifold->localPoint, P);
        manifold->points[0].id.key = 0;
        manifold->points[0].id.cf = cf;
        b2Vec2Assign(manifold->points[0].localPoint, circleB->m_p);
        return;
    }

    // Region AB
    den = b2Vec2DotProduct(e, e);
    b2Assert(den > 0.0f);
    b2Vec2Scale(t0, A, u);
    b2Vec2Scale(t1, B, v);
    b2Vec2Add(t, t0, t1);
    b2Vec2Scale(P, t, 1.0f / den);
    b2Vec2Sub(d, Q, P);
    dd = b2Vec2DotProduct(d, d);
    if (dd > radius * radius)
    {
        return;
    }

    if (offset < 0.0f)
    {
        b2Vec2Make(n, -n[0], -n[1]);
    }
    b2Vec2Normalize(n, n);

    cf.indexA = 0;
    cf.typeA = b2ContactFeatureTypeFace;
    manifold->pointCount = 1;
    manifold->type = b2ManifoldTypeFaceA;
    b2Vec2Assign(manifold->localNormal, n);
    b2Vec2Assign(manifold->localPoint, A);
    manifold->points[0].id.key = 0;
    manifold->points[0].id.cf = cf;
    b2Vec2Assign(manifold->points[0].localPoint, circleB->m_p);
}

enum b2EPAxisType
{
    b2EPAxisUnknown,
    b2EPAxisEdgeA,
    b2EPAxisEdgeB,
};
// This structure is used to keep track of the best separating axis.
struct b2EPAxis
{
    b2Vec2 normal;
    enum b2EPAxisType type;
    int32 index;
    float separation;
};

// This holds polygon B expressed in frame A.
struct b2TempPolygon
{
    b2Vec2 vertices[b2_maxPolygonVertices];
    b2Vec2 normals[b2_maxPolygonVertices];
    int32 count;
};

// Reference face used for clipping
struct b2ReferenceFace
{
    int32 i1, i2;
    b2Vec2 v1, v2;
    b2Vec2 normal;

    b2Vec2 sideNormal1;
    float sideOffset1;

    b2Vec2 sideNormal2;
    float sideOffset2;
};

static 
void 
b2ComputeEdgeSeparation(
    struct b2EPAxis* axis,
    const struct b2TempPolygon* polygonB, 
    const b2Vec2 v1, 
    const b2Vec2 normal1)
{
    int32 i, j;
    b2Vec2 axes[2];

    axis->type = b2EPAxisEdgeA;
    axis->index = -1;
    axis->separation = -FLT_MAX;
    b2Vec2SetZero(axis->normal);

    b2Vec2Assign(axes[0], normal1);
    b2Vec2Negate(axes[1], normal1);

    // Find axis with least overlap (min-max problem)
    for (j = 0; j < 2; ++j)
    {
        float sj = FLT_MAX;

        // Find deepest polygon vertex along axis j
        for (i = 0; i < polygonB->count; ++i)
        {
            b2Vec2 v;
            float si;
            b2Vec2Sub(v, polygonB->vertices[i], v1);
            si = b2Vec2DotProduct(axes[j], v);
            if (si < sj)
            {
                sj = si;
            }
        }

        if (sj > axis->separation)
        {
            axis->index = j;
            axis->separation = sj;
            b2Vec2Assign(axis->normal, axes[j]);
        }
    }
}

static 
void
b2ComputePolygonSeparation(
    struct b2EPAxis* axis, 
    const struct b2TempPolygon* polygonB, 
    const b2Vec2 v1, 
    const b2Vec2 v2)
{
    int32 i;

    axis->type = b2EPAxisUnknown;
    axis->index = -1;
    axis->separation = -FLT_MAX;
    b2Vec2SetZero(axis->normal);

    for (i = 0; i < polygonB->count; ++i)
    {
        float s1;
        float s2;
        float s;

        b2Vec2 n;
        b2Vec2 v;

        b2Vec2Negate(n, polygonB->normals[i]);

        b2Vec2Sub(v, polygonB->vertices[i], v1);
        s1 = b2Vec2DotProduct(n, v);
        b2Vec2Sub(v, polygonB->vertices[i], v2);
        s2 = b2Vec2DotProduct(n, v);
        s = b2MinFloat(s1, s2);

        if (s > axis->separation)
        {
            axis->type = b2EPAxisEdgeB;
            axis->index = i;
            axis->separation = s;
            b2Vec2Assign(axis->normal, n);
        }
    }
}

B2_API
void
b2CollideEdgeAndPolygon(
    struct b2Manifold* manifold,
    const struct b2ShapeEdge* edgeA, const b2Transform xfA,
    const struct b2ShapePolygon* polygonB, const b2Transform xfB)
{
    b2Vec2 v;

    b2Transform xf;

    b2Vec2 centroidB;

    b2Vec2ConstRef v1;
    b2Vec2ConstRef v2;

    b2Vec2 edge1;

    b2Vec2 normal1;
    float offset1;

    int oneSided;

    struct b2TempPolygon tempPolygonB;

    int32 i;

    float radius;

    struct b2EPAxis edgeAxis;
    struct b2EPAxis polygonAxis;

    struct b2EPAxis primaryAxis;

    struct b2ClipVertex clipPoints[2];
    struct b2ReferenceFace ref;

    struct b2ClipVertex clipPoints1[2];
    struct b2ClipVertex clipPoints2[2];
    int32 np;

    int32 pointCount;

    // Use hysteresis for jitter reduction.
    const float k_relativeTol = 0.98f;
    const float k_absoluteTol = 0.001f;

    manifold->pointCount = 0;

    b2TransformMulT(xf, xfA, xfB);

    b2TransformMulVec2(centroidB, xf, polygonB->m_centroid);

    v1 = edgeA->m_vertex1;
    v2 = edgeA->m_vertex2;

    b2Vec2Sub(edge1, v2, v1);
    b2Vec2Normalize(edge1, edge1);

    // Normal points to the right for a CCW winding
    b2Vec2Make(normal1, +edge1[1], -edge1[0]);
    b2Vec2Sub(v, centroidB, v1);
    offset1 = b2Vec2DotProduct(normal1, v);

    oneSided = edgeA->m_oneSided;
    if (oneSided && offset1 < 0.0f)
    {
        return;
    }

    // Get polygonB in frameA
    tempPolygonB.count = polygonB->m_count;
    for (i = 0; i < polygonB->m_count; ++i)
    {
        b2TransformMulVec2(tempPolygonB.vertices[i], xf, polygonB->m_vertices[i]);
        b2RotMulVec2(tempPolygonB.normals[i], xf[1], polygonB->m_normals[i]);
    }

    radius = polygonB->m_radius + edgeA->m_radius;

    b2ComputeEdgeSeparation(&edgeAxis, &tempPolygonB, v1, normal1);
    if (edgeAxis.separation > radius)
    {
        return;
    }

    b2ComputePolygonSeparation(&polygonAxis, &tempPolygonB, v1, v2);
    if (polygonAxis.separation > radius)
    {
        return;
    }

    if (polygonAxis.separation - radius > k_relativeTol * (edgeAxis.separation - radius) + k_absoluteTol)
    {
        primaryAxis = polygonAxis;
    }
    else
    {
        primaryAxis = edgeAxis;
    }

    if (oneSided)
    {
        b2Vec2 edge0;
        b2Vec2 normal0;
        int convex1;

        b2Vec2 edge2;
        b2Vec2 normal2;
        int convex2;

        int side1;

        const float sinTol = 0.1f;

        // Smooth collision
        // See https://box2d.org/posts/2020/06/ghost-collisions/

        b2Vec2Sub(edge0, v1, edgeA->m_vertex0);
        b2Vec2Normalize(edge0, edge0);
        b2Vec2Make(normal0, +edge0[1], -edge0[0]);
        convex1 = b2Vec2CrossProduct(edge0, edge1) >= 0.0f;

        b2Vec2Sub(edge2, edgeA->m_vertex3, v2);
        b2Vec2Normalize(edge2, edge2);
        b2Vec2Make(normal2, +edge2[1], -edge2[0]);
        convex2 = b2Vec2CrossProduct(edge1, edge2) >= 0.0f;

        side1 = b2Vec2DotProduct(primaryAxis.normal, edge1) <= 0.0f;

        // Check Gauss Map
        if (side1)
        {
            if (convex1)
            {
                if (b2Vec2CrossProduct(primaryAxis.normal, normal0) > sinTol)
                {
                    // Skip region
                    return;
                }

                // Admit region
            }
            else
            {
                // Snap region
                primaryAxis = edgeAxis;
            }
        }
        else
        {
            if (convex2)
            {
                if (b2Vec2CrossProduct(normal2, primaryAxis.normal) > sinTol)
                {
                    // Skip region
                    return;
                }

                // Admit region
            }
            else
            {
                // Snap region
                primaryAxis = edgeAxis;
            }
        }
    }

    if (primaryAxis.type == b2EPAxisEdgeA)
    {
        int32 i;

        int32 bestIndex;
        float bestValue;

        int32 i1;
        int32 i2;

        manifold->type = b2ManifoldTypeFaceA;

        // Search for the polygon normal that is most anti-parallel to the edge normal.
        bestIndex = 0;
        bestValue = b2Vec2DotProduct(primaryAxis.normal, tempPolygonB.normals[0]);
        for (i = 1; i < tempPolygonB.count; ++i)
        {
            float value = b2Vec2DotProduct(primaryAxis.normal, tempPolygonB.normals[i]);
            if (value < bestValue)
            {
                bestValue = value;
                bestIndex = i;
            }
        }

        i1 = bestIndex;
        i2 = i1 + 1 < tempPolygonB.count ? i1 + 1 : 0;

        b2Vec2Assign(clipPoints[0].v, tempPolygonB.vertices[i1]);
        clipPoints[0].id.cf.indexA = 0;
        clipPoints[0].id.cf.indexB = (uint8)(i1);
        clipPoints[0].id.cf.typeA = b2ContactFeatureTypeFace;
        clipPoints[0].id.cf.typeB = b2ContactFeatureTypeVertex;

        b2Vec2Assign(clipPoints[1].v, tempPolygonB.vertices[i2]);
        clipPoints[1].id.cf.indexA = 0;
        clipPoints[1].id.cf.indexB = (uint8)(i2);
        clipPoints[1].id.cf.typeA = b2ContactFeatureTypeFace;
        clipPoints[1].id.cf.typeB = b2ContactFeatureTypeVertex;

        ref.i1 = 0;
        ref.i2 = 1;
        b2Vec2Assign(ref.v1, v1);
        b2Vec2Assign(ref.v2, v2);
        b2Vec2Assign(ref.normal, primaryAxis.normal);
        b2Vec2Negate(ref.sideNormal1, edge1);
        b2Vec2Assign(ref.sideNormal2, edge1);
    }
    else
    {
        manifold->type = b2ManifoldTypeFaceB;

        b2Vec2Assign(clipPoints[0].v, v2);
        clipPoints[0].id.cf.indexA = 1;
        clipPoints[0].id.cf.indexB = (uint8)(primaryAxis.index);
        clipPoints[0].id.cf.typeA = b2ContactFeatureTypeVertex;
        clipPoints[0].id.cf.typeB = b2ContactFeatureTypeFace;

        b2Vec2Assign(clipPoints[1].v, v1);
        clipPoints[1].id.cf.indexA = 0;
        clipPoints[1].id.cf.indexB = (uint8)(primaryAxis.index);
        clipPoints[1].id.cf.typeA = b2ContactFeatureTypeVertex;
        clipPoints[1].id.cf.typeB = b2ContactFeatureTypeFace;

        ref.i1 = primaryAxis.index;
        ref.i2 = ref.i1 + 1 < tempPolygonB.count ? ref.i1 + 1 : 0;
        b2Vec2Assign(ref.v1, tempPolygonB.vertices[ref.i1]);
        b2Vec2Assign(ref.v2, tempPolygonB.vertices[ref.i2]);
        b2Vec2Assign(ref.normal, tempPolygonB.normals[ref.i1]);

        // CCW winding
        b2Vec2Make(ref.sideNormal1, +ref.normal[1], -ref.normal[0]);
        b2Vec2Negate(ref.sideNormal2, ref.sideNormal1);
    }

    ref.sideOffset1 = b2Vec2DotProduct(ref.sideNormal1, ref.v1);
    ref.sideOffset2 = b2Vec2DotProduct(ref.sideNormal2, ref.v2);

    // Clip incident edge against reference face side planes
    // Clip to side 1
    np = b2ClipSegmentToLine(clipPoints1, clipPoints, ref.sideNormal1, ref.sideOffset1, ref.i1);

    if (np < b2_maxManifoldPoints)
    {
        return;
    }

    // Clip to side 2
    np = b2ClipSegmentToLine(clipPoints2, clipPoints1, ref.sideNormal2, ref.sideOffset2, ref.i2);

    if (np < b2_maxManifoldPoints)
    {
        return;
    }

    // Now clipPoints2 contains the clipped points.
    if (primaryAxis.type == b2EPAxisEdgeA)
    {
        b2Vec2Assign(manifold->localNormal, ref.normal);
        b2Vec2Assign(manifold->localPoint, ref.v1);
    }
    else
    {
        b2Vec2Assign(manifold->localNormal, polygonB->m_normals[ref.i1]);
        b2Vec2Assign(manifold->localPoint, polygonB->m_vertices[ref.i1]);
    }

    pointCount = 0;
    for (i = 0; i < b2_maxManifoldPoints; ++i)
    {
        float separation;

        b2Vec2Sub(v, clipPoints2[i].v, ref.v1);
        separation = b2Vec2DotProduct(ref.normal, v);

        if (separation <= radius)
        {
            struct b2ManifoldPoint* cp;

            cp = manifold->points + pointCount;

            if (primaryAxis.type == b2EPAxisEdgeA)
            {
                b2TransformMulTVec2(cp->localPoint, xf, clipPoints2[i].v);
                cp->id = clipPoints2[i].id;
            }
            else
            {
                b2Vec2Assign(cp->localPoint, clipPoints2[i].v);
                cp->id.cf.typeA = clipPoints2[i].id.cf.typeB;
                cp->id.cf.typeB = clipPoints2[i].id.cf.typeA;
                cp->id.cf.indexA = clipPoints2[i].id.cf.indexB;
                cp->id.cf.indexB = clipPoints2[i].id.cf.indexA;
            }

            ++pointCount;
        }
    }

    manifold->pointCount = pointCount;
}
