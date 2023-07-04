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
#include "mmB2ShapeCircle.h"
#include "mmB2ShapePolygon.h"

/// Compute the collision manifold between two circles.
B2_API
void
b2CollideCircles(
    struct b2Manifold* manifold,
    const struct b2ShapeCircle* circleA, const b2Transform xfA,
    const struct b2ShapeCircle* circleB, const b2Transform xfB)
{
    b2Vec2 pA;
    b2Vec2 pB;

    b2Vec2 d;
    float distSqr;
    float rA, rB;
    float radius;

    manifold->pointCount = 0;

    b2TransformMulVec2(pA, xfA, circleA->m_p);
    b2TransformMulVec2(pB, xfB, circleB->m_p);

    b2Vec2Sub(d, pB, pA);
    distSqr = b2Vec2DotProduct(d, d);
    rA = circleA->m_radius; rB = circleB->m_radius;
    radius = rA + rB;
    if (distSqr > radius * radius)
    {
        return;
    }

    manifold->type = b2ManifoldTypeCircles;
    b2Vec2Assign(manifold->localPoint, circleA->m_p);
    b2Vec2SetZero(manifold->localNormal);
    manifold->pointCount = 1;

    b2Vec2Assign(manifold->points[0].localPoint, circleB->m_p);
    manifold->points[0].id.key = 0;
}

/// Compute the collision manifold between a polygon and a circle.
B2_API
void
b2CollidePolygonAndCircle(
    struct b2Manifold* manifold,
    const struct b2ShapePolygon* polygonA, const b2Transform xfA,
    const struct b2ShapeCircle* circleB, const b2Transform xfB)
{
    b2Vec2 v;
    b2Vec2 t0, t1;

    b2Vec2 c;
    b2Vec2 cLocal;

    int32 normalIndex;
    float separation;
    float radius;
    int32 vertexCount;
    const b2Vec2* vertices;
    const b2Vec2* normals;

    int32 i;

    int32 vertIndex1;
    int32 vertIndex2;
    b2Vec2ConstRef v1;
    b2Vec2ConstRef v2;

    float u1;
    float u2;

    manifold->pointCount = 0;

    // Compute circle position in the frame of the polygon.
    b2TransformMulVec2(c, xfB, circleB->m_p);
    b2TransformMulTVec2(cLocal, xfA, c);

    // Find the min separating edge.
    normalIndex = 0;
    separation = -b2_maxFloat;
    radius = polygonA->m_radius + circleB->m_radius;
    vertexCount = polygonA->m_count;
    vertices = polygonA->m_vertices;
    normals = polygonA->m_normals;

    for (i = 0; i < vertexCount; ++i)
    {
        float s;

        b2Vec2Sub(v, cLocal, vertices[i]);

        s = b2Vec2DotProduct(normals[i], v);

        if (s > radius)
        {
            // Early out.
            return;
        }

        if (s > separation)
        {
            separation = s;
            normalIndex = i;
        }
    }

    // Vertices that subtend the incident face.
    vertIndex1 = normalIndex;
    vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
    v1 = vertices[vertIndex1];
    v2 = vertices[vertIndex2];

    // If the center is inside the polygon ...
    if (separation < b2_epsilon)
    {
        manifold->pointCount = 1;
        manifold->type = b2ManifoldTypeFaceA;
        b2Vec2Assign(manifold->localNormal, normals[normalIndex]);
        b2Vec2Add(v, v1, v2);
        b2Vec2Scale(manifold->localPoint, v, 0.5f);
        b2Vec2Assign(manifold->points[0].localPoint, circleB->m_p);
        manifold->points[0].id.key = 0;
        return;
    }

    // Compute barycentric coordinates
    b2Vec2Sub(t0, cLocal, v1);
    b2Vec2Sub(t1, v2, v1);
    u1 = b2Vec2DotProduct(t0, t1);
    b2Vec2Sub(t0, cLocal, v2);
    b2Vec2Sub(t1, v1, v2);
    u2 = b2Vec2DotProduct(t0, t1);
    if (u1 <= 0.0f)
    {
        if (b2Vec2SquaredDistance(cLocal, v1) > radius * radius)
        {
            return;
        }

        manifold->pointCount = 1;
        manifold->type = b2ManifoldTypeFaceA;
        b2Vec2Sub(manifold->localNormal, cLocal, v1);
        b2Vec2Normalize(manifold->localNormal, manifold->localNormal);
        b2Vec2Assign(manifold->localPoint, v1);
        b2Vec2Assign(manifold->points[0].localPoint, circleB->m_p);
        manifold->points[0].id.key = 0;
    }
    else if (u2 <= 0.0f)
    {
        if (b2Vec2SquaredDistance(cLocal, v2) > radius * radius)
        {
            return;
        }

        manifold->pointCount = 1;
        manifold->type = b2ManifoldTypeFaceA;
        b2Vec2Sub(manifold->localNormal, cLocal, v2);
        b2Vec2Normalize(manifold->localNormal, manifold->localNormal);
        b2Vec2Assign(manifold->localPoint, v2);
        b2Vec2Assign(manifold->points[0].localPoint, circleB->m_p);
        manifold->points[0].id.key = 0;
    }
    else
    {
        b2Vec2 faceCenter;
        float s;

        b2Vec2Add(v, v1, v2);
        b2Vec2Scale(faceCenter, v, 0.5f);
        b2Vec2Sub(v, cLocal, faceCenter);
        s = b2Vec2DotProduct(v, normals[vertIndex1]);
        if (s > radius)
        {
            return;
        }

        manifold->pointCount = 1;
        manifold->type = b2ManifoldTypeFaceA;
        b2Vec2Assign(manifold->localNormal, normals[vertIndex1]);
        b2Vec2Assign(manifold->localPoint, faceCenter);
        b2Vec2Assign(manifold->points[0].localPoint, circleB->m_p);
        manifold->points[0].id.key = 0;
    }
}
