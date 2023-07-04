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
#include "mmB2ShapePolygon.h"

#include <assert.h>

// Find the max separation between poly1 and poly2 using edge normals from poly1.
static 
float 
b2FindMaxSeparation(
    int32* edgeIndex,
    const struct b2ShapePolygon* poly1, const b2Transform xf1,
    const struct b2ShapePolygon* poly2, const b2Transform xf2)
{
    int32 i, j;
    b2Vec2 v;

    int32 count1;
    int32 count2;
    const b2Vec2* n1s;
    const b2Vec2* v1s;
    const b2Vec2* v2s;
    b2Transform xf;

    int32 bestIndex;
    float maxSeparation;

    count1 = poly1->m_count;
    count2 = poly2->m_count;
    n1s = poly1->m_normals;
    v1s = poly1->m_vertices;
    v2s = poly2->m_vertices;
    b2TransformMulT(xf, xf2, xf1);

    bestIndex = 0;
    maxSeparation = -b2_maxFloat;
    for (i = 0; i < count1; ++i)
    {
        b2Vec2 n;
        b2Vec2 v1;

        float si;

        // Get poly1 normal in frame2.
        b2RotMulVec2(n, xf[1], n1s[i]);
        b2TransformMulVec2(v1, xf, v1s[i]);

        // Find deepest point for normal i.
        si = b2_maxFloat;
        for (j = 0; j < count2; ++j)
        {
            float sij;
            b2Vec2Sub(v, v2s[j], v1);
            sij = b2Vec2DotProduct(n, v);
            if (sij < si)
            {
                si = sij;
            }
        }

        if (si > maxSeparation)
        {
            maxSeparation = si;
            bestIndex = i;
        }
    }

    *edgeIndex = bestIndex;
    return maxSeparation;
}

static 
void 
b2FindIncidentEdge(
    struct b2ClipVertex c[2],
    const struct b2ShapePolygon* poly1, const b2Transform xf1, int32 edge1,
    const struct b2ShapePolygon* poly2, const b2Transform xf2)
{
    int32 i;
    b2Vec2 v;

    const b2Vec2* normals1;

    int32 count2;
    const b2Vec2* vertices2;
    const b2Vec2* normals2;

    b2Vec2 normal1;

    int32 index;
    float minDot;

    int32 i1;
    int32 i2;

    normals1 = poly1->m_normals;

    count2 = poly2->m_count;
    vertices2 = poly2->m_vertices;
    normals2 = poly2->m_normals;

    b2Assert(0 <= edge1 && edge1 < poly1->m_count);

    // Get the normal of the reference edge in poly2's frame.
    b2RotMulVec2(v, xf1[1], normals1[edge1]);
    b2RotMulTVec2(normal1, xf2[1], v);

    // Find the incident edge on poly2.
    index = 0;
    minDot = b2_maxFloat;
    for (i = 0; i < count2; ++i)
    {
        float dot = b2Vec2DotProduct(normal1, normals2[i]);
        if (dot < minDot)
        {
            minDot = dot;
            index = i;
        }
    }

    // Build the clip vertices for the incident edge.
    i1 = index;
    i2 = i1 + 1 < count2 ? i1 + 1 : 0;

    b2TransformMulVec2(c[0].v, xf2, vertices2[i1]);
    c[0].id.cf.indexA = (uint8)edge1;
    c[0].id.cf.indexB = (uint8)i1;
    c[0].id.cf.typeA = b2ContactFeatureTypeFace;
    c[0].id.cf.typeB = b2ContactFeatureTypeVertex;

    b2TransformMulVec2(c[1].v, xf2, vertices2[i2]);
    c[1].id.cf.indexA = (uint8)edge1;
    c[1].id.cf.indexB = (uint8)i2;
    c[1].id.cf.typeA = b2ContactFeatureTypeFace;
    c[1].id.cf.typeB = b2ContactFeatureTypeVertex;
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
B2_API
void
b2CollidePolygons(
    struct b2Manifold* manifold,
    const struct b2ShapePolygon* polygonA, const b2Transform xfA,
    const struct b2ShapePolygon* polygonB, const b2Transform xfB)
{
    int32 i;
    b2Vec2 v;

    float totalRadius;

    int32 edgeA;
    float separationA;

    int32 edgeB;
    float separationB;

    struct b2ClipVertex incidentEdge[2];

    int32 count1;
    const b2Vec2* vertices1;

    int32 iv1;
    int32 iv2;

    b2Vec2 v11;
    b2Vec2 v12;

    b2Vec2 localTangent;

    b2Vec2 localNormal;
    b2Vec2 planePoint;

    b2Vec2 tangent;
    b2Vec2 normal;

    float frontOffset;

    float sideOffset1;
    float sideOffset2;

    struct b2ClipVertex clipPoints1[2];
    struct b2ClipVertex clipPoints2[2];
    int np;

    int32 pointCount;

    const struct b2ShapePolygon* poly1;	// reference polygon
    const struct b2ShapePolygon* poly2;	// incident polygon
    b2Transform xf1, xf2;
    int32 edge1;					// reference edge
    uint8 flip;
    const float k_tol = 0.1f * b2_linearSlop;

    manifold->pointCount = 0;
    totalRadius = polygonA->m_radius + polygonB->m_radius;

    edgeA = 0;
    separationA = b2FindMaxSeparation(&edgeA, polygonA, xfA, polygonB, xfB);
    if (separationA > totalRadius)
        return;

    edgeB = 0;
    separationB = b2FindMaxSeparation(&edgeB, polygonB, xfB, polygonA, xfA);
    if (separationB > totalRadius)
        return;

    if (separationB > separationA + k_tol)
    {
        poly1 = polygonB;
        poly2 = polygonA;
        b2TransformAssign(xf1, xfB);
        b2TransformAssign(xf2, xfA);
        edge1 = edgeB;
        manifold->type = b2ManifoldTypeFaceB;
        flip = 1;
    }
    else
    {
        poly1 = polygonA;
        poly2 = polygonB;
        b2TransformAssign(xf1, xfA);
        b2TransformAssign(xf2, xfB);
        edge1 = edgeA;
        manifold->type = b2ManifoldTypeFaceA;
        flip = 0;
    }

    b2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

    count1 = poly1->m_count;
    vertices1 = poly1->m_vertices;

    iv1 = edge1;
    iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

    b2Vec2Assign(v11, vertices1[iv1]);
    b2Vec2Assign(v12, vertices1[iv2]);

    b2Vec2Sub(localTangent, v12, v11);
    b2Vec2Normalize(localTangent, localTangent);

    b2Vec2CrossProductKR(localNormal, localTangent, 1.0f);
    b2Vec2Add(v, v11, v12);
    b2Vec2Scale(planePoint, v, 0.5f);

    b2RotMulVec2(tangent, xf1[1], localTangent);
    b2Vec2CrossProductKR(normal, tangent, 1.0f);

    b2TransformMulVec2(v11, xf1, v11);
    b2TransformMulVec2(v12, xf1, v12);

    // Face offset.
    frontOffset = b2Vec2DotProduct(normal, v11);

    // Side offsets, extended by polytope skin thickness.
    sideOffset1 = -b2Vec2DotProduct(tangent, v11) + totalRadius;
    sideOffset2 = +b2Vec2DotProduct(tangent, v12) + totalRadius;

    // Clip incident edge against extruded edge1 side edges.

    // Clip to box side 1
    b2Vec2Negate(v, tangent);
    np = b2ClipSegmentToLine(clipPoints1, incidentEdge, v, sideOffset1, iv1);

    if (np < 2)
        return;

    // Clip to negative box side 1
    np = b2ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);

    if (np < 2)
    {
        return;
    }

    // Now clipPoints2 contains the clipped points.
    b2Vec2Assign(manifold->localNormal, localNormal);
    b2Vec2Assign(manifold->localPoint, planePoint);

    pointCount = 0;
    for (i = 0; i < b2_maxManifoldPoints; ++i)
    {
        float separation = b2Vec2DotProduct(normal, clipPoints2[i].v) - frontOffset;

        if (separation <= totalRadius)
        {
            struct b2ManifoldPoint* cp;
            cp = manifold->points + pointCount;
            b2TransformMulTVec2(cp->localPoint, xf2, clipPoints2[i].v);
            cp->id = clipPoints2[i].id;
            if (flip)
            {
                struct b2ContactFeature cf;
                // Swap features
                cf = cp->id.cf;
                cp->id.cf.indexA = cf.indexB;
                cp->id.cf.indexB = cf.indexA;
                cp->id.cf.typeA = cf.typeB;
                cp->id.cf.typeB = cf.typeA;
            }
            ++pointCount;
        }
    }

    manifold->pointCount = pointCount;
}
