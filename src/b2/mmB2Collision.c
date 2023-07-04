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
#include "mmB2Distance.h"

#include <string.h>
#include <limits.h>
#include <assert.h>

B2_API const uint8 b2_nullFeature = UCHAR_MAX;

B2_API
void
b2WorldManifoldInitialize(
    struct b2WorldManifold* p,
    const struct b2Manifold* manifold,
    const b2Transform xfA, float radiusA,
    const b2Transform xfB, float radiusB)
{
    if (manifold->pointCount == 0)
    {
        return;
    }

    switch (manifold->type)
    {
    case b2ManifoldTypeCircles:
    {
        b2Vec2 pointA;
        b2Vec2 pointB;
        b2Vec2 cA;
        b2Vec2 cB;
        b2Vec2 v;

        b2Vec2Make(p->normal, 1.0f, 0.0f);

        b2TransformMulVec2(pointA, xfA, manifold->localPoint);
        b2TransformMulVec2(pointB, xfB, manifold->points[0].localPoint);
        if (b2Vec2SquaredDistance(pointA, pointB) > b2_epsilon * b2_epsilon)
        {
            b2Vec2Sub(p->normal, pointB, pointA);
            b2Vec2Normalize(p->normal, p->normal);
        }

        b2Vec2Scale(v, p->normal, radiusA);
        b2Vec2Add(cA, pointA, v);

        b2Vec2Scale(v, p->normal, radiusB);
        b2Vec2Sub(cB, pointB, v);

        b2Vec2Add(v, cA, cB);
        b2Vec2Scale(p->points[0], v, 0.5f);

        b2Vec2Sub(v, cB, cA);
        p->separations[0] = b2Vec2DotProduct(v, p->normal);
    }
    break;

    case b2ManifoldTypeFaceA:
    {
        int32 i;
        b2Vec2 planePoint;
        b2RotMulVec2(p->normal, xfA[1], manifold->localNormal);
        b2TransformMulVec2(planePoint, xfA, manifold->localPoint);

        for (i = 0; i < manifold->pointCount; ++i)
        {
            b2Vec2 clipPoint;
            b2Vec2 cA;
            b2Vec2 cB;
            b2Vec2 v;

            b2TransformMulVec2(clipPoint, xfB, manifold->points[i].localPoint);
            b2Vec2Sub(v, clipPoint, planePoint);
            b2Vec2Scale(v, p->normal, radiusA - b2Vec2DotProduct(v, p->normal));
            b2Vec2Add(cA, clipPoint, v);

            b2Vec2Scale(v, p->normal, radiusB);
            b2Vec2Sub(cB, clipPoint, v);

            b2Vec2Add(v, cA, cB);
            b2Vec2Scale(p->points[i], v, 0.5f);

            b2Vec2Sub(v, cB, cA);
            p->separations[i] = b2Vec2DotProduct(v, p->normal);
        }
    }
    break;

    case b2ManifoldTypeFaceB:
    {
        int32 i;
        b2Vec2 planePoint;
        b2RotMulVec2(p->normal, xfB[1], manifold->localNormal);
        b2TransformMulVec2(planePoint, xfB, manifold->localPoint);

        for (i = 0; i < manifold->pointCount; ++i)
        {
            b2Vec2 clipPoint;
            b2Vec2 cB;
            b2Vec2 cA;
            b2Vec2 v;

            b2TransformMulVec2(clipPoint, xfA, manifold->points[i].localPoint);

            b2Vec2Sub(v, clipPoint, planePoint);
            b2Vec2Scale(v, p->normal, radiusB - b2Vec2DotProduct(v, p->normal));
            b2Vec2Add(cB, clipPoint, v);
            
            b2Vec2Scale(v, p->normal, radiusA);
            b2Vec2Sub(cA, clipPoint, v);
            
            b2Vec2Add(v, cA, cB);
            b2Vec2Scale(p->points[i], v, 0.5f);

            b2Vec2Add(v, cA, cB);
            p->separations[i] = b2Vec2DotProduct(v, p->normal);
        }

        // Ensure normal points from A to B.
        b2Vec2Negate(p->normal, p->normal);
    }
    break;
    }
}

B2_API
void
b2GetPointStates(
    enum b2PointState state1[b2_maxManifoldPoints],
    enum b2PointState state2[b2_maxManifoldPoints],
    const struct b2Manifold* manifold1,
    const struct b2Manifold* manifold2)
{
    int32 i;
    int32 j;

    for (i = 0; i < b2_maxManifoldPoints; ++i)
    {
        state1[i] = b2StateNull;
        state2[i] = b2StateNull;
    }

    // Detect persists and removes.
    for (i = 0; i < manifold1->pointCount; ++i)
    {
        union b2ContactID id = manifold1->points[i].id;

        state1[i] = b2StateRemove;

        for (j = 0; j < manifold2->pointCount; ++j)
        {
            if (manifold2->points[j].id.key == id.key)
            {
                state1[i] = b2StatePersist;
                break;
            }
        }
    }

    // Detect persists and adds.
    for (i = 0; i < manifold2->pointCount; ++i)
    {
        union b2ContactID id = manifold2->points[i].id;

        state2[i] = b2StateAdd;

        for (j = 0; j < manifold1->pointCount; ++j)
        {
            if (manifold1->points[j].id.key == id.key)
            {
                state2[i] = b2StatePersist;
                break;
            }
        }
    }
}

/// Verify that the bounds are sorted.
B2_API
int
b2AABBIsValid(
    const struct b2AABB* p)
{
    int valid;
    b2Vec2 d;
    b2Vec2Sub(d, p->upperBound, p->lowerBound);

    valid = (d[0] >= 0.0f && d[1] >= 0.0f);
    valid = valid && b2Vec2IsValid(p->lowerBound) && b2Vec2IsValid(p->upperBound);
    return valid;
}

/// Get the center of the AABB.
B2_API
void
b2AABBGetCenter(
    const struct b2AABB* p,
    b2Vec2 v)
{
    b2Vec2Add(v, p->lowerBound, p->upperBound);
    b2Vec2Scale(v, v, 0.5f);
}

/// Get the extents of the AABB (half-widths).
B2_API
void
b2AABBGetExtents(
    const struct b2AABB* p,
    b2Vec2 v)
{
    b2Vec2Sub(v, p->upperBound, p->lowerBound);
    b2Vec2Scale(v, v, 0.5f);
}

/// Get the perimeter length
B2_API
float
b2AABBGetPerimeter(
    const struct b2AABB* p)
{
    float wx = p->upperBound[0] - p->lowerBound[0];
    float wy = p->upperBound[1] - p->lowerBound[1];
    return 2.0f * (wx + wy);
}

/// Combine an AABB into this one.
B2_API
void
b2AABBCombine(
    struct b2AABB* p,
    const struct b2AABB* aabb)
{
    b2Vec2Min(p->lowerBound, p->lowerBound, aabb->lowerBound);
    b2Vec2Max(p->upperBound, p->upperBound, aabb->upperBound);
}

/// Combine two AABBs into this one.
B2_API
void
b2AABBCombine2(
    struct b2AABB* p,
    const struct b2AABB* aabb1,
    const struct b2AABB* aabb2)
{
    b2Vec2Min(p->lowerBound, aabb1->lowerBound, aabb2->lowerBound);
    b2Vec2Max(p->upperBound, aabb1->upperBound, aabb2->upperBound);
}

/// Does this aabb contain the provided AABB.
B2_API
int
b2AABBContains(
    const struct b2AABB* p,
    const struct b2AABB* aabb)
{
    int result = b2True;
    result = result && p->lowerBound[0] <= aabb->lowerBound[0];
    result = result && p->lowerBound[1] <= aabb->lowerBound[1];
    result = result && aabb->upperBound[0] <= p->upperBound[0];
    result = result && aabb->upperBound[1] <= p->upperBound[1];
    return result;
}

// From Real-time Collision Detection, p179.
B2_API
int
b2AABBRayCast(
    const struct b2AABB* p,
    struct b2RayCastOutput* output,
    const struct b2RayCastInput* input)
{
    b2Vec2 a;
    b2Vec2 d;
    b2Vec2 absD;

    b2Vec2 normal;

    int32 i;

    float tmin = -b2_maxFloat;
    float tmax = +b2_maxFloat;

    b2Vec2Assign(a, input->p1);
    b2Vec2Sub(d, input->p2, input->p1);
    b2Vec2Abs(absD, d);

    for (i = 0; i < 2; ++i)
    {
        if (absD[i] < b2_epsilon)
        {
            // Parallel.
            if (a[i] < p->lowerBound[i] || p->upperBound[i] < a[i])
            {
                return b2False;
            }
        }
        else
        {
            float inv_d = 1.0f / d[i];
            float t1 = (p->lowerBound[i] - a[i]) * inv_d;
            float t2 = (p->upperBound[i] - a[i]) * inv_d;

            // Sign of the normal vector.
            float s = -1.0f;

            if (t1 > t2)
            {
                b2SwapFloat(t1, t2);
                s = 1.0f;
            }

            // Push the min up
            if (t1 > tmin)
            {
                b2Vec2SetZero(normal);
                normal[i] = s;
                tmin = t1;
            }

            // Pull the max down
            tmax = b2MinFloat(tmax, t2);

            if (tmin > tmax)
            {
                return b2False;
            }
        }
    }

    // Does the ray start inside the box?
    // Does the ray intersect beyond the max fraction?
    if (tmin < 0.0f || input->maxFraction < tmin)
    {
        return b2False;
    }

    // Intersection.
    output->fraction = tmin;
    b2Vec2Assign(output->normal, normal);
    return b2True;
}

B2_API
int
b2AABBTestOverlap(
    const struct b2AABB* a,
    const struct b2AABB* b)
{
    b2Vec2 d1, d2;
    b2Vec2Sub(d1, b->lowerBound, a->upperBound);
    b2Vec2Sub(d2, a->lowerBound, b->upperBound);

    if (d1[0] > 0.0f || d1[1] > 0.0f)
        return b2False;

    if (d2[0] > 0.0f || d2[1] > 0.0f)
        return b2False;

    return b2True;
}

// Sutherland-Hodgman clipping.
B2_API
int32 
b2ClipSegmentToLine(
    struct b2ClipVertex vOut[2], 
    const struct b2ClipVertex vIn[2],
    const b2Vec2 normal, 
    float offset, 
    int32 vertexIndexA)
{
    // Start with no output points
    int32 count = 0;

    // Calculate the distance of end points to the line
    float distance0 = b2Vec2DotProduct(normal, vIn[0].v) - offset;
    float distance1 = b2Vec2DotProduct(normal, vIn[1].v) - offset;

    // If the points are behind the plane
    if (distance0 <= 0.0f) vOut[count++] = vIn[0];
    if (distance1 <= 0.0f) vOut[count++] = vIn[1];

    // If the points are on different sides of the plane
    if (distance0 * distance1 < 0.0f)
    {
        b2Vec2 v;
        // Find intersection point of edge and plane
        float interp = distance0 / (distance0 - distance1);

        b2Vec2Sub(v, vIn[1].v, vIn[0].v);
        b2Vec2Scale(v, v, interp);
        b2Vec2Add(vOut[count].v, vIn[0].v, v);

        // VertexA is hitting edgeB.
        vOut[count].id.cf.indexA = (uint8)(vertexIndexA);
        vOut[count].id.cf.indexB = vIn[0].id.cf.indexB;
        vOut[count].id.cf.typeA = b2ContactFeatureTypeVertex;
        vOut[count].id.cf.typeB = b2ContactFeatureTypeFace;
        ++count;

        b2Assert(count == 2);
    }

    return count;
}

/// Determine if two generic shapes overlap.
B2_API
int 
b2TestOverlap(
    const struct b2Shape* shapeA, int32 indexA,
    const struct b2Shape* shapeB, int32 indexB,
    const b2Transform xfA, const b2Transform xfB)
{
    struct b2DistanceInput input;
    struct b2SimplexCache cache;
    struct b2DistanceOutput output;

    b2DistanceInputReset(&input);

    b2DistanceProxySetShape(&input.proxyA, shapeA, indexA);
    b2DistanceProxySetShape(&input.proxyB, shapeB, indexB);
    b2TransformAssign(input.transformA, xfA);
    b2TransformAssign(input.transformB, xfB);
    input.useRadii = b2True;

    cache.count = 0;

    b2Distance(&output, &cache, &input);

    return output.distance < 10.0f * b2_epsilon;
}

// quickhull recursion
static 
void 
b2RecurseHull(
    struct b2Hull* hull,
    b2Vec2 p1, 
    b2Vec2 p2, 
    b2Vec2* ps, 
    int32 count)
{
    b2Vec2 e;
    b2Vec2 v;

    // discard points left of e and find point furthest to the right of e
    b2Vec2 rightPoints[b2_maxPolygonVertices];
    int32 rightCount;

    int32 bestIndex;
    float bestDistance;

    int32 i;

    b2Vec2 bestPoint;

    struct b2Hull hull1;
    struct b2Hull hull2;

    hull->count = 0;

    if (count == 0)
    {
        return;
    }

    // create an edge vector pointing from p1 to p2
    b2Vec2Sub(e, p2, p1);
    b2Vec2Normalize(e, e);

    // discard points left of e and find point furthest to the right of e
    memset(rightPoints, 0, sizeof(rightPoints));
    rightCount = 0;

    bestIndex = 0;
    b2Vec2Sub(v, ps[bestIndex], p1);
    bestDistance = b2Vec2CrossProduct(v, e);
    if (bestDistance > 0.0f)
    {
        b2Vec2Assign(rightPoints[rightCount++], ps[bestIndex]);
    }

    for (i = 1; i < count; ++i)
    {
        float distance;
        b2Vec2Sub(v, ps[i], p1);
        distance = b2Vec2CrossProduct(v, e);
        if (distance > bestDistance)
        {
            bestIndex = i;
            bestDistance = distance;
        }

        if (distance > 0.0f)
        {
            b2Vec2Assign(rightPoints[rightCount++], ps[i]);
        }
    }

    if (bestDistance < 2.0f * b2_linearSlop)
    {
        return;
    }

    b2Vec2Assign(bestPoint, ps[bestIndex]);

    // compute hull to the right of p1-bestPoint
    b2RecurseHull(&hull1, p1, bestPoint, rightPoints, rightCount);

    // compute hull to the right of bestPoint-p2
    b2RecurseHull(&hull2, bestPoint, p2, rightPoints, rightCount);

    // stich together hulls
    for (i = 0; i < hull1.count; ++i)
    {
        b2Vec2Assign(hull->points[hull->count++], hull1.points[i]);
    }

    b2Vec2Assign(hull->points[hull->count++], bestPoint);

    for (i = 0; i < hull2.count; ++i)
    {
        b2Vec2Assign(hull->points[hull->count++], hull2.points[i]);
    }

    b2Assert(hull->count < b2_maxPolygonVertices);
}

// quickhull algorithm
// - merges vertices based on b2_linearSlop
// - removes collinear points using b2_linearSlop
// - returns an empty hull if it fails
B2_API
void
b2ComputeHull(
    struct b2Hull* hull,
    const b2Vec2* points,
    int32 count)
{
    b2Vec2 ps[b2_maxPolygonVertices];
    int32 n = 0;
    int32 i, j;
    b2Vec2 vi;
    b2Vec2 vj;

    b2Vec2 c;
    int32 i1;
    float dsq1;
    float dsq;

    b2Vec2 p1;

    int32 i2;
    float dsq2;

    b2Vec2 p2;

    b2Vec2 rightPoints[b2_maxPolygonVertices - 2];
    int32 rightCount;

    b2Vec2 leftPoints[b2_maxPolygonVertices - 2];
    int32 leftCount;

    b2Vec2 e;

    struct b2Hull hull1;
    struct b2Hull hull2;

    int searching;

    struct b2AABB aabb = { {b2_maxFloat, b2_maxFloat}, {-b2_maxFloat, -b2_maxFloat} };
    const float tolSqr = 16.0f * b2_linearSlop * b2_linearSlop;

    hull->count = 0;

    if (count < 3 || count > b2_maxPolygonVertices)
    {
        // check your data
        return;
    }

    count = b2MinInt32(count, b2_maxPolygonVertices);

    // Perform aggressive point welding. First point always remains.
    // Also compute the bounding box for later.

    for (i = 0; i < count; ++i)
    {
        int unique;

        b2Vec2Min(aabb.lowerBound, aabb.lowerBound, points[i]);
        b2Vec2Max(aabb.upperBound, aabb.upperBound, points[i]);

        b2Vec2Assign(vi, points[i]);

        unique = b2True;
        for (j = 0; j < i; ++j)
        {
            b2Vec2Assign(vj, points[j]);

            dsq = b2Vec2SquaredDistance(vi, vj);
            if (dsq < tolSqr)
            {
                unique = b2False;
                break;
            }
        }

        if (unique)
        {
            b2Vec2Assign(ps[n++], vi);
        }
    }

    if (n < 3)
    {
        // all points very close together, check your data and check your scale
        return;
    }

    // Find an extreme point as the first point on the hull
    b2AABBGetCenter(&aabb, c);
    i1 = 0;
    dsq1 = b2Vec2SquaredDistance(c, ps[i1]);
    for (i = 1; i < n; ++i)
    {
        dsq = b2Vec2SquaredDistance(c, ps[i]);
        if (dsq > dsq1)
        {
            i1 = i;
            dsq1 = dsq;
        }
    }

    // remove p1 from working set
    b2Vec2Assign(p1, ps[i1]);
    b2Vec2Assign(ps[i1], ps[n - 1]);
    n = n - 1;

    i2 = 0;
    dsq2 = b2Vec2SquaredDistance(p1, ps[i2]);
    for (i = 1; i < n; ++i)
    {
        dsq = b2Vec2SquaredDistance(p1, ps[i]);
        if (dsq > dsq2)
        {
            i2 = i;
            dsq2 = dsq;
        }
    }

    // remove p2 from working set
    b2Vec2Assign(p2, ps[i2]);
    b2Vec2Assign(ps[i2], ps[n - 1]);
    n = n - 1;

    // split the points into points that are left and right of the line p1-p2.
    rightCount = 0;
    leftCount = 0;

    b2Vec2Sub(e, p2, p1);
    b2Vec2Normalize(e, e);

    for (i = 0; i < n; ++i)
    {
        b2Vec2 v;
        float d;

        b2Vec2Sub(v, ps[i], p1);
        d = b2Vec2CrossProduct(v, e);

        // slop used here to skip points that are very close to the line p1-p2
        if (d >= 2.0f * b2_linearSlop)
        {
            b2Vec2Assign(rightPoints[rightCount++], ps[i]);
        }
        else if (d <= -2.0f * b2_linearSlop)
        {
            b2Vec2Assign(leftPoints[leftCount++], ps[i]);
        }
    }

    // compute hulls on right and left
    b2RecurseHull(&hull1, p1, p2, rightPoints, rightCount);
    b2RecurseHull(&hull2, p2, p1, leftPoints, leftCount);

    if (hull1.count == 0 && hull2.count == 0)
    {
        // all points collinear
        return;
    }

    // stitch hulls together, preserving CCW winding order
    b2Vec2Assign(hull->points[hull->count++], p1);

    for (i = 0; i < hull1.count; ++i)
    {
        b2Vec2Assign(hull->points[hull->count++], hull1.points[i]);
    }

    b2Vec2Assign(hull->points[hull->count++], p2);

    for (i = 0; i < hull2.count; ++i)
    {
        b2Vec2Assign(hull->points[hull->count++], hull2.points[i]);
    }

    b2Assert(hull->count <= b2_maxPolygonVertices);

    // merge collinear
    searching = b2True;
    while (searching && hull->count > 2)
    {
        searching = b2False;

        for (i = 0; i < hull->count; ++i)
        {
            b2Vec2ConstRef p1;
            b2Vec2ConstRef p2;
            b2Vec2ConstRef p3;

            b2Vec2 e;
            b2Vec2 v;

            float distance;

            int32 i1 = i;
            int32 i2 = (i + 1) % hull->count;
            int32 i3 = (i + 2) % hull->count;

            p1 = hull->points[i1];
            p2 = hull->points[i2];
            p3 = hull->points[i3];

            b2Vec2Sub(e, p3, p1);
            b2Vec2Normalize(e, e);

            b2Vec2Sub(v, p2, p1);
            distance = b2Vec2CrossProduct(v, e);
            if (distance <= 2.0f * b2_linearSlop)
            {
                // remove midpoint from hull
                for (j = i2; j < hull->count - 1; ++j)
                {
                    b2Vec2Assign(hull->points[j], hull->points[j + 1]);
                }
                hull->count -= 1;

                // continue searching for collinear points
                searching = b2True;

                break;
            }
        }
    }

    if (hull->count < 3)
    {
        // all points collinear, shouldn't be reached since this was validated above
        hull->count = 0;
    }
}

B2_API
int
b2ValidateHull(
    const struct b2Hull* hull)
{
    int32 i, j;

    if (hull->count < 3 || b2_maxPolygonVertices < hull->count)
    {
        return b2False;
    }

    // test that every point is behind every edge
    for (i = 0; i < hull->count; ++i)
    {
        int32 i1;
        int32 i2;
        b2Vec2ConstRef p;
        b2Vec2 e;

        // create an edge vector
        i1 = i;
        i2 = i < hull->count - 1 ? i1 + 1 : 0;
        p = hull->points[i1];
        b2Vec2Sub(e, hull->points[i2], p);
        b2Vec2Normalize(e, e);

        for (j = 0; j < hull->count; ++j)
        {
            b2Vec2 v;
            float distance;

            // skip points that subtend the current edge
            if (j == i1 || j == i2)
            {
                continue;
            }

            b2Vec2Sub(v, hull->points[j], p);
            distance = b2Vec2CrossProduct(v, e);
            if (distance >= 0.0f)
            {
                return b2False;
            }
        }
    }

    // test for collinear points
    for (i = 0; i < hull->count; ++i)
    {
        int32 i1;
        int32 i2;
        int32 i3;

        b2Vec2ConstRef p1;
        b2Vec2ConstRef p2;
        b2Vec2ConstRef p3;

        b2Vec2 e;

        b2Vec2 v;

        float distance;

        i1 = i;
        i2 = (i + 1) % hull->count;
        i3 = (i + 2) % hull->count;

        p1 = hull->points[i1];
        p2 = hull->points[i2];
        p3 = hull->points[i3];

        b2Vec2Sub(e, p3, p1);
        b2Vec2Normalize(e, e);

        b2Vec2Sub(v, p2, p1);
        distance = b2Vec2CrossProduct(v, e);
        if (distance <= b2_linearSlop)
        {
            // p1-p2-p3 are collinear
            return b2False;
        }
    }

    return b2True;
}
