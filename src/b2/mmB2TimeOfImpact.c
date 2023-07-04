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

#include "mmB2TimeOfImpact.h"
#include "mmB2Common.h"
#include "mmB2Math.h"
#include "mmB2Timer.h"

#include <assert.h>

B2_API float b2_toiTime, b2_toiMaxTime;
B2_API int32 b2_toiCalls, b2_toiIters, b2_toiMaxIters;
B2_API int32 b2_toiRootIters, b2_toiMaxRootIters;

enum b2SeparationFunctionType
{
    b2SeparationFunctionPoints,
    b2SeparationFunctionFaceA,
    b2SeparationFunctionFaceB,
};

struct b2SeparationFunction
{
    const struct b2DistanceProxy* m_proxyA;
    const struct b2DistanceProxy* m_proxyB;
    struct b2Sweep m_sweepA, m_sweepB;
    enum b2SeparationFunctionType m_type;
    b2Vec2 m_localPoint;
    b2Vec2 m_axis;
};

// TODO_ERIN might not need to return the separation

static
float 
b2SeparationFunctionInitialize(
    struct b2SeparationFunction* p,
    const struct b2SimplexCache* cache,
    const struct b2DistanceProxy* proxyA, const struct b2Sweep* sweepA,
    const struct b2DistanceProxy* proxyB, const struct b2Sweep* sweepB,
    float t1)
{
    int32 count;

    b2Transform xfA, xfB;

    p->m_proxyA = proxyA;
    p->m_proxyB = proxyB;
    count = cache->count;
    b2Assert(0 < count && count < 3);

    p->m_sweepA = (*sweepA);
    p->m_sweepB = (*sweepB);

    b2SweepGetTransform(&p->m_sweepA, xfA, t1);
    b2SweepGetTransform(&p->m_sweepB, xfB, t1);

    if (count == 1)
    {
        b2Vec2ConstRef localPointA;
        b2Vec2ConstRef localPointB;
        b2Vec2 pointA;
        b2Vec2 pointB;

        p->m_type = b2SeparationFunctionPoints;
        localPointA = b2DistanceProxyGetVertex(p->m_proxyA, cache->indexA[0]);
        localPointB = b2DistanceProxyGetVertex(p->m_proxyB, cache->indexB[0]);
        b2TransformMulVec2(pointA, xfA, localPointA);
        b2TransformMulVec2(pointB, xfB, localPointB);
        b2Vec2Sub(p->m_axis, pointB, pointA);
        return b2Vec2Normalize(p->m_axis, p->m_axis);
    }
    else if (cache->indexA[0] == cache->indexA[1])
    {
        b2Vec2 v;
        b2Vec2ConstRef localPointB1;
        b2Vec2ConstRef localPointB2;

        b2Vec2 normal;

        b2Vec2 pointB;

        b2Vec2ConstRef localPointA;
        b2Vec2 pointA;

        float s;

        // Two points on B and one on A.
        p->m_type = b2SeparationFunctionFaceB;
        localPointB1 = b2DistanceProxyGetVertex(proxyB, cache->indexB[0]);
        localPointB2 = b2DistanceProxyGetVertex(proxyB, cache->indexB[1]);

        b2Vec2Sub(v, localPointB2, localPointB1);
        b2Vec2CrossProductKR(p->m_axis, v, 1.0f);
        b2Vec2Normalize(p->m_axis, p->m_axis);
        b2RotMulVec2(normal, xfB[1], p->m_axis);

        b2Vec2Add(v, localPointB1, localPointB2);
        b2Vec2Scale(p->m_localPoint, v, 0.5f);
        b2TransformMulVec2(pointB, xfB, p->m_localPoint);

        localPointA = b2DistanceProxyGetVertex(proxyA, cache->indexA[0]);
        b2TransformMulVec2(pointA, xfA, localPointA);

        b2Vec2Sub(v, pointA, pointB);
        s = b2Vec2DotProduct(v, normal);
        if (s < 0.0f)
        {
            b2Vec2Negate(p->m_axis, p->m_axis);
            return -s;
        }
        else
        {
            return +s;
        }
    }
    else
    {
        b2Vec2 v;

        b2Vec2ConstRef localPointA1;
        b2Vec2ConstRef localPointA2;

        b2Vec2 normal;

        b2Vec2 pointA;

        b2Vec2ConstRef localPointB;
        b2Vec2 pointB;

        float s;

        // Two points on A and one or two points on B.
        p->m_type = b2SeparationFunctionFaceA;
        localPointA1 = b2DistanceProxyGetVertex(p->m_proxyA, cache->indexA[0]);
        localPointA2 = b2DistanceProxyGetVertex(p->m_proxyA, cache->indexA[1]);

        b2Vec2Sub(v, localPointA2, localPointA1);
        b2Vec2CrossProductKR(p->m_axis, v, 1.0f);
        b2Vec2Normalize(p->m_axis, p->m_axis);
        b2RotMulVec2(normal, xfA[1], p->m_axis);

        b2Vec2Add(v, localPointA1, localPointA2);
        b2Vec2Scale(p->m_localPoint, v, 0.5f);
        b2TransformMulVec2(pointA, xfA, p->m_localPoint);

        localPointB = b2DistanceProxyGetVertex(p->m_proxyB, cache->indexB[0]);
        b2TransformMulVec2(pointB, xfB, localPointB);

        b2Vec2Sub(v, pointB, pointA);
        s = b2Vec2DotProduct(v, normal);
        if (s < 0.0f)
        {
            b2Vec2Negate(p->m_axis, p->m_axis);
            return -s;
        }
        else
        {
            return +s;
        }
    }
}

static
float 
b2SeparationFunctionFindMinSeparation(
    const struct b2SeparationFunction* p,
    int32* indexA, 
    int32* indexB, 
    float t)
{
    b2Transform xfA, xfB;

    b2SweepGetTransform(&p->m_sweepA, xfA, t);
    b2SweepGetTransform(&p->m_sweepB, xfB, t);

    switch (p->m_type)
    {
    case b2SeparationFunctionPoints:
    {
        b2Vec2 v;

        b2Vec2 axisA;
        b2Vec2 axisB;

        b2Vec2ConstRef localPointA;
        b2Vec2ConstRef localPointB;

        b2Vec2 pointA;
        b2Vec2 pointB;

        float separation;

        b2RotMulTVec2(axisA, xfA[1], p->m_axis);
        b2Vec2Negate(v, p->m_axis);
        b2RotMulTVec2(axisB, xfB[1], v);

        *indexA = b2DistanceProxyGetSupport(p->m_proxyA, axisA);
        *indexB = b2DistanceProxyGetSupport(p->m_proxyB, axisB);

        localPointA = b2DistanceProxyGetVertex(p->m_proxyA, *indexA);
        localPointB = b2DistanceProxyGetVertex(p->m_proxyB, *indexB);

        b2TransformMulVec2(pointA, xfA, localPointA);
        b2TransformMulVec2(pointB, xfB, localPointB);

        b2Vec2Sub(v, pointB, pointA);
        separation = b2Vec2DotProduct(v, p->m_axis);
        return separation;
    }

    case b2SeparationFunctionFaceA:
    {
        b2Vec2 v;

        b2Vec2 normal;
        b2Vec2 pointA;

        b2Vec2 axisB;

        b2Vec2ConstRef localPointB;

        b2Vec2 pointB;

        float separation;

        b2RotMulVec2(normal, xfA[1], p->m_axis);
        b2TransformMulVec2(pointA, xfA, p->m_localPoint);

        b2Vec2Negate(v, normal);
        b2RotMulTVec2(axisB, xfB[1], v);

        *indexA = -1;
        *indexB = b2DistanceProxyGetSupport(p->m_proxyB, axisB);

        localPointB = b2DistanceProxyGetVertex(p->m_proxyB, *indexB);
        b2TransformMulVec2(pointB, xfB, localPointB);

        b2Vec2Sub(v, pointB, pointA);
        separation = b2Vec2DotProduct(v, normal);
        return separation;
    }

    case b2SeparationFunctionFaceB:
    {
        b2Vec2 v;

        b2Vec2 normal;
        b2Vec2 pointB;

        b2Vec2 axisA;

        b2Vec2ConstRef localPointA;
        b2Vec2 pointA;

        float separation;

        b2RotMulVec2(normal, xfB[1], p->m_axis);
        b2TransformMulVec2(pointB, xfB, p->m_localPoint);

        b2Vec2Negate(v, normal);
        b2RotMulTVec2(axisA, xfA[1], v);

        *indexB = -1;
        *indexA = b2DistanceProxyGetSupport(p->m_proxyA, axisA);

        localPointA = b2DistanceProxyGetVertex(p->m_proxyA, *indexA);
        b2TransformMulVec2(pointA, xfA, localPointA);

        b2Vec2Sub(v, pointA, pointB);
        separation = b2Vec2DotProduct(v, normal);
        return separation;
    }

    default:
        b2Assert(b2False);
        *indexA = -1;
        *indexB = -1;
        return 0.0f;
    }
}

static
float 
b2SeparationFunctionEvaluate(
    const struct b2SeparationFunction* p,
    int32 indexA, 
    int32 indexB, 
    float t)
{
    b2Transform xfA, xfB;

    b2SweepGetTransform(&p->m_sweepA, xfA, t);
    b2SweepGetTransform(&p->m_sweepB, xfB, t);

    switch (p->m_type)
    {
    case b2SeparationFunctionPoints:
    {
        b2Vec2 v;

        b2Vec2ConstRef localPointA;
        b2Vec2ConstRef localPointB;

        b2Vec2 pointA;
        b2Vec2 pointB;

        float separation;

        localPointA = b2DistanceProxyGetVertex(p->m_proxyA, indexA);
        localPointB = b2DistanceProxyGetVertex(p->m_proxyB, indexB);

        b2TransformMulVec2(pointA, xfA, localPointA);
        b2TransformMulVec2(pointB, xfB, localPointB);

        b2Vec2Sub(v, pointB, pointA);
        separation = b2Vec2DotProduct(v, p->m_axis);
        return separation;
    }

    case b2SeparationFunctionFaceA:
    {
        b2Vec2 v;

        b2Vec2 normal;
        b2Vec2 pointA;

        b2Vec2ConstRef localPointB;
        b2Vec2 pointB;

        float separation;

        b2RotMulVec2(normal, xfA[1], p->m_axis);
        b2TransformMulVec2(pointA, xfA, p->m_localPoint);

        localPointB = b2DistanceProxyGetVertex(p->m_proxyB, indexB);
        b2TransformMulVec2(pointB, xfB, localPointB);

        b2Vec2Sub(v, pointB, pointA);
        separation = b2Vec2DotProduct(v, normal);
        return separation;
    }

    case b2SeparationFunctionFaceB:
    {
        b2Vec2 v;

        b2Vec2 normal;
        b2Vec2 pointB;

        b2Vec2ConstRef localPointA;
        b2Vec2 pointA;

        float separation;

        b2RotMulVec2(normal, xfB[1], p->m_axis);
        b2TransformMulVec2(pointB, xfB, p->m_localPoint);

        localPointA = b2DistanceProxyGetVertex(p->m_proxyA, indexA);
        b2TransformMulVec2(pointA, xfA, localPointA);

        b2Vec2Sub(v, pointA, pointB);
        separation = b2Vec2DotProduct(v, normal);
        return separation;
    }

    default:
        b2Assert(b2False);
        return 0.0f;
    }
}

B2_API
void
b2TOIInputReset(
    struct b2TOIInput* p)
{
    b2DistanceProxyReset(&p->proxyA);
    b2DistanceProxyReset(&p->proxyB);
}

B2_API
void
b2TimeOfImpact(
    struct b2TOIOutput* output,
    const struct b2TOIInput* input)
{
    const struct b2DistanceProxy* proxyA;
    const struct b2DistanceProxy* proxyB;

    struct b2Sweep sweepA;
    struct b2Sweep sweepB;

    struct b2Timer timer;

    float tMax;

    float totalRadius;
    float target;
    float tolerance;

    float t1;
    int32 iter;

    struct b2SimplexCache cache;
    struct b2DistanceInput distanceInput;

    float time;

    const int32 k_maxIterations = 20;	// TODO_ERIN b2Settings

    b2DistanceInputReset(&distanceInput);

    b2TimerMake(&timer);

    ++b2_toiCalls;

    output->state = b2TOIOutputUnknown;
    output->t = input->tMax;

    proxyA = &input->proxyA;
    proxyB = &input->proxyB;

    sweepA = input->sweepA;
    sweepB = input->sweepB;

    // Large rotations can make the root finder fail, so we normalize the
    // sweep angles.
    b2SweepNormalize(&sweepA);
    b2SweepNormalize(&sweepB);

    tMax = input->tMax;

    totalRadius = proxyA->m_radius + proxyB->m_radius;
    target = b2MaxFloat(b2_linearSlop, totalRadius - 3.0f * b2_linearSlop);
    tolerance = 0.25f * b2_linearSlop;
    b2Assert(target > tolerance);

    t1 = 0.0f;
    iter = 0;

    // Prepare input for distance query.
    cache.count = 0;
    distanceInput.proxyA = input->proxyA;
    distanceInput.proxyB = input->proxyB;
    distanceInput.useRadii = b2False;

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    for (;;)
    {
        b2Transform xfA, xfB;

        struct b2DistanceOutput distanceOutput;

        int done;
        float t2;
        int32 pushBackIter;

        struct b2SeparationFunction fcn;

        b2SweepGetTransform(&sweepA, xfA, t1);
        b2SweepGetTransform(&sweepB, xfB, t1);

        // Get the distance between shapes. We can also use the results
        // to get a separating axis.
        b2TransformAssign(distanceInput.transformA, xfA);
        b2TransformAssign(distanceInput.transformB, xfB);
        b2Distance(&distanceOutput, &cache, &distanceInput);

        // If the shapes are overlapped, we give up on continuous collision.
        if (distanceOutput.distance <= 0.0f)
        {
            // Failure!
            output->state = b2TOIOutputOverlapped;
            output->t = 0.0f;
            break;
        }

        if (distanceOutput.distance < target + tolerance)
        {
            // Victory!
            output->state = b2TOIOutputTouching;
            output->t = t1;
            break;
        }

        // Initialize the separating axis.
        b2SeparationFunctionInitialize(&fcn, &cache, proxyA, &sweepA, proxyB, &sweepB, t1);
#if 0
        // Dump the curve seen by the root finder
        {
            enum
            {
                N = 100,
            };
            float dx = 1.0f / N;
            float xs[N + 1];
            float fs[N + 1];

            float x = 0.0f;

            int32 i;

            for (i = 0; i <= N; ++i)
            {
                float f;
                b2SweepGetTransform(&sweepA, xfA, x);
                b2SweepGetTransform(&sweepB, xfB, x);
                f = b2SeparationFunctionEvaluate(&fcn, xfA, xfB) - target;

                printf("%g %g\n", x, f);

                xs[i] = x;
                fs[i] = f;

                x += dx;
            }
        }
#endif

        // Compute the TOI on the separating axis. We do this by successively
        // resolving the deepest point. This loop is bounded by the number of vertices.
        done = b2False;
        t2 = tMax;
        pushBackIter = 0;
        for (;;)
        {
            // Find the deepest point at t2. Store the witness point indices.
            int32 indexA, indexB;
            float s2;
            float s1;
            
            int32 rootIterCount;
            float a1, a2;

            s2 = b2SeparationFunctionFindMinSeparation(&fcn, &indexA, &indexB, t2);

            // Is the final configuration separated?
            if (s2 > target + tolerance)
            {
                // Victory!
                output->state = b2TOIOutputSeparated;
                output->t = tMax;
                done = b2True;
                break;
            }

            // Has the separation reached tolerance?
            if (s2 > target - tolerance)
            {
                // Advance the sweeps
                t1 = t2;
                break;
            }

            // Compute the initial separation of the witness points.
            s1 = b2SeparationFunctionEvaluate(&fcn, indexA, indexB, t1);

            // Check for initial overlap. This might happen if the root finder
            // runs out of iterations.
            if (s1 < target - tolerance)
            {
                output->state = b2TOIOutputFailed;
                output->t = t1;
                done = b2True;
                break;
            }

            // Check for touching
            if (s1 <= target + tolerance)
            {
                // Victory! t1 should hold the TOI (could be 0.0).
                output->state = b2TOIOutputTouching;
                output->t = t1;
                done = b2True;
                break;
            }

            // Compute 1D root of: f(x) - target = 0
            rootIterCount = 0;
            a1 = t1; a2 = t2;
            for (;;)
            {
                // Use a mix of the secant rule and bisection.
                float t;
                float s;

                if (rootIterCount & 1)
                {
                    // Secant rule to improve convergence.
                    t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
                }
                else
                {
                    // Bisection to guarantee progress.
                    t = 0.5f * (a1 + a2);
                }

                ++rootIterCount;
                ++b2_toiRootIters;

                s = b2SeparationFunctionEvaluate(&fcn, indexA, indexB, t);

                if (b2AbsFloat(s - target) < tolerance)
                {
                    // t2 holds a tentative value for t1
                    t2 = t;
                    break;
                }

                // Ensure we continue to bracket the root.
                if (s > target)
                {
                    a1 = t;
                    s1 = s;
                }
                else
                {
                    a2 = t;
                    s2 = s;
                }

                if (rootIterCount == 50)
                {
                    break;
                }
            }

            b2_toiMaxRootIters = b2MaxInt32(b2_toiMaxRootIters, rootIterCount);

            ++pushBackIter;

            if (pushBackIter == b2_maxPolygonVertices)
            {
                break;
            }
        }

        ++iter;
        ++b2_toiIters;

        if (done)
        {
            break;
        }

        if (iter == k_maxIterations)
        {
            // Root finder got stuck. Semi-victory.
            output->state = b2TOIOutputFailed;
            output->t = t1;
            break;
        }
    }

    b2_toiMaxIters = b2MaxInt32(b2_toiMaxIters, iter);

    time = b2TimerGetMilliseconds(&timer);
    b2_toiMaxTime = b2MaxFloat(b2_toiMaxTime, time);
    b2_toiTime += time;
}

