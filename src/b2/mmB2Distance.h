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

#ifndef __mmB2Distance_h__
#define __mmB2Distance_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"

#include "b2/mmB2Prefix.h"

struct b2Shape;

B2_API extern int32 b2_gjkCalls;
B2_API extern int32 b2_gjkIters;
B2_API extern int32 b2_gjkMaxIters;

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
struct b2DistanceProxy
{
    b2Vec2 m_buffer[2];
    const b2Vec2* m_vertices;
    int32 m_count;
    float m_radius;
};

B2_API
void
b2DistanceProxyReset(
    struct b2DistanceProxy* p);

/// Initialize the proxy using the given shape. The shape
/// must remain in scope while the proxy is in use.
B2_API
void
b2DistanceProxySetShape(
    struct b2DistanceProxy* p,
    const struct b2Shape* shape, 
    int32 index);

/// Initialize the proxy using a vertex cloud and radius. The vertices
/// must remain in scope while the proxy is in use.
B2_API
void
b2DistanceProxySetVertex(
    struct b2DistanceProxy* p,
    const b2Vec2* vertices, 
    int32 count, 
    float radius);

/// Get the supporting vertex index in the given direction.
B2_API
int32
b2DistanceProxyGetSupport(
    const struct b2DistanceProxy* p,
    const b2Vec2 d);

/// Get the supporting vertex in the given direction.
B2_API
b2Vec2ConstRef
b2DistanceProxyGetSupportVertex(
    const struct b2DistanceProxy* p, 
    const b2Vec2 d);

/// Get the vertex count.
static
inline
int32
b2DistanceProxyGetVertexCount(
    const struct b2DistanceProxy* p)
{
    return p->m_count;
}

/// Get a vertex by index. Used by b2Distance.
B2_API
b2Vec2ConstRef
b2DistanceProxyGetVertex(
    const struct b2DistanceProxy* p, 
    int32 index);

/// Used to warm start b2Distance.
/// Set count to zero on first call.
struct b2SimplexCache
{
    float metric;		///< length or area
    uint16 count;
    uint8 indexA[3];	///< vertices on shape A
    uint8 indexB[3];	///< vertices on shape B
};

/// Input for b2Distance.
/// You have to option to use the shape radii
/// in the computation. Even
struct b2DistanceInput
{
    struct b2DistanceProxy proxyA;
    struct b2DistanceProxy proxyB;
    b2Transform transformA;
    b2Transform transformB;
    int useRadii;
};

B2_API
void
b2DistanceInputReset(
    struct b2DistanceInput* p);

/// Output for b2Distance.
struct b2DistanceOutput
{
    b2Vec2 pointA;		///< closest point on shapeA
    b2Vec2 pointB;		///< closest point on shapeB
    float distance;
    int32 iterations;	///< number of GJK iterations used
};

/// Compute the closest points between two shapes. Supports any combination of:
/// b2CircleShape, b2PolygonShape, b2EdgeShape. The simplex cache is input/output.
/// On the first call set b2SimplexCache.count to zero.
B2_API
void 
b2Distance(
    struct b2DistanceOutput* output,
    struct b2SimplexCache* cache,
    const struct b2DistanceInput* input);

/// Input parameters for b2ShapeCast
struct b2ShapeCastInput
{
    struct b2DistanceProxy proxyA;
    struct b2DistanceProxy proxyB;
    b2Transform transformA;
    b2Transform transformB;
    b2Vec2 translationB;
};

B2_API
void
b2ShapeCastInputReset(
    struct b2ShapeCastInput* p);

/// Output results for b2ShapeCast
struct b2ShapeCastOutput
{
    b2Vec2 point;
    b2Vec2 normal;
    float lambda;
    int32 iterations;
};

/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
/// @returns true if hit, false if there is no hit or an initial overlap
B2_API
int
b2ShapeCast(
    struct b2ShapeCastOutput* output, 
    const struct b2ShapeCastInput* input);

#include "b2/mmB2Suffix.h"

#endif//__mmB2Distance_h__
