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

#ifndef __mmB2Collision_h__
#define __mmB2Collision_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Common.h"

#include "b2/mmB2Prefix.h"

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

struct b2Shape;
struct b2ShapeCircle;
struct b2ShapeEdge;
struct b2ShapePolygon;

B2_API extern const uint8 b2_nullFeature;

enum b2ContactFeatureType
{
    b2ContactFeatureTypeVertex = 0,
    b2ContactFeatureTypeFace   = 1,
};
/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
struct b2ContactFeature
{
    uint8 indexA;		///< Feature index on shapeA
    uint8 indexB;		///< Feature index on shapeB
    uint8 typeA;		///< The feature type on shapeA
    uint8 typeB;		///< The feature type on shapeB
};

/// Contact ids to facilitate warm starting.
union b2ContactID
{
    struct b2ContactFeature cf;
    uint32 key;					///< Used to quickly compare contact ids.
};

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
struct b2ManifoldPoint
{
    b2Vec2 localPoint;		///< usage depends on manifold type
    float normalImpulse;	///< the non-penetration impulse
    float tangentImpulse;	///< the friction impulse
    union b2ContactID id;			///< uniquely identifies a contact point between two shapes
};

enum b2ManifoldType
{
    b2ManifoldTypeCircles,
    b2ManifoldTypeFaceA,
    b2ManifoldTypeFaceB,
};
/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
struct b2Manifold
{
    struct b2ManifoldPoint points[b2_maxManifoldPoints];	///< the points of contact
    b2Vec2 localNormal;								///< not use for Type::e_points
    b2Vec2 localPoint;								///< usage depends on manifold type
    enum b2ManifoldType type;
    int32 pointCount;								///< the number of manifold points
};

/// This is used to compute the current state of a contact manifold.
struct b2WorldManifold
{
    b2Vec2 normal;								///< world vector pointing from A to B
    b2Vec2 points[b2_maxManifoldPoints];		///< world contact point (point of intersection)
    float separations[b2_maxManifoldPoints];	///< a negative value indicates overlap, in meters
};

/// Evaluate the manifold with supplied transforms. This assumes
/// modest motion from the original state. This does not change the
/// point count, impulses, etc. The radii must come from the shapes
/// that generated the manifold.
B2_API
void
b2WorldManifoldInitialize(
    struct b2WorldManifold* p,
    const struct b2Manifold* manifold,
    const b2Transform xfA, float radiusA,
    const b2Transform xfB, float radiusB);

/// This is used for determining the state of contact points.
enum b2PointState
{
    b2StateNull,		///< point does not exist
    b2StateAdd,		    ///< point was added in the update
    b2StatePersist,	    ///< point persisted across the update
    b2StateRemove,		///< point was removed in the update
};

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
B2_API
void 
b2GetPointStates(
    enum b2PointState state1[b2_maxManifoldPoints], 
    enum b2PointState state2[b2_maxManifoldPoints],
    const struct b2Manifold* manifold1, 
    const struct b2Manifold* manifold2);

/// Used for computing contact manifolds.
struct b2ClipVertex
{
    b2Vec2 v;
    union b2ContactID id;
};

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
struct b2RayCastInput
{
    b2Vec2 p1, p2;
    float maxFraction;
};

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
struct b2RayCastOutput
{
    b2Vec2 normal;
    float fraction;
};

/// An axis aligned bounding box.
struct b2AABB
{
    b2Vec2 lowerBound;	///< the lower vertex
    b2Vec2 upperBound;	///< the upper vertex
};

/// Verify that the bounds are sorted.
B2_API
int
b2AABBIsValid(
    const struct b2AABB* p);

/// Get the center of the AABB.
B2_API
void
b2AABBGetCenter(
    const struct b2AABB* p,
    b2Vec2 v);

/// Get the extents of the AABB (half-widths).
B2_API
void
b2AABBGetExtents(
    const struct b2AABB* p,
    b2Vec2 v);

/// Get the perimeter length
B2_API
float
b2AABBGetPerimeter(
    const struct b2AABB* p);

/// Combine an AABB into this one.
B2_API
void
b2AABBCombine(
    struct b2AABB* p,
    const struct b2AABB* aabb);

/// Combine two AABBs into this one.
B2_API
void
b2AABBCombine2(
    struct b2AABB* p,
    const struct b2AABB* aabb1,
    const struct b2AABB* aabb2);

/// Does this aabb contain the provided AABB.
B2_API
int
b2AABBContains(
    const struct b2AABB* p,
    const struct b2AABB* aabb);

B2_API
int
b2AABBRayCast(
    const struct b2AABB* p, 
    struct b2RayCastOutput* output, 
    const struct b2RayCastInput* input);

B2_API
int
b2AABBTestOverlap(
    const struct b2AABB* a,
    const struct b2AABB* b);

/// Compute the collision manifold between two circles.
B2_API
void 
b2CollideCircles(
    struct b2Manifold* manifold,
    const struct b2ShapeCircle* circleA, const b2Transform xfA,
    const struct b2ShapeCircle* circleB, const b2Transform xfB);

/// Compute the collision manifold between a polygon and a circle.
B2_API
void 
b2CollidePolygonAndCircle(
    struct b2Manifold* manifold,
    const struct b2ShapePolygon* polygonA, const b2Transform xfA,
    const struct b2ShapeCircle* circleB, const b2Transform xfB);

/// Compute the collision manifold between two polygons.
B2_API
void 
b2CollidePolygons(
    struct b2Manifold* manifold,
    const struct b2ShapePolygon* polygonA, const b2Transform xfA,
    const struct b2ShapePolygon* polygonB, const b2Transform xfB);

/// Compute the collision manifold between an edge and a circle.
B2_API
void
b2CollideEdgeAndCircle(
    struct b2Manifold* manifold,
    const struct b2ShapeEdge* polygonA, const b2Transform xfA,
    const struct b2ShapeCircle* circleB, const b2Transform xfB);

/// Compute the collision manifold between an edge and a polygon.
B2_API
void
b2CollideEdgeAndPolygon(
    struct b2Manifold* manifold,
    const struct b2ShapeEdge* edgeA, const b2Transform xfA,
    const struct b2ShapePolygon* polygonB, const b2Transform xfB);

/// Clipping for contact manifolds.
B2_API
int32 
b2ClipSegmentToLine(
    struct b2ClipVertex vOut[2], 
    const struct b2ClipVertex vIn[2],
    const b2Vec2 normal, 
    float offset, 
    int32 vertexIndexA);

/// Determine if two generic shapes overlap.
B2_API
int
b2TestOverlap(
    const struct b2Shape* shapeA, int32 indexA,
    const struct b2Shape* shapeB, int32 indexB,
    const b2Transform xfA, const b2Transform xfB);

/// Convex hull used for polygon collision
struct b2Hull
{
    b2Vec2 points[b2_maxPolygonVertices];
    int32 count;
};

/// Compute the convex hull of a set of points. Returns an empty hull if it fails.
/// Some failure cases:
/// - all points very close together
/// - all points on a line
/// - less than 3 points
/// - more than b2_maxPolygonVertices points
/// This welds close points and removes collinear points.
B2_API
void
b2ComputeHull(
    struct b2Hull* hull,
    const b2Vec2* points, 
    int32 count);

/// This determines if a hull is valid. Checks for:
/// - convexity
/// - collinear points
/// This is expensive and should not be called at runtime.
B2_API
int
b2ValidateHull(
    const struct b2Hull* hull);

#include "b2/mmB2Suffix.h"

#endif//__mmB2Collision_h__
