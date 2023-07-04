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

#ifndef __mmB2Fixture_h__
#define __mmB2Fixture_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2Collision.h"
#include "b2/mmB2Body.h"

#include "b2/mmB2Prefix.h"

struct b2MassData;
struct b2BroadPhase;
struct b2BlockAllocator;

/// This holds contact filtering data.
struct b2Filter
{
    /// The collision category bits. Normally you would just set one bit.
    uint16 categoryBits;

    /// The collision mask bits. This states the categories that this
    /// shape would accept for collision.
    uint16 maskBits;

    /// Collision groups allow a certain group of objects to never collide (negative)
    /// or always collide (positive). Zero means no collision group. Non-zero group
    /// filtering always wins against the mask bits.
    int16 groupIndex;
};

B2_API
void
b2FilterReset(
    struct b2Filter* p);

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
struct b2FixtureDef
{
    /// The shape, this must be set. The shape will be cloned, so you
    /// can create the shape on the stack.
    const struct b2Shape* shape;

    /// Use this to store application specific fixture data.
    uintptr_t userData;

    /// The friction coefficient, usually in the range [0,1].
    float friction;

    /// The restitution (elasticity) usually in the range [0,1].
    float restitution;

    /// Restitution velocity threshold, usually in m/s. Collisions above this
    /// speed have restitution applied (will bounce).
    float restitutionThreshold;

    /// The density, usually in kg/m^2.
    float density;

    /// A sensor shape collects contact information but never generates a collision
    /// response.
    int isSensor;

    /// Contact filtering data.
    struct b2Filter filter;
};

/// The constructor sets the default fixture definition values.
B2_API
void
b2FixtureDefReset(
    struct b2FixtureDef* p);

/// This proxy is used internally to connect fixtures to the broad-phase.
struct b2FixtureProxy
{
    struct b2AABB aabb;
    struct b2Fixture* fixture;
    int32 childIndex;
    int32 proxyId;
};

/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via b2Body::CreateFixture.
/// @warning you cannot reuse fixtures.
struct b2Fixture
{
    float m_density;

    struct b2Fixture* m_next;
    struct b2Body* m_body;

    struct b2Shape* m_shape;

    float m_friction;
    float m_restitution;
    float m_restitutionThreshold;

    struct b2FixtureProxy* m_proxies;
    int32 m_proxyCount;

    struct b2Filter m_filter;

    int m_isSensor;

    uintptr_t m_userData;
};

B2_API
void
b2FixtureInit(
    struct b2Fixture* p);

B2_API
void
b2FixtureDestroy(
    struct b2Fixture* p);

/// Get the type of the child shape. You can use this to down cast to the concrete shape.
/// @return the shape type.
B2_API
enum b2ShapeType
b2FixtureGetType(
    const struct b2Fixture* p);

/// Get the child shape. You can modify the child shape, however you should not change the
/// number of vertices because this will crash some collision caching mechanisms.
/// Manipulating the shape may lead to non-physical behavior.
static
inline
struct b2Shape* 
b2FixtureGetShapeRef(
    struct b2Fixture* p)
{
    return p->m_shape;
}

static
inline
const struct b2Shape* 
b2FixtureGetShape(
    const struct b2Fixture* p)
{
    return p->m_shape;
}

/// Set if this fixture is a sensor.
B2_API
void
b2FixtureSetSensor(
    struct b2Fixture* p,
    int sensor);

/// Is this fixture a sensor (non-solid)?
/// @return the true if the shape is a sensor.
static
inline
int 
b2FixtureIsSensor(
    const struct b2Fixture* p)
{
    return p->m_isSensor;
}

/// Set the contact filtering data. This will not update contacts until the next time
/// step when either parent body is active and awake.
/// This automatically calls Refilter.
B2_API
void
b2FixtureSetFilterData(
    struct b2Fixture* p,
    const struct b2Filter* filter);

/// Get the contact filtering data.
static
inline
const struct b2Filter*
b2FixtureGetFilterData(
    const struct b2Fixture* p)
{
    return &p->m_filter;
}

/// Call this if you want to establish collision that was previously disabled by b2ContactFilter::ShouldCollide.
B2_API
void
b2FixtureRefilter(
    struct b2Fixture* p);

/// Get the parent body of this fixture. This is nullptr if the fixture is not attached.
/// @return the parent body.
static
inline
struct b2Body* 
b2FixtureGetBodyRef(
struct b2Fixture* p)
{
    return p->m_body;
}

static
inline
const struct b2Body*
b2FixtureGetBody(
    const struct b2Fixture* p)
{
    return p->m_body;
}

/// Get the next fixture in the parent body's fixture list.
/// @return the next shape.
static
inline
struct b2Fixture*
b2FixtureGetNextRef(
    struct b2Fixture* p)
{
    return p->m_next;
}

static
inline
const struct b2Fixture*
b2FixtureGetNext(
    const struct b2Fixture* p)
{
    return p->m_next;
}

/// Get the user data that was assigned in the fixture definition. Use this to
/// store your application specific data.
static
inline
uintptr_t
b2FixtureGetUserData(
    const struct b2Fixture* p)
{
    return p->m_userData;
}

/// Test a point for containment in this fixture.
/// @param p a point in world coordinates.
B2_API
int
b2FixtureTestPoint(
    const struct b2Fixture* p,
    const b2Vec2 point);

/// Cast a ray against this shape.
/// @param output the ray-cast results.
/// @param input the ray-cast input parameters.
/// @param childIndex the child shape index (e.g. edge index)
B2_API
int
b2FixtureRayCast(
    const struct b2Fixture* p,
    struct b2RayCastOutput* output, 
    const struct b2RayCastInput* input, 
    int32 childIndex);

/// Get the mass data for this fixture. The mass data is based on the density and
/// the shape. The rotational inertia is about the shape's origin. This operation
/// may be expensive.
B2_API
void
b2FixtureGetMassData(
    const struct b2Fixture* p,
    struct b2MassData* massData);

/// Set the density of this fixture. This will _not_ automatically adjust the mass
/// of the body. You must call b2Body::ResetMassData to update the body's mass.
B2_API
void
b2FixtureSetDensity(
    struct b2Fixture* p,
    float density);

/// Get the density of this fixture.
static
inline
float 
b2FixtureGetDensity(
    const struct b2Fixture* p)
{
    return p->m_density;
}

/// Get the coefficient of friction.
static
inline
float 
b2FixtureGetFriction(
    const struct b2Fixture* p)
{
    return p->m_friction;
}

/// Set the coefficient of friction. This will _not_ change the friction of
/// existing contacts.
static
inline
void 
b2FixtureSetFriction(
    struct b2Fixture* p,
    float friction)
{
    p->m_friction = friction;
}

/// Get the coefficient of restitution.
static
inline
float 
b2FixtureGetRestitution(
    const struct b2Fixture* p)
{
    return p->m_restitution;
}

/// Set the coefficient of restitution. This will _not_ change the restitution of
/// existing contacts.
static
inline
void 
b2FixtureSetRestitution(
    struct b2Fixture* p,
    float restitution)
{
    p->m_restitution = restitution;
}

/// Get the restitution velocity threshold.
static
inline
float 
b2FixtureGetRestitutionThreshold(
    const struct b2Fixture* p)
{
    return p->m_restitutionThreshold;
}

/// Set the restitution threshold. This will _not_ change the restitution threshold of
/// existing contacts.
static
inline
void 
b2FixtureSetRestitutionThreshold(
    struct b2Fixture* p,
    float threshold)
{
    p->m_restitutionThreshold = threshold;
}

/// Get the fixture's AABB. This AABB may be enlarge and/or stale.
/// If you need a more accurate AABB, compute it using the shape and
/// the body transform.
B2_API
const struct b2AABB*
b2FixtureGetAABB(
    const struct b2Fixture* p,
    int32 childIndex);

/// Dump this fixture to the log file.
B2_API
void
b2FixtureDump(
    const struct b2Fixture* p,
    int32 bodyIndex);

// We need separation create/destroy functions from the constructor/destructor because
// the destructor cannot access the allocator (no destructor arguments allowed by C++).
B2_API
void
b2FixtureCreate(
    struct b2Fixture* p,
    struct b2BlockAllocator* allocator, 
    struct b2Body* body, 
    const struct b2FixtureDef* def);

B2_API
void
b2FixtureDelete(
    struct b2Fixture* p,
    struct b2BlockAllocator* allocator);

/// These support body activation/deactivation.
B2_API
void
b2FixtureCreateProxies(
    struct b2Fixture* p,
    struct b2BroadPhase* broadPhase, 
    const b2Transform xf);

B2_API
void
b2FixtureDeleteProxies(
    struct b2Fixture* p,
    struct b2BroadPhase* broadPhase);

B2_API
void
b2FixtureSynchronize(
    struct b2Fixture* p,
    struct b2BroadPhase* broadPhase, 
    const b2Transform transform1,
    const b2Transform transform2);

#include "b2/mmB2Suffix.h"

#endif//__mmB2Fixture_h__
