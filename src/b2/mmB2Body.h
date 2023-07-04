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

#ifndef __mmB2Body_h__
#define __mmB2Body_h__

#include <stdint.h>

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"

#include "b2/mmB2Prefix.h"

struct b2FixtureDef;
struct b2MassData;

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
enum b2BodyType
{
    b2BodyTypeStatic = 0,
    b2BodyTypeKinematic,
    b2BodyTypeDynamic,
};

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
struct b2BodyDef
{
    /// The body type: static, kinematic, or dynamic.
    /// Note: if a dynamic body would have zero mass, the mass is set to one.
    enum b2BodyType type;

    /// The world position of the body. Avoid creating bodies at the origin
    /// since this can lead to many overlapping shapes.
    b2Vec2 position;

    /// The world angle of the body in radians.
    float angle;

    /// The linear velocity of the body's origin in world co-ordinates.
    b2Vec2 linearVelocity;

    /// The angular velocity of the body.
    float angularVelocity;

    /// Linear damping is use to reduce the linear velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    /// Units are 1/time
    float linearDamping;

    /// Angular damping is use to reduce the angular velocity. The damping parameter
    /// can be larger than 1.0f but the damping effect becomes sensitive to the
    /// time step when the damping parameter is large.
    /// Units are 1/time
    float angularDamping;

    /// Set this flag to false if this body should never fall asleep. Note that
    /// this increases CPU usage.
    int allowSleep;

    /// Is this body initially awake or sleeping?
    int awake;

    /// Should this body be prevented from rotating? Useful for characters.
    int fixedRotation;

    /// Is this a fast moving body that should be prevented from tunneling through
    /// other moving bodies? Note that all bodies are prevented from tunneling through
    /// kinematic and static bodies. This setting is only considered on dynamic bodies.
    /// @warning You should use this flag sparingly since it increases processing time.
    int bullet;

    /// Does this body start out enabled?
    int enabled;

    /// Use this to store application specific body data.
    uintptr_t userData;

    /// Scale the gravity applied to this body.
    float gravityScale;
};

/// This constructor sets the body definition default values.
B2_API
void
b2BodyDefReset(
    struct b2BodyDef* p);

// m_flags
enum
{
	b2BodyFlagIsland        = 0x0001,
	b2BodyFlagAwake         = 0x0002,
	b2BodyFlagAutoSleep     = 0x0004,
	b2BodyFlagBullet        = 0x0008,
	b2BodyFlagFixedRotation = 0x0010,
	b2BodyFlagEnabled       = 0x0020,
	b2BodyFlagToi           = 0x0040,
};

/// A rigid body. These are created via b2World::CreateBody.
struct b2Body
{
    enum b2BodyType m_type;

    uint16 m_flags;

    int32 m_islandIndex;

    b2Transform m_xf;           // the body origin transform
    struct b2Sweep m_sweep;     // the swept motion for CCD

    b2Vec2 m_linearVelocity;
    float m_angularVelocity;

    b2Vec2 m_force;
    float m_torque;

    struct b2World* m_world;
    struct b2Body* m_prev;
    struct b2Body* m_next;

    struct b2Fixture* m_fixtureList;
    int32 m_fixtureCount;

    struct b2JointEdge* m_jointList;
    struct b2ContactEdge* m_contactList;

    float m_mass, m_invMass;

    // Rotational inertia about the center of mass.
    float m_I, m_invI;

    float m_linearDamping;
    float m_angularDamping;
    float m_gravityScale;

    float m_sleepTime;

    uintptr_t m_userData;
};

/// Creates a fixture and attach it to this body. Use this function if you need
/// to set some fixture parameters, like friction. Otherwise you can create the
/// fixture directly from a shape.
/// If the density is non-zero, this function automatically updates the mass of the body.
/// Contacts are not created until the next time step.
/// @param def the fixture definition.
/// @warning This function is locked during callbacks.
B2_API
struct b2Fixture*
b2BodyCreateFixtureFromDef(
    struct b2Body* p,
    const struct b2FixtureDef* def);

/// Creates a fixture from a shape and attach it to this body.
/// This is a convenience function. Use b2FixtureDef if you need to set parameters
/// like friction, restitution, user data, or filtering.
/// If the density is non-zero, this function automatically updates the mass of the body.
/// @param shape the shape to be cloned.
/// @param density the shape density (set to zero for static bodies).
/// @warning This function is locked during callbacks.
B2_API
struct b2Fixture*
b2BodyCreateFixtureFromShape(
    struct b2Body* p,
    const void* shape,
    float density);

/// Delete a fixture. This removes the fixture from the broad-phase and
/// destroys all contacts associated with this fixture. This will
/// automatically adjust the mass of the body if the body is dynamic and the
/// fixture has positive density.
/// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
/// @param fixture the fixture to be removed.
/// @warning This function is locked during callbacks.
B2_API
void
b2BodyDeleteFixture(
    struct b2Body* p,
    struct b2Fixture* fixture);

/// Set the position of the body's origin and rotation.
/// Manipulating a body's transform may cause non-physical behavior.
/// Note: contacts are updated on the next call to b2World::Step.
/// @param position the world position of the body's local origin.
/// @param angle the world rotation in radians.
B2_API
void
b2BodySetTransform(
    struct b2Body* p,
    const b2Vec2 position, 
    float angle);

/// Get the body transform for the body's origin.
/// @return the world transform of the body's origin.
static
inline
b2TransformConstRef 
b2BodyGetTransform(
    const struct b2Body* p)
{
    return p->m_xf;
}

/// Get the world body origin position.
/// @return the world position of the body's origin.
static
inline
b2Vec2ConstRef
b2BodyGetPosition(
    const struct b2Body* p)
{
    return p->m_xf[0];
}

/// Get the angle in radians.
/// @return the current world rotation angle in radians.
static
inline
float 
b2BodyGetAngle(
    const struct b2Body* p)
{
    return p->m_sweep.a;
}

/// Get the world position of the center of mass.
static
inline
b2Vec2ConstRef
b2BodyGetWorldCenter(
    const struct b2Body* p)
{
    return p->m_sweep.c;
}

/// Get the local position of the center of mass.
static
inline
b2Vec2ConstRef
b2BodyGetLocalCenter(
    const struct b2Body* p)
{
    return p->m_sweep.localCenter;
}

/// Set the linear velocity of the center of mass.
/// @param v the new linear velocity of the center of mass.
B2_API
void
b2BodySetLinearVelocity(
    struct b2Body* p,
    const b2Vec2 v);

/// Get the linear velocity of the center of mass.
/// @return the linear velocity of the center of mass.
static
inline
b2Vec2ConstRef
b2BodyGetLinearVelocity(
    const struct b2Body* p)
{
    return p->m_linearVelocity;
}

/// Set the angular velocity.
/// @param omega the new angular velocity in radians/second.
B2_API
void
b2BodySetAngularVelocity(
    struct b2Body* p,
    float omega);

/// Get the angular velocity.
/// @return the angular velocity in radians/second.
static
inline
float 
b2BodyGetAngularVelocity(
    const struct b2Body* p)
{
    return p->m_angularVelocity;
}

/// Apply a force at a world point. If the force is not
/// applied at the center of mass, it will generate a torque and
/// affect the angular velocity. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param point the world position of the point of application.
/// @param wake also wake up the body
B2_API
void
b2BodyApplyForce(
    struct b2Body* p,
    const b2Vec2 force, 
    const b2Vec2 point, 
    int wake);

/// Apply a force to the center of mass. This wakes up the body.
/// @param force the world force vector, usually in Newtons (N).
/// @param wake also wake up the body
B2_API
void
b2BodyApplyForceToCenter(
    struct b2Body* p,
    const b2Vec2 force, 
    int wake);

/// Apply a torque. This affects the angular velocity
/// without affecting the linear velocity of the center of mass.
/// @param torque about the z-axis (out of the screen), usually in N-m.
/// @param wake also wake up the body
B2_API
void
b2BodyApplyTorque(
    struct b2Body* p,
    float torque, 
    int wake);

/// Apply an impulse at a point. This immediately modifies the velocity.
/// It also modifies the angular velocity if the point of application
/// is not at the center of mass. This wakes up the body.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param point the world position of the point of application.
/// @param wake also wake up the body
B2_API
void
b2BodyApplyLinearImpulse(
    struct b2Body* p,
    const b2Vec2 impulse, 
    const b2Vec2 point, 
    int wake);

/// Apply an impulse to the center of mass. This immediately modifies the velocity.
/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
/// @param wake also wake up the body
B2_API
void
b2BodyApplyLinearImpulseToCenter(
    struct b2Body* p,
    const b2Vec2 impulse, 
    int wake);

/// Apply an angular impulse.
/// @param impulse the angular impulse in units of kg*m*m/s
/// @param wake also wake up the body
B2_API
void
b2BodyApplyAngularImpulse(
    struct b2Body* p,
    float impulse, 
    int wake);

/// Get the total mass of the body.
/// @return the mass, usually in kilograms (kg).
static
inline
float 
b2BodyGetMass(
    const struct b2Body* p)
{
    return p->m_mass;
}

/// Get the rotational inertia of the body about the local origin.
/// @return the rotational inertia, usually in kg-m^2.
B2_API
float
b2BodyGetInertia(
    const struct b2Body* p);

/// Get the mass data of the body.
/// @return a struct containing the mass, inertia and center of the body.
B2_API
void
b2BodyGetMassData(
    const struct b2Body* p,
    struct b2MassData* data);

/// Set the mass properties to override the mass properties of the fixtures.
/// Note that this changes the center of mass position.
/// Note that creating or destroying fixtures can also alter the mass.
/// This function has no effect if the body isn't dynamic.
/// @param data the mass properties.
B2_API
void
b2BodySetMassData(
    struct b2Body* p,
    const struct b2MassData* data);

/// This resets the mass properties to the sum of the mass properties of the fixtures.
/// This normally does not need to be called unless you called SetMassData to override
/// the mass and you later want to reset the mass.
B2_API
void
b2BodyResetMassData(
    struct b2Body* p);

/// Get the world coordinates of a point given the local coordinates.
/// @param localPoint a point on the body measured relative the the body's origin.
/// @return the same point expressed in world coordinates.
B2_API
void
b2BodyGetWorldPoint(
    const struct b2Body* p,
    const b2Vec2 localPoint,
    b2Vec2 worldPoint);

/// Get the world coordinates of a vector given the local coordinates.
/// @param localVector a vector fixed in the body.
/// @return the same vector expressed in world coordinates.
B2_API
void
b2BodyGetWorldVector(
    const struct b2Body* p,
    const b2Vec2 localVector,
    b2Vec2 worldVector);

/// Gets a local point relative to the body's origin given a world point.
/// @param worldPoint a point in world coordinates.
/// @return the corresponding local point relative to the body's origin.
B2_API
void
b2BodyGetLocalPoint(
    const struct b2Body* p,
    const b2Vec2 worldPoint,
    b2Vec2 localPoint);

/// Gets a local vector given a world vector.
/// @param worldVector a vector in world coordinates.
/// @return the corresponding local vector.
B2_API
void
b2BodyGetLocalVector(
    const struct b2Body* p,
    const b2Vec2 worldVector,
    b2Vec2 localVector);

/// Get the world linear velocity of a world point attached to this body.
/// @param worldPoint a point in world coordinates.
/// @return the world velocity of a point.
B2_API
void
b2BodyGetLinearVelocityFromWorldPoint(
    const struct b2Body* p,
    const b2Vec2 worldPoint,
    b2Vec2 linearVelocity);

/// Get the world velocity of a local point.
/// @param localPoint a point in local coordinates.
/// @return the world velocity of a point.
B2_API
void
b2BodyGetLinearVelocityFromLocalPoint(
    const struct b2Body* p,
    const b2Vec2 localPoint,
    b2Vec2 linearVelocity);

/// Get the linear damping of the body.
static
inline
float 
b2BodyGetLinearDamping(
    const struct b2Body* p)
{
    return p->m_linearDamping;
}

/// Set the linear damping of the body.
static
inline
void 
b2BodySetLinearDamping(
    struct b2Body* p,
    float linearDamping)
{
    p->m_linearDamping = linearDamping;
}

/// Get the angular damping of the body.
static
inline
float 
b2BodyGetAngularDamping(
    const struct b2Body* p)
{
    return p->m_linearDamping;
}

/// Set the angular damping of the body.
static
inline
void 
b2BodySetAngularDamping(
    struct b2Body* p,
    float angularDamping)
{
    p->m_linearDamping = angularDamping;
}

/// Get the gravity scale of the body.
static
inline
float 
b2BodyGetGravityScale(
    const struct b2Body* p)
{
    return p->m_gravityScale;
}

/// Set the gravity scale of the body.
static
inline
void 
b2BodySetGravityScale(
    struct b2Body* p,
    float scale)
{
    p->m_gravityScale = scale;
}

/// Set the type of this body. This may alter the mass and velocity.
B2_API
void
b2BodySetType(
    struct b2Body* p,
    enum b2BodyType type);

/// Get the type of this body.
static
inline
enum b2BodyType
b2BodyGetType(
    const struct b2Body* p)
{
    return p->m_type;
}

/// Should this body be treated like a bullet for continuous collision detection?
B2_API
void
b2BodySetBullet(
    struct b2Body* p,
    int flag);

/// Is this body treated like a bullet for continuous collision detection?
B2_API
int
b2BodyIsBullet(
    const struct b2Body* p);

/// You can disable sleeping on this body. If you disable sleeping, the
/// body will be woken.
B2_API
void
b2BodySetSleepingAllowed(
    struct b2Body* p,
    int flag);

/// Is this body allowed to sleep
B2_API
int
b2BodyIsSleepingAllowed(
    const struct b2Body* p);

/// Set the sleep state of the body. A sleeping body has very
/// low CPU cost.
/// @param flag set to true to wake the body, false to put it to sleep.
B2_API
void
b2BodySetAwake(
    struct b2Body* p,
    int flag);

/// Get the sleeping state of this body.
/// @return true if the body is awake.
B2_API
int
b2BodyIsAwake(
    const struct b2Body* p);

/// Allow a body to be disabled. A disabled body is not simulated and cannot
/// be collided with or woken up.
/// If you pass a flag of true, all fixtures will be added to the broad-phase.
/// If you pass a flag of false, all fixtures will be removed from the
/// broad-phase and all contacts will be destroyed.
/// Fixtures and joints are otherwise unaffected. You may continue
/// to create/destroy fixtures and joints on disabled bodies.
/// Fixtures on a disabled body are implicitly disabled and will
/// not participate in collisions, ray-casts, or queries.
/// Joints connected to a disabled body are implicitly disabled.
/// An diabled body is still owned by a b2World object and remains
/// in the body list.
B2_API
void
b2BodySetEnabled(
    struct b2Body* p,
    int flag);

/// Get the active state of the body.
B2_API
int
b2BodyIsEnabled(
    const struct b2Body* p);

/// Set this body to have fixed rotation. This causes the mass
/// to be reset.
B2_API
void
b2BodySetFixedRotation(
    struct b2Body* p,
    int flag);

/// Does this body have fixed rotation?
B2_API
int
b2BodyIsFixedRotation(
    const struct b2Body* p);

/// Get the list of all fixtures attached to this body.
static
inline
struct b2Fixture*
b2BodyGetFixtureListRef(
    struct b2Body* p)
{
    return p->m_fixtureList;
}

static
inline
const struct b2Fixture*
b2BodyGetFixtureList(
    const struct b2Body* p)
{
    return p->m_fixtureList;
}

/// Get the list of all joints attached to this body.
static
inline
struct b2JointEdge*
b2BodyGetJointListRef(
    struct b2Body* p)
{
    return p->m_jointList;
}

static
inline
const struct b2JointEdge*
b2BodyGetJointList(
    const struct b2Body* p)
{
    return p->m_jointList;
}

/// Get the list of all contacts attached to this body.
/// @warning this list changes during the time step and you may
/// miss some collisions if you don't use b2ContactListener.
static
inline
struct b2ContactEdge*
b2BodyGetContactListRef(
    struct b2Body* p)
{
    return p->m_contactList;
}

static
inline
const struct b2ContactEdge*
b2BodyGetContactList(
    const struct b2Body* p)
{
    return p->m_contactList;
}

/// Get the next body in the world's body list.
static
inline
struct b2Body*
b2BodyGetNextRef(
    struct b2Body* p)
{
    return p->m_next;
}

static
inline
const struct b2Body*
b2BodyGetNext(
    const struct b2Body* p)
{
    return p->m_next;
}

/// Get the user data pointer that was provided in the body definition.
static
inline
uintptr_t
b2BodyGetUserData(
    const struct b2Body* p)
{
    return p->m_userData;
}

/// Get the parent world of this body.
static
inline
struct b2World*
b2BodyGetWorldRef(
    struct b2Body* p)
{
    return p->m_world;
}

static
inline
const struct b2World*
b2BodyGetWorld(
    const struct b2Body* p)
{
    return p->m_world;
}

/// Dump this body to a file
B2_API
void
b2BodyDump(
    const struct b2Body* p);

B2_API
void
b2BodyPrepare(
    struct b2Body* p,
    const struct b2BodyDef* bd,
    struct b2World* world);

B2_API
void
b2BodyDiscard(
    struct b2Body* p);

B2_API
void
b2BodySynchronizeFixtures(
    struct b2Body* p);

B2_API
void
b2BodySynchronizeTransform(
    struct b2Body* p);

/// This is used to prevent connected bodies from colliding.
/// It may lie, depending on the collideConnected flag.
B2_API
int
b2BodyShouldCollide(
    const struct b2Body* p,
    const struct b2Body* other);

B2_API
void
b2BodyAdvance(
    struct b2Body* p,
    float alpha);

#include "b2/mmB2Suffix.h"

#endif//__mmB2Body_h__
