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

#ifndef __mmB2World_h__
#define __mmB2World_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2BlockAllocator.h"
#include "b2/mmB2StackAllocator.h"
#include "b2/mmB2ContactManager.h"
#include "b2/mmB2TimeStep.h"
#include "b2/mmB2Draw.h"

#include "b2/mmB2Prefix.h"

struct b2BodyDef;
struct b2QueryCallback;
struct b2RayCastCallback;

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
struct b2World
{
    struct b2BlockAllocator m_blockAllocator;
    struct b2StackAllocator m_stackAllocator;

    struct b2ContactManager m_contactManager;

    struct b2Body* m_bodyList;
    struct b2Joint* m_jointList;

    int32 m_bodyCount;
    int32 m_jointCount;

    b2Vec2 m_gravity;
    int m_allowSleep;

    struct b2DestructionListener* m_destructionListener;
    struct b2Draw* m_debugDraw;

    // This is used to compute the time step ratio to
    // support a variable time step.
    float m_inv_dt0;

    int m_newContacts;
    int m_locked;
    int m_clearForces;

    // These are for debugging the solver.
    int m_warmStarting;
    int m_continuousPhysics;
    int m_subStepping;

    int m_stepComplete;

    struct b2Profile m_profile;
};

B2_API
void
b2WorldInit(
    struct b2World* p);

B2_API
void
b2WorldDestroy(
    struct b2World* p);

/// Register a destruction listener. The listener is owned by you and must
/// remain in scope.
static
inline
void 
b2WorldSetDestructionListener(
    struct b2World* p,
    struct b2DestructionListener* listener)
{
    p->m_destructionListener = listener;
}

/// Register a contact filter to provide specific control over collision.
/// Otherwise the default filter is used (b2_defaultFilter). The listener is
/// owned by you and must remain in scope.
static
inline
void
b2WorldSetContactFilter(
    struct b2World* p,
    struct b2ContactFilter* filter)
{
    p->m_contactManager.m_contactFilter = filter;
}

/// Register a contact event listener. The listener is owned by you and must
/// remain in scope.
static
inline
void
b2WorldSetContactListener(
    struct b2World* p,
    struct b2ContactListener* listener)
{
    p->m_contactManager.m_contactListener = listener;
}

/// Register a routine for debug drawing. The debug draw functions are called
/// inside with b2World::DebugDraw method. The debug draw object is owned
/// by you and must remain in scope.
static
inline
void
b2WorldSetDebugDraw(
    struct b2World* p, 
    struct b2Draw* debugDraw)
{
    p->m_debugDraw = debugDraw;
}

/// Create a rigid body given a definition. No reference to the definition
/// is retained.
/// @warning This function is locked during callbacks.
B2_API
struct b2Body*
b2WorldCreateBody(
    struct b2World* p,
    const struct b2BodyDef* def);

/// Delete a rigid body given a definition. No reference to the definition
/// is retained. This function is locked during callbacks.
/// @warning This automatically deletes all associated shapes and joints.
/// @warning This function is locked during callbacks.
B2_API
void
b2WorldDeleteBody(
    struct b2World* p,
    struct b2Body* b);

/// Create a joint to constrain bodies together. No reference to the definition
/// is retained. This may cause the connected bodies to cease colliding.
/// @warning This function is locked during callbacks.
B2_API
struct b2Joint*
b2WorldCreateJoint(
    struct b2World* p,
    const void* jointDef);

/// Delete a joint. This may cause the connected bodies to begin colliding.
/// @warning This function is locked during callbacks.
B2_API
void
b2WorldDeleteJoint(
    struct b2World* p,
    struct b2Joint* j);

/// Take a time step. This performs collision detection, integration,
/// and constraint solution.
/// @param timeStep the amount of time to simulate, this should not vary.
/// @param velocityIterations for the velocity constraint solver.
/// @param positionIterations for the position constraint solver.
B2_API
void
b2WorldStep(
    struct b2World* p,
    float timeStep,
    int32 velocityIterations,
    int32 positionIterations);

/// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
/// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
/// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
/// a fixed sized time step under a variable frame-rate.
/// When you perform sub-stepping you will disable auto clearing of forces and instead call
/// ClearForces after all sub-steps are complete in one pass of your game loop.
/// @see SetAutoClearForces
B2_API
void
b2WorldClearForces(
    struct b2World* p);

/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
B2_API
void
b2WorldDebugDraw(
    struct b2World* p);

/// Query the world for all fixtures that potentially overlap the
/// provided AABB.
/// @param callback a user implemented callback class.
/// @param aabb the query box.
B2_API
void
b2WorldQueryAABB(
    const struct b2World* p,
    struct b2QueryCallback* callback, 
    const struct b2AABB* aabb);

/// Ray-cast the world for all fixtures in the path of the ray. Your callback
/// controls whether you get the closest point, any point, or n-points.
/// The ray-cast ignores shapes that contain the starting point.
/// @param callback a user implemented callback class.
/// @param point1 the ray starting point
/// @param point2 the ray ending point
B2_API
void
b2WorldRayCast(
    const struct b2World* p, 
    struct b2RayCastCallback* callback, 
    const b2Vec2 point1, 
    const b2Vec2 point2);

/// Get the world body list. With the returned body, use b2Body::GetNext to get
/// the next body in the world list. A nullptr body indicates the end of the list.
/// @return the head of the world body list.
static
inline
struct b2Body* 
b2WorldGetBodyListRef(
    struct b2World* p)
{
    return p->m_bodyList;
}

static
inline
const struct b2Body* 
b2WorldGetBodyList(
    const struct b2World* p)
{
    return p->m_bodyList;
}

/// Get the world joint list. With the returned joint, use b2Joint::GetNext to get
/// the next joint in the world list. A nullptr joint indicates the end of the list.
/// @return the head of the world joint list.
static
inline
struct b2Joint*
b2WorldGetJointListRef(
    struct b2World* p)
{
    return p->m_jointList;
}

static
inline
const struct b2Joint*
b2WorldGetJointList(
    const struct b2World* p)
{
    return p->m_jointList;
}

/// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
/// the next contact in the world list. A nullptr contact indicates the end of the list.
/// @return the head of the world contact list.
/// @warning contacts are created and destroyed in the middle of a time step.
/// Use b2ContactListener to avoid missing contacts.
static
inline
struct b2Contact*
b2WorldGetContactListRef(
    struct b2World* p)
{
    return p->m_contactManager.m_contactList;
}

static
inline
const struct b2Contact*
b2WorldGetContactList(
    const struct b2World* p)
{
    return p->m_contactManager.m_contactList;
}

/// Enable/disable sleep.
B2_API
void
b2WorldSetAllowSleeping(
    struct b2World* p,
    int flag);

static
inline
int
b2WorldGetAllowSleeping(
    const struct b2World* p)
{
    return p->m_allowSleep;
}

/// Enable/disable warm starting. For testing.
static
inline
void
b2WorldSetWarmStarting(
    struct b2World* p,
    int flag) 
{
    p->m_warmStarting = flag;
}

static
inline
int
b2WorldGetWarmStarting(
    const struct b2World* p)
{
    return p->m_warmStarting;
}

/// Enable/disable continuous physics. For testing.
static
inline
void
b2WorldSetContinuousPhysics(
    struct b2World* p,
    int flag)
{
    p->m_continuousPhysics = flag;
}

static
inline
int
b2WorldGetContinuousPhysics(
    const struct b2World* p)
{
    return p->m_continuousPhysics;
}

/// Enable/disable single stepped continuous physics. For testing.
static
inline
void
b2WorldSetSubStepping(
    struct b2World* p,
    int flag)
{
    p->m_subStepping = flag;
}

static
inline
int
b2WorldGetSubStepping(
    const struct b2World* p)
{
    return p->m_subStepping;
}

/// Get the number of broad-phase proxies.
B2_API
int32
b2WorldGetProxyCount(
    const struct b2World* p);

/// Get the number of bodies.
static
inline
int32 
b2WorldGetBodyCount(
    const struct b2World* p)
{
    return p->m_bodyCount;
}

/// Get the number of joints.
static
inline
int32 
b2WorldGetJointCount(
    const struct b2World* p)
{
    return p->m_jointCount;
}

/// Get the number of contacts (each may have 0 or more contact points).
B2_API
int32
b2WorldGetContactCount(
    const struct b2World* p);

/// Get the height of the dynamic tree.
B2_API
int32
b2WorldGetTreeHeight(
    const struct b2World* p);

/// Get the balance of the dynamic tree.
B2_API
int32
b2WorldGetTreeBalance(
    const struct b2World* p);

/// Get the quality metric of the dynamic tree. The smaller the better.
/// The minimum is 1.
B2_API
float
b2WorldGetTreeQuality(
    const struct b2World* p);

/// Change the global gravity vector.
B2_API
void
b2WorldSetGravity(
    struct b2World* p,
    const b2Vec2 gravity);

/// Get the global gravity vector.
static
inline
b2Vec2ConstRef 
b2WorldGetGravity(
    const struct b2World* p)
{
    return p->m_gravity;
}

/// Is the world locked (in the middle of a time step).
static
inline
int 
b2WorldIsLocked(
    const struct b2World* p)
{
    return p->m_locked;
}

/// Set flag to control automatic clearing of forces after each time step.
static
inline
void 
b2WorldSetAutoClearForces(
    struct b2World* p,
    int flag)
{
    p->m_clearForces = flag;
}

/// Get the flag that controls automatic clearing of forces after each time step.
static
inline
int
b2WorldGetAutoClearForces(
    const struct b2World* p)
{
    return p->m_clearForces;
}

/// Shift the world origin. Useful for large worlds.
/// The body shift formula is: position -= newOrigin
/// @param newOrigin the new origin with respect to the old origin
B2_API
void
b2WorldShiftOrigin(
    struct b2World* p,
    const b2Vec2 newOrigin);

/// Get the contact manager for testing.
static
inline
const struct b2ContactManager*
b2WorldGetContactManager(
    const struct b2World* p)
{
    return &p->m_contactManager;
}

/// Get the current profile.
static
inline
const struct b2Profile*
b2WorldGetProfile(
    const struct b2World* p)
{
    return &p->m_profile;
}

/// Dump the world into the log file.
/// @warning this should be called outside of a time step.
B2_API
void
b2WorldDump(
    struct b2World* p);


B2_API
void
b2WorldSolve(
    struct b2World* p,
    const struct b2TimeStep* step);

B2_API
void
b2WorldSolveTOI(
    struct b2World* p,
    const struct b2TimeStep* step);

B2_API
void
b2WorldDrawShape(
    const struct b2World* p,
    struct b2Fixture* fixture,
    const b2Transform xf,
    const b2Color color);

#include "b2/mmB2Suffix.h"

#endif//__mmB2World_h__
