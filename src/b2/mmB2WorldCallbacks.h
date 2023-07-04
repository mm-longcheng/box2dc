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

#ifndef __mmB2WorldCallbacks_h__
#define __mmB2WorldCallbacks_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Common.h"

#include "b2/mmB2Prefix.h"

struct b2Joint;
struct b2Fixture;
struct b2Manifold;
struct b2Contact;

/// Joints and fixtures are destroyed when their associated
/// body is destroyed. Implement this listener so that you
/// may nullify references to these joints and shapes.
struct b2DestructionListenerMeta
{
    /// Called when any joint is about to be destroyed due
    /// to the destruction of one of its attached bodies.
    /// void(*SayGoodbyeJoint)(
    ///     void* obj,
    ///     struct b2Joint* joint);
    void* SayGoodbyeJoint;

    /// Called when any fixture is about to be destroyed due
    /// to the destruction of its parent body.
    /// void(*SayGoodbyeFixture)(
    ///     void* obj,
    ///     struct b2Fixture* fixture);
    void* SayGoodbyeFixture;
};

struct b2DestructionListener
{
    const struct b2DestructionListenerMeta* m;
    void* o;
};

B2_API
void
b2DestructionListenerSayGoodbyeJoint(
    struct b2DestructionListener* p,
    struct b2Joint* joint);

B2_API
void
b2DestructionListenerSayGoodbyeFixture(
    struct b2DestructionListener* p,
    struct b2Fixture* fixture);

/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
struct b2ContactFilterMeta
{
    /// Return true if contact calculations should be performed between these two shapes.
    /// @warning for performance reasons this is only called when the AABBs begin to overlap.
    /// int(*ShouldCollide)(
    ///     void* obj,
    ///     struct b2Fixture* fixtureA,
    ///     struct b2Fixture* fixtureB);
    void* ShouldCollide;
};

struct b2ContactFilter
{
    const struct b2ContactFilterMeta* m;
    void* o;
};

B2_API extern const struct b2ContactFilterMeta b2ContactFilterMetaDefault;

B2_API
int
b2ContactFilterShouldCollide(
    struct b2ContactFilter* p,
    struct b2Fixture* fixtureA,
    struct b2Fixture* fixtureB);

/// Contact impulses for reporting. Impulses are used instead of forces because
/// sub-step forces may approach infinity for rigid body collisions. These
/// match up one-to-one with the contact points in b2Manifold.
struct b2ContactImpulse
{
    float normalImpulses[b2_maxManifoldPoints];
    float tangentImpulses[b2_maxManifoldPoints];
    int32 count;
};

/// Implement this class to get contact information. You can use these results for
/// things like sounds and game logic. You can also get contact results by
/// traversing the contact lists after the time step. However, you might miss
/// some contacts because continuous physics leads to sub-stepping.
/// Additionally you may receive multiple callbacks for the same contact in a
/// single time step.
/// You should strive to make your callbacks efficient because there may be
/// many callbacks per time step.
/// @warning You cannot create/destroy Box2D entities inside these callbacks.
struct b2ContactListenerMeta
{
    /// Called when two fixtures begin to touch.
    /// void(*BeginContact)(
    ///     void* obj,
    ///     struct b2Contact* contact);
    void* BeginContact;

    /// Called when two fixtures cease to touch.
    /// void(*EndContact)(
    ///     void* obj,
    ///     struct b2Contact* contact);
    void* EndContact;

    /// This is called after a contact is updated. This allows you to inspect a
    /// contact before it goes to the solver. If you are careful, you can modify the
    /// contact manifold (e.g. disable contact).
    /// A copy of the old manifold is provided so that you can detect changes.
    /// Note: this is called only for awake bodies.
    /// Note: this is called even when the number of contact points is zero.
    /// Note: this is not called for sensors.
    /// Note: if you set the number of contact points to zero, you will not
    /// get an EndContact callback. However, you may get a BeginContact callback
    /// the next step.
    /// void(*PreSolve)(
    ///     void* obj,
    ///     struct b2Contact* contact,
    ///     const struct b2Manifold* oldManifold);
    void* PreSolve;

    /// This lets you inspect a contact after the solver is finished. This is useful
    /// for inspecting impulses.
    /// Note: the contact manifold does not include time of impact impulses, which can be
    /// arbitrarily large if the sub-step is small. Hence the impulse is provided explicitly
    /// in a separate data structure.
    /// Note: this is only called for contacts that are touching, solid, and awake.
    /// void(*PostSolve)(
    ///     void* obj,
    ///     struct b2Contact* contact,
    ///     const struct b2ContactImpulse* impulse);
    void* PostSolve;
};

struct b2ContactListener
{
    const struct b2ContactListenerMeta* m;
    void* o;
};

B2_API extern const struct b2ContactListenerMeta b2ContactListenerMetaDefault;

B2_API
void
b2ContactListenerBeginContact(
    struct b2ContactListener* p,
    struct b2Contact* contact);

B2_API
void
b2ContactListenerEndContact(
    struct b2ContactListener* p,
    struct b2Contact* contact);

B2_API
void
b2ContactListenerPreSolve(
    struct b2ContactListener* p,
    struct b2Contact* contact,
    const struct b2Manifold* oldManifold);

B2_API
void
b2ContactListenerPostSolve(
    struct b2ContactListener* p,
    struct b2Contact* contact,
    const struct b2ContactImpulse* impulse);

/// Callback class for AABB queries.
/// See b2World::Query
struct b2QueryCallbackMeta
{
    /// Called for each fixture found in the query AABB.
    /// @return false to terminate the query.
    /// int (*ReportFixture)(
    ///     void* obj,
    ///     struct b2Fixture* fixture);
    void* ReportFixture;
};

struct b2QueryCallback
{
    const struct b2QueryCallbackMeta* m;
    void* o;
};

B2_API
int
b2QueryCallbackReportFixture(
    struct b2QueryCallback* p,
    struct b2Fixture* fixture);

/// Callback class for ray casts.
/// See b2World::RayCast
struct b2RayCastCallbackMeta
{
    /// Called for each fixture found in the query. You control how the ray cast
    /// proceeds by returning a float:
    /// return -1: ignore this fixture and continue
    /// return 0: terminate the ray cast
    /// return fraction: clip the ray to this point
    /// return 1: don't clip the ray and continue
    /// @param fixture the fixture hit by the ray
    /// @param point the point of initial intersection
    /// @param normal the normal vector at the point of intersection
    /// @param fraction the fraction along the ray at the point of intersection
    /// @return -1 to filter, 0 to terminate, fraction to clip the ray for
    /// closest hit, 1 to continue
    /// float (*ReportFixture)(
    ///     void* obj,
    ///     struct b2Fixture* fixture, 
    ///     const b2Vec2 point,
    ///     const b2Vec2 normal, 
    ///     float fraction);
    void* ReportFixture;
};

struct b2RayCastCallback
{
    const struct b2RayCastCallbackMeta* m;
    void* o;
};

B2_API
float
b2RayCastCallbackReportFixture(
    struct b2RayCastCallback* p,
    struct b2Fixture* fixture, 
    const b2Vec2 point,
    const b2Vec2 normal, 
    float fraction);

#include "b2/mmB2Suffix.h"

#endif//__mmB2WorldCallbacks_h__
