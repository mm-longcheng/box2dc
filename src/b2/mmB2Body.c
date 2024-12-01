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

#include "mmB2Body.h"
#include "mmB2Shape.h"
#include "mmB2Common.h"
#include "mmB2Fixture.h"
#include "mmB2Joint.h"
#include "mmB2Contact.h"
#include "mmB2World.h"

#include <assert.h>
#include <stddef.h>

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif

B2_API
void
b2BodyDefReset(
    struct b2BodyDef* p)
{
    b2Vec2Make(p->position, 0.0f, 0.0f);
    p->angle = 0.0f;
    b2Vec2Make(p->linearVelocity, 0.0f, 0.0f);
    p->angularVelocity = 0.0f;
    p->linearDamping = 0.0f;
    p->angularDamping = 0.0f;
    p->allowSleep = b2True;
    p->awake = b2True;
    p->fixedRotation = b2False;
    p->bullet = b2False;
    p->type = b2BodyTypeStatic;
    p->enabled = b2True;
    p->userData = 0;
    p->gravityScale = 1.0f;
}

B2_API
struct b2Fixture*
b2BodyCreateFixtureFromDef(
    struct b2Body* p,
    const struct b2FixtureDef* def)
{
    struct b2BlockAllocator* allocator;
    void* memory;
    struct b2Fixture* fixture;

    b2Assert(b2WorldIsLocked(p->m_world) == b2False);
    if (b2WorldIsLocked(p->m_world) == b2True)
    {
        return NULL;
    }

    allocator = &p->m_world->m_blockAllocator;

    memory = b2BlockAllocatorAllocate(allocator, sizeof(struct b2Fixture));
    fixture = (struct b2Fixture*)(memory);
    b2FixtureInit(fixture);
    b2FixtureCreate(fixture, allocator, p, def);

    if (p->m_flags & b2BodyFlagEnabled)
    {
        struct b2BroadPhase* broadPhase = &p->m_world->m_contactManager.m_broadPhase;
        b2FixtureCreateProxies(fixture, broadPhase, p->m_xf);
    }

    fixture->m_next = p->m_fixtureList;
    p->m_fixtureList = fixture;
    ++p->m_fixtureCount;

    fixture->m_body = p;

    // Adjust mass properties if needed.
    if (fixture->m_density > 0.0f)
    {
        b2BodyResetMassData(p);
    }

    // Let the world know we have a new fixture. This will cause new contacts
    // to be created at the beginning of the next time step.
    p->m_world->m_newContacts = b2True;

    return fixture;
}

B2_API
struct b2Fixture*
b2BodyCreateFixtureFromShape(
    struct b2Body* p,
    const void* shape,
    float density)
{
    struct b2FixtureDef def;

    b2FixtureDefReset(&def);

    def.shape = (const struct b2Shape*)shape;
    def.density = density;

    return b2BodyCreateFixtureFromDef(p, &def);
}

B2_API
void
b2BodyDeleteFixture(
    struct b2Body* p,
    struct b2Fixture* fixture)
{
    struct b2Fixture** node;
    int found;

    float density;

    struct b2ContactEdge* edge;

    struct b2BlockAllocator* allocator;

    if (fixture == NULL)
    {
        return;
    }

    b2Assert(b2WorldIsLocked(p->m_world) == b2False);
    if (b2WorldIsLocked(p->m_world) == b2True)
    {
        return;
    }

    b2Assert(fixture->m_body == p);

    // Remove the fixture from this body's singly linked list.
    b2Assert(p->m_fixtureCount > 0);
    node = &p->m_fixtureList;
    found = b2False;
    while (*node != NULL)
    {
        if (*node == fixture)
        {
            *node = fixture->m_next;
            found = b2True;
            break;
        }

        node = &(*node)->m_next;
    }

    // You tried to remove a shape that is not attached to this body.
    b2Assert(found);

    density = fixture->m_density;

    // Destroy any contacts associated with the fixture.
    edge = p->m_contactList;
    while (edge)
    {
        struct b2Contact* c;
        struct b2Fixture* fixtureA;
        struct b2Fixture* fixtureB;

        c = edge->contact;
        edge = edge->next;

        fixtureA = b2ContactGetFixtureARef(c);
        fixtureB = b2ContactGetFixtureBRef(c);

        if (fixture == fixtureA || fixture == fixtureB)
        {
            // This destroys the contact and removes it from
            // this body's contact list.
            b2ContactManagerDelete(&p->m_world->m_contactManager, c);
        }
    }

    allocator = &p->m_world->m_blockAllocator;

    if (p->m_flags & b2BodyFlagEnabled)
    {
        struct b2BroadPhase* broadPhase = &p->m_world->m_contactManager.m_broadPhase;
        b2FixtureDeleteProxies(fixture, broadPhase);
    }

    fixture->m_body = NULL;
    fixture->m_next = NULL;
    b2FixtureDelete(fixture, allocator);
    b2FixtureDestroy(fixture);
    b2BlockAllocatorFree(allocator, fixture, sizeof(struct b2Fixture));

    --p->m_fixtureCount;

    // Reset the mass data
    if (density > 0.0f)
    {
        b2BodyResetMassData(p);
    }
}

B2_API
void
b2BodySetTransform(
    struct b2Body* p,
    const b2Vec2 position,
    float angle)
{
    struct b2BroadPhase* broadPhase;

    struct b2Fixture* f;

    b2Assert(b2WorldIsLocked(p->m_world) == b2False);
    if (b2WorldIsLocked(p->m_world) == b2True)
    {
        return;
    }

    b2RotFromAngle(p->m_xf[1], angle);
    b2Vec2Assign(p->m_xf[0], position);

    b2TransformMulVec2(p->m_sweep.c, p->m_xf, p->m_sweep.localCenter);
    p->m_sweep.a = angle;

    b2Vec2Assign(p->m_sweep.c0, p->m_sweep.c);
    p->m_sweep.a0 = angle;

    broadPhase = &p->m_world->m_contactManager.m_broadPhase;
    for (f = p->m_fixtureList; f; f = f->m_next)
    {
        b2FixtureSynchronize(f, broadPhase, p->m_xf, p->m_xf);
    }

    // Check for new contacts the next step
    p->m_world->m_newContacts = b2True;
}

B2_API
void
b2BodySetLinearVelocity(
    struct b2Body* p,
    const b2Vec2 v)
{
    if (p->m_type == b2BodyTypeStatic)
    {
        return;
    }

    if (b2Vec2DotProduct(v, v) > 0.0f)
    {
        b2BodySetAwake(p, b2True);
    }

    b2Vec2Assign(p->m_linearVelocity, v);
}

B2_API
void
b2BodySetAngularVelocity(
    struct b2Body* p,
    float omega)
{
    if (p->m_type == b2BodyTypeStatic)
    {
        return;
    }

    if (omega * omega > 0.0f)
    {
        b2BodySetAwake(p, b2True);
    }

    p->m_angularVelocity = omega;
}

B2_API
void
b2BodyApplyForce(
    struct b2Body* p,
    const b2Vec2 force,
    const b2Vec2 point,
    int wake)
{
    if (p->m_type != b2BodyTypeDynamic)
    {
        return;
    }

    if (wake && (p->m_flags & b2BodyFlagAwake) == 0)
    {
        b2BodySetAwake(p, b2True);
    }

    // Don't accumulate a force if the body is sleeping.
    if (p->m_flags & b2BodyFlagAwake)
    {
        b2Vec2 v;
        b2Vec2Add(p->m_force, p->m_force, force);
        b2Vec2Sub(v, point, p->m_sweep.c);
        p->m_torque += b2Vec2CrossProduct(v, force);
    }
}

B2_API
void
b2BodyApplyForceToCenter(
    struct b2Body* p,
    const b2Vec2 force,
    int wake)
{
    if (p->m_type != b2BodyTypeDynamic)
    {
        return;
    }

    if (wake && (p->m_flags & b2BodyFlagAwake) == 0)
    {
        b2BodySetAwake(p, b2True);
    }

    // Don't accumulate a force if the body is sleeping
    if (p->m_flags & b2BodyFlagAwake)
    {
        b2Vec2Add(p->m_force, p->m_force, force);
    }
}

B2_API
void
b2BodyApplyTorque(
    struct b2Body* p,
    float torque,
    int wake)
{
    if (p->m_type != b2BodyTypeDynamic)
    {
        return;
    }

    if (wake && (p->m_flags & b2BodyFlagAwake) == 0)
    {
        b2BodySetAwake(p, b2True);
    }

    // Don't accumulate a force if the body is sleeping
    if (p->m_flags & b2BodyFlagAwake)
    {
        p->m_torque += torque;
    }
}

B2_API
void
b2BodyApplyLinearImpulse(
    struct b2Body* p,
    const b2Vec2 impulse,
    const b2Vec2 point,
    int wake)
{
    if (p->m_type != b2BodyTypeDynamic)
    {
        return;
    }

    if (wake && (p->m_flags & b2BodyFlagAwake) == 0)
    {
        b2BodySetAwake(p, b2True);
    }

    // Don't accumulate velocity if the body is sleeping
    if (p->m_flags & b2BodyFlagAwake)
    {
        b2Vec2 v;
        b2Vec2Scale(v, impulse, p->m_invMass);
        b2Vec2Add(p->m_linearVelocity, p->m_linearVelocity, v);

        b2Vec2Sub(v, point, p->m_sweep.c);
        p->m_angularVelocity += p->m_invI * b2Vec2CrossProduct(v, impulse);
    }
}

B2_API
void
b2BodyApplyLinearImpulseToCenter(
    struct b2Body* p,
    const b2Vec2 impulse,
    int wake)
{
    if (p->m_type != b2BodyTypeDynamic)
    {
        return;
    }

    if (wake && (p->m_flags & b2BodyFlagAwake) == 0)
    {
        b2BodySetAwake(p, b2True);
    }

    // Don't accumulate velocity if the body is sleeping
    if (p->m_flags & b2BodyFlagAwake)
    {
        b2Vec2 v;
        b2Vec2Scale(v, impulse, p->m_invMass);
        b2Vec2Add(p->m_linearVelocity, p->m_linearVelocity, v);
    }
}

B2_API
void
b2BodyApplyAngularImpulse(
    struct b2Body* p,
    float impulse,
    int wake)
{
    if (p->m_type != b2BodyTypeDynamic)
    {
        return;
    }

    if (wake && (p->m_flags & b2BodyFlagAwake) == 0)
    {
        b2BodySetAwake(p, b2True);
    }

    // Don't accumulate velocity if the body is sleeping
    if (p->m_flags & b2BodyFlagAwake)
    {
        p->m_angularVelocity += p->m_invI * impulse;
    }
}

B2_API
float
b2BodyGetInertia(
    const struct b2Body* p)
{
    return p->m_I + p->m_mass * b2Vec2DotProduct(p->m_sweep.localCenter, p->m_sweep.localCenter);
}

B2_API
void
b2BodyGetMassData(
    const struct b2Body* p,
    struct b2MassData* data)
{
    data->mass = p->m_mass;
    data->I = p->m_I + p->m_mass * b2Vec2DotProduct(p->m_sweep.localCenter, p->m_sweep.localCenter);
    b2Vec2Assign(data->center, p->m_sweep.localCenter);
}

B2_API
void
b2BodySetMassData(
    struct b2Body* p,
    const struct b2MassData* data)
{
    b2Vec2 v;

    b2Vec2 oldCenter;

    b2Assert(b2WorldIsLocked(p->m_world) == b2False);
    if (b2WorldIsLocked(p->m_world) == b2True)
    {
        return;
    }

    if (p->m_type != b2BodyTypeDynamic)
    {
        return;
    }

    p->m_invMass = 0.0f;
    p->m_I = 0.0f;
    p->m_invI = 0.0f;

    p->m_mass = data->mass;
    if (p->m_mass <= 0.0f)
    {
        p->m_mass = 1.0f;
    }

    p->m_invMass = 1.0f / p->m_mass;

    if (data->I > 0.0f && (p->m_flags & b2BodyFlagFixedRotation) == 0)
    {
        p->m_I = data->I - p->m_mass * b2Vec2DotProduct(data->center, data->center);
        b2Assert(p->m_I > 0.0f);
        p->m_invI = 1.0f / p->m_I;
    }

    // Move center of mass.
    b2Vec2Assign(oldCenter, p->m_sweep.c);
    b2Vec2Assign(p->m_sweep.localCenter, data->center);
    b2TransformMulVec2(p->m_sweep.c, p->m_xf, p->m_sweep.localCenter);
    b2Vec2Assign(p->m_sweep.c0, p->m_sweep.c);

    // Update center of mass velocity.
    b2Vec2Sub(v, p->m_sweep.c, oldCenter);
    b2Vec2CrossProductKL(v, p->m_angularVelocity, v);
    b2Vec2Add(p->m_linearVelocity, p->m_linearVelocity, v);
}

B2_API
void
b2BodyResetMassData(
    struct b2Body* p)
{
    b2Vec2 v;

    b2Vec2 localCenter;

    struct b2Fixture* f;

    b2Vec2 oldCenter;

    // Compute mass data from shapes. Each shape has its own density.
    p->m_mass = 0.0f;
    p->m_invMass = 0.0f;
    p->m_I = 0.0f;
    p->m_invI = 0.0f;
    b2Vec2SetZero(p->m_sweep.localCenter);

    // Static and kinematic bodies have zero mass.
    if (p->m_type == b2BodyTypeStatic || p->m_type == b2BodyTypeKinematic)
    {
        b2Vec2Assign(p->m_sweep.c0, p->m_xf[0]);
        b2Vec2Assign(p->m_sweep.c, p->m_xf[0]);
        p->m_sweep.a0 = p->m_sweep.a;
        return;
    }

    b2Assert(p->m_type == b2BodyTypeDynamic);

    // Accumulate mass over all fixtures.
    b2Vec2Assign(localCenter, b2Vec2Zero);
    for (f = p->m_fixtureList; f; f = f->m_next)
    {
        struct b2MassData massData;

        if (f->m_density == 0.0f)
        {
            continue;
        }

        b2FixtureGetMassData(f, &massData);
        p->m_mass += massData.mass;
        b2Vec2Scale(v, massData.center, massData.mass);
        b2Vec2Add(localCenter, localCenter, v);
        p->m_I += massData.I;
    }

    // Compute center of mass.
    if (p->m_mass > 0.0f)
    {
        p->m_invMass = 1.0f / p->m_mass;
        b2Vec2Scale(localCenter, localCenter, p->m_invMass);
    }

    if (p->m_I > 0.0f && (p->m_flags & b2BodyFlagFixedRotation) == 0)
    {
        // Center the inertia about the center of mass.
        p->m_I -= p->m_mass * b2Vec2DotProduct(localCenter, localCenter);
        b2Assert(p->m_I > 0.0f);
        p->m_invI = 1.0f / p->m_I;

    }
    else
    {
        p->m_I = 0.0f;
        p->m_invI = 0.0f;
    }

    // Move center of mass.
    b2Vec2Assign(oldCenter, p->m_sweep.c);
    b2Vec2Assign(p->m_sweep.localCenter, localCenter);
    b2TransformMulVec2(v, p->m_xf, p->m_sweep.localCenter);
    b2Vec2Assign(p->m_sweep.c, v);
    b2Vec2Assign(p->m_sweep.c0, v);

    // Update center of mass velocity.
    b2Vec2Sub(v, p->m_sweep.c, oldCenter);
    b2Vec2CrossProductKL(v, p->m_angularVelocity, v);
    b2Vec2Add(p->m_linearVelocity, p->m_linearVelocity, v);
}

B2_API
void
b2BodyGetWorldPoint(
    const struct b2Body* p,
    const b2Vec2 localPoint,
    b2Vec2 worldPoint)
{
    b2TransformMulVec2(worldPoint, p->m_xf, localPoint);
}

B2_API
void
b2BodyGetWorldVector(
    const struct b2Body* p,
    const b2Vec2 localVector,
    b2Vec2 worldVector)
{
    b2RotMulVec2(worldVector, p->m_xf[1], localVector);
}

B2_API
void
b2BodyGetLocalPoint(
    const struct b2Body* p,
    const b2Vec2 worldPoint,
    b2Vec2 localPoint)
{
    b2TransformMulTVec2(localPoint, p->m_xf, worldPoint);
}

B2_API
void
b2BodyGetLocalVector(
    const struct b2Body* p,
    const b2Vec2 worldVector,
    b2Vec2 localVector)
{
    b2RotMulTVec2(localVector, p->m_xf[1], worldVector);
}

B2_API
void
b2BodyGetLinearVelocityFromWorldPoint(
    const struct b2Body* p,
    const b2Vec2 worldPoint,
    b2Vec2 linearVelocity)
{
    b2Vec2 v;
    b2Vec2Sub(v, worldPoint, p->m_sweep.c);
    b2Vec2CrossProductKL(v, p->m_angularVelocity, v);
    b2Vec2Add(linearVelocity, p->m_linearVelocity, v);
}

B2_API
void
b2BodyGetLinearVelocityFromLocalPoint(
    const struct b2Body* p,
    const b2Vec2 localPoint,
    b2Vec2 linearVelocity)
{
    b2Vec2 worldPoint;
    b2BodyGetWorldPoint(p, localPoint, worldPoint);
    b2BodyGetLinearVelocityFromWorldPoint(p, worldPoint, linearVelocity);
}

B2_API
void
b2BodySetType(
    struct b2Body* p,
    enum b2BodyType type)
{
    struct b2ContactEdge* ce;

    struct b2BroadPhase* broadPhase;

    struct b2Fixture* f;

    b2Assert(b2WorldIsLocked(p->m_world) == b2False);
    if (b2WorldIsLocked(p->m_world) == b2True)
    {
        return;
    }

    if (p->m_type == type)
    {
        return;
    }

    p->m_type = type;

    b2BodyResetMassData(p);

    if (p->m_type == b2BodyTypeStatic)
    {
        b2Vec2SetZero(p->m_linearVelocity);
        p->m_angularVelocity = 0.0f;
        p->m_sweep.a0 = p->m_sweep.a;
        b2Vec2Assign(p->m_sweep.c0, p->m_sweep.c);
        p->m_flags &= ~b2BodyFlagAwake;
        b2BodySynchronizeFixtures(p);
    }

    b2BodySetAwake(p, b2True);

    b2Vec2SetZero(p->m_force);
    p->m_torque = 0.0f;

    // Delete the attached contacts.
    ce = p->m_contactList;
    while (ce)
    {
        struct b2ContactEdge* ce0;
        ce0 = ce;
        ce = ce->next;
        b2ContactManagerDelete(&p->m_world->m_contactManager, ce0->contact);
    }
    p->m_contactList = NULL;

    // Touch the proxies so that new contacts will be created (when appropriate)
    broadPhase = &p->m_world->m_contactManager.m_broadPhase;
    for (f = p->m_fixtureList; f; f = f->m_next)
    {
        int32 i;
        int32 proxyCount;

        proxyCount = f->m_proxyCount;
        for (i = 0; i < proxyCount; ++i)
        {
            b2BroadPhaseTouchProxy(broadPhase, f->m_proxies[i].proxyId);
        }
    }
}

B2_API
void
b2BodySetBullet(
    struct b2Body* p,
    int flag)
{
    if (flag)
    {
        p->m_flags |= b2BodyFlagBullet;
    }
    else
    {
        p->m_flags &= ~b2BodyFlagBullet;
    }
}

B2_API
int
b2BodyIsBullet(
    const struct b2Body* p)
{
    return (p->m_flags & b2BodyFlagBullet) == b2BodyFlagBullet;
}

B2_API
void
b2BodySetSleepingAllowed(
    struct b2Body* p,
    int flag)
{
    if (flag)
    {
        p->m_flags |= b2BodyFlagAutoSleep;
    }
    else
    {
        p->m_flags &= ~b2BodyFlagAutoSleep;
        b2BodySetAwake(p, b2True);
    }
}

B2_API
int
b2BodyIsSleepingAllowed(
    const struct b2Body* p)
{
    return (p->m_flags & b2BodyFlagAutoSleep) == b2BodyFlagAutoSleep;
}

B2_API
void
b2BodySetAwake(
    struct b2Body* p,
    int flag)
{
    if (p->m_type == b2BodyTypeStatic)
    {
        return;
    }

    if (flag)
    {
        p->m_flags |= b2BodyFlagAwake;
        p->m_sleepTime = 0.0f;
    }
    else
    {
        p->m_flags &= ~b2BodyFlagAwake;
        p->m_sleepTime = 0.0f;
        b2Vec2SetZero(p->m_linearVelocity);
        p->m_angularVelocity = 0.0f;
        b2Vec2SetZero(p->m_force);
        p->m_torque = 0.0f;
    }
}

B2_API
int
b2BodyIsAwake(
    const struct b2Body* p)
{
    return (p->m_flags & b2BodyFlagAwake) == b2BodyFlagAwake;
}

B2_API
void
b2BodySetEnabled(
    struct b2Body* p,
    int flag)
{
    b2Assert(b2WorldIsLocked(p->m_world) == b2False);

    if (flag == b2BodyIsEnabled(p))
    {
        return;
    }

    if (flag)
    {
        struct b2BroadPhase* broadPhase;

        struct b2Fixture* f;

        p->m_flags |= b2BodyFlagEnabled;

        // Create all proxies.
        broadPhase = &p->m_world->m_contactManager.m_broadPhase;
        for (f = p->m_fixtureList; f; f = f->m_next)
        {
            b2FixtureCreateProxies(f, broadPhase, p->m_xf);
        }

        // Contacts are created at the beginning of the next
        p->m_world->m_newContacts = b2True;
    }
    else
    {
        struct b2BroadPhase* broadPhase;

        struct b2Fixture* f;

        struct b2ContactEdge* ce;

        p->m_flags &= ~b2BodyFlagEnabled;

        // Destroy all proxies.
        broadPhase = &p->m_world->m_contactManager.m_broadPhase;
        for (f = p->m_fixtureList; f; f = f->m_next)
        {
            b2FixtureDeleteProxies(f, broadPhase);
        }

        // Destroy the attached contacts.
        ce = p->m_contactList;
        while (ce)
        {
            struct b2ContactEdge* ce0;
            ce0 = ce;
            ce = ce->next;
            b2ContactManagerDelete(&p->m_world->m_contactManager, ce0->contact);
        }
        p->m_contactList = NULL;
    }
}

B2_API
int
b2BodyIsEnabled(
    const struct b2Body* p)
{
    return (p->m_flags & b2BodyFlagEnabled) == b2BodyFlagEnabled;
}

B2_API
void
b2BodySetFixedRotation(
    struct b2Body* p,
    int flag)
{
    int status = (p->m_flags & b2BodyFlagFixedRotation) == b2BodyFlagFixedRotation;
    if (status == flag)
    {
        return;
    }

    if (flag)
    {
        p->m_flags |= b2BodyFlagFixedRotation;
    }
    else
    {
        p->m_flags &= ~b2BodyFlagFixedRotation;
    }

    p->m_angularVelocity = 0.0f;

    b2BodyResetMassData(p);
}

B2_API
int
b2BodyIsFixedRotation(
    const struct b2Body* p)
{
    return (p->m_flags & b2BodyFlagFixedRotation) == b2BodyFlagFixedRotation;
}

B2_API
void
b2BodyDump(
    const struct b2Body* p)
{
    struct b2Fixture* f;
    int32 bodyIndex = p->m_islandIndex;

    // %.9g is sufficient to save and load the same value using text
    // FLT_DECIMAL_DIG == 9

    b2Dump("{\n");
    b2Dump("  b2BodyDef bd;\n");
    b2Dump("  bd.type = b2BodyType(%d);\n", p->m_type);
    b2Dump("  bd.position.Set(%.9g, %.9g);\n", p->m_xf[0][0], p->m_xf[0][1]);
    b2Dump("  bd.angle = %.9g;\n", p->m_sweep.a);
    b2Dump("  bd.linearVelocity.Set(%.9g, %.9g);\n", p->m_linearVelocity[0], p->m_linearVelocity[1]);
    b2Dump("  bd.angularVelocity = %.9g;\n", p->m_angularVelocity);
    b2Dump("  bd.linearDamping = %.9g;\n", p->m_linearDamping);
    b2Dump("  bd.angularDamping = %.9g;\n", p->m_angularDamping);
    b2Dump("  bd.allowSleep = bool(%d);\n", p->m_flags & b2BodyFlagAutoSleep);
    b2Dump("  bd.awake = bool(%d);\n", p->m_flags & b2BodyFlagAwake);
    b2Dump("  bd.fixedRotation = bool(%d);\n", p->m_flags & b2BodyFlagFixedRotation);
    b2Dump("  bd.bullet = bool(%d);\n", p->m_flags & b2BodyFlagBullet);
    b2Dump("  bd.enabled = bool(%d);\n", p->m_flags & b2BodyFlagEnabled);
    b2Dump("  bd.gravityScale = %.9g;\n", p->m_gravityScale);
    b2Dump("  bodies[%d] = m_world->CreateBody(&bd);\n", p->m_islandIndex);
    b2Dump("\n");
    for (f = p->m_fixtureList; f; f = f->m_next)
    {
        b2Dump("  {\n");
        b2FixtureDump(f, bodyIndex);
        b2Dump("  }\n");
    }
    b2Dump("}\n");
}

B2_API
void
b2BodyPrepare(
    struct b2Body* p,
    const struct b2BodyDef* bd,
    struct b2World* world)
{
    b2Assert(b2Vec2IsValid(bd->position));
    b2Assert(b2Vec2IsValid(bd->linearVelocity));
    b2Assert(b2IsValid(bd->angle));
    b2Assert(b2IsValid(bd->angularVelocity));
    b2Assert(b2IsValid(bd->angularDamping) && bd->angularDamping >= 0.0f);
    b2Assert(b2IsValid(bd->linearDamping) && bd->linearDamping >= 0.0f);

    p->m_flags = 0;

    if (bd->bullet)
    {
        p->m_flags |= b2BodyFlagBullet;
    }
    if (bd->fixedRotation)
    {
        p->m_flags |= b2BodyFlagFixedRotation;
    }
    if (bd->allowSleep)
    {
        p->m_flags |= b2BodyFlagAutoSleep;
    }
    if (bd->awake && bd->type != b2BodyTypeStatic)
    {
        p->m_flags |= b2BodyFlagAwake;
    }
    if (bd->enabled)
    {
        p->m_flags |= b2BodyFlagEnabled;
    }

    p->m_world = world;

    b2Vec2Assign(p->m_xf[0], bd->position);
    b2RotFromAngle(p->m_xf[1], bd->angle);

    b2Vec2SetZero(p->m_sweep.localCenter);
    b2Vec2Assign(p->m_sweep.c0, p->m_xf[0]);
    b2Vec2Assign(p->m_sweep.c, p->m_xf[0]);
    p->m_sweep.a0 = bd->angle;
    p->m_sweep.a = bd->angle;
    p->m_sweep.alpha0 = 0.0f;

    p->m_jointList = NULL;
    p->m_contactList = NULL;
    p->m_prev = NULL;
    p->m_next = NULL;

    b2Vec2Assign(p->m_linearVelocity, bd->linearVelocity);
    p->m_angularVelocity = bd->angularVelocity;

    p->m_linearDamping = bd->linearDamping;
    p->m_angularDamping = bd->angularDamping;
    p->m_gravityScale = bd->gravityScale;

    b2Vec2SetZero(p->m_force);
    p->m_torque = 0.0f;

    p->m_sleepTime = 0.0f;

    p->m_userData = 0;

    p->m_type = bd->type;

    p->m_mass = 0.0f;
    p->m_invMass = 0.0f;

    p->m_I = 0.0f;
    p->m_invI = 0.0f;

    p->m_userData = bd->userData;

    p->m_fixtureList = NULL;
    p->m_fixtureCount = 0;
}

B2_API
void
b2BodyDiscard(
    struct b2Body* p)
{
    // shapes and joints are destroyed in b2World::Destroy
}

B2_API
void
b2BodySynchronizeFixtures(
    struct b2Body* p)
{
    struct b2BroadPhase* broadPhase = &p->m_world->m_contactManager.m_broadPhase;

    if (p->m_flags & b2BodyFlagAwake)
    {
        struct b2Fixture* f;

        b2Vec2 v;
        b2Transform xf1;
        b2RotFromAngle(xf1[1], p->m_sweep.a0);
        b2RotMulVec2(v, xf1[1], p->m_sweep.localCenter);
        b2Vec2Sub(xf1[0], p->m_sweep.c0, v);

        for (f = p->m_fixtureList; f; f = f->m_next)
        {
            b2FixtureSynchronize(f, broadPhase, xf1, p->m_xf);
        }
    }
    else
    {
        struct b2Fixture* f;

        for (f = p->m_fixtureList; f; f = f->m_next)
        {
            b2FixtureSynchronize(f, broadPhase, p->m_xf, p->m_xf);
        }
    }
}

B2_API
void
b2BodySynchronizeTransform(
    struct b2Body* p)
{
    b2Vec2 v;
    b2RotFromAngle(p->m_xf[1], p->m_sweep.a);
    b2RotMulVec2(v, p->m_xf[1], p->m_sweep.localCenter);
    b2Vec2Sub(p->m_xf[0], p->m_sweep.c, v);
}

// This is used to prevent connected bodies from colliding.
// It may lie, depending on the collideConnected flag.
B2_API
int
b2BodyShouldCollide(
    const struct b2Body* p,
    const struct b2Body* other)
{
    struct b2JointEdge* jn;

    // At least one body should be dynamic.
    if (p->m_type != b2BodyTypeDynamic && other->m_type != b2BodyTypeDynamic)
    {
        return b2False;
    }

    // Does a joint prevent collision?
    for (jn = p->m_jointList; jn; jn = jn->next)
    {
        if (jn->other == other)
        {
            if (jn->joint->m_collideConnected == b2False)
            {
                return b2False;
            }
        }
    }

    return b2True;
}

B2_API
void
b2BodyAdvance(
    struct b2Body* p,
    float alpha)
{
    b2Vec2 v;
    // Advance to the new safe time. This doesn't sync the broad-phase.
    b2SweepAdvance(&p->m_sweep, alpha);
    b2Vec2Assign(p->m_sweep.c, p->m_sweep.c0);
    p->m_sweep.a = p->m_sweep.a0;
    b2RotFromAngle(p->m_xf[1], p->m_sweep.a);
    b2RotMulVec2(v, p->m_xf[1], p->m_sweep.localCenter);
    b2Vec2Sub(p->m_xf[0], p->m_sweep.c, v);
}
