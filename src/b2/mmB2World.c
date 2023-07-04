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

#include "mmB2World.h"
#include "mmB2Shape.h"
#include "mmB2Body.h"
#include "mmB2Joint.h"
#include "mmB2Fixture.h"
#include "mmB2Contact.h"
#include "mmB2Draw.h"
#include "mmB2Island.h"
#include "mmB2WorldCallbacks.h"
#include "mmB2TimeOfImpact.h"
#include "mmB2Timer.h"

#include "mmB2ShapeCircle.h"
#include "mmB2ShapeEdge.h"
#include "mmB2ShapeChain.h"
#include "mmB2ShapePolygon.h"

#include <assert.h>
#include <string.h>

struct b2WorldQueryWrapper
{
    const struct b2BroadPhase* broadPhase;
    struct b2QueryCallback* callback;
};

static
int 
b2WorldQueryWrapperQueryCallback(
    struct b2WorldQueryWrapper* p,
    int32 proxyId)
{
    struct b2FixtureProxy* proxy;
    proxy = (struct b2FixtureProxy*)b2BroadPhaseGetUserData(p->broadPhase, proxyId);
    return b2QueryCallbackReportFixture(p->callback, proxy->fixture);
}

struct b2WorldRayCastWrapper
{
    const struct b2BroadPhase* broadPhase;
    struct b2RayCastCallback* callback;
};

static
float 
b2WorldRayCastWrapperRayCastCallback(
    struct b2WorldRayCastWrapper* p,
    const struct b2RayCastInput* input, 
    int32 proxyId)
{
    void* userData;
    struct b2FixtureProxy* proxy;
    struct b2Fixture* fixture;
    int32 index;
    struct b2RayCastOutput output;
    int hit;

    userData = b2BroadPhaseGetUserData(p->broadPhase, proxyId);
    proxy = (struct b2FixtureProxy*)userData;
    fixture = proxy->fixture;
    index = proxy->childIndex;
    hit = b2FixtureRayCast(fixture, &output, input, index);

    if (hit)
    {
        b2Vec2 v1, v2;

        float fraction;
        b2Vec2 point;

        fraction = output.fraction;
        b2Vec2Scale(v1, input->p1, 1.0f - fraction);
        b2Vec2Scale(v2, input->p2, fraction);
        b2Vec2Add(point, v1, v2);
        return b2RayCastCallbackReportFixture(p->callback, fixture, point, output.normal, fraction);
    }
    else
    {
        return input->maxFraction;
    }
}

B2_API
void
b2WorldInit(
    struct b2World* p)
{
    b2BlockAllocatorInit(&p->m_blockAllocator);
    b2StackAllocatorInit(&p->m_stackAllocator);
    b2ContactManagerInit(&p->m_contactManager);

    p->m_destructionListener = NULL;
    p->m_debugDraw = NULL;

    p->m_bodyList = NULL;
    p->m_jointList = NULL;

    p->m_bodyCount = 0;
    p->m_jointCount = 0;

    p->m_warmStarting = b2True;
    p->m_continuousPhysics = b2True;
    p->m_subStepping = b2False;

    p->m_stepComplete = b2True;

    p->m_allowSleep = b2True;
    b2Vec2Make(p->m_gravity, 0.0f, 0.0f);

    p->m_newContacts = b2False;
    p->m_locked = b2False;
    p->m_clearForces = b2True;

    p->m_inv_dt0 = 0.0f;

    p->m_contactManager.m_allocator = &p->m_blockAllocator;

    memset(&p->m_profile, 0, sizeof(struct b2Profile));
}

B2_API
void
b2WorldDestroy(
    struct b2World* p)
{
    // Some shapes allocate using b2Alloc.
    struct b2Body* b = p->m_bodyList;
    while (b)
    {
        struct b2Body* bNext = b->m_next;

        struct b2Fixture* f = b->m_fixtureList;
        while (f)
        {
            struct b2Fixture* fNext = f->m_next;
            f->m_proxyCount = 0;
            b2FixtureDelete(f, &p->m_blockAllocator);
            f = fNext;
        }

        b = bNext;
    }

    b2ContactManagerDestroy(&p->m_contactManager);
    b2StackAllocatorDestroy(&p->m_stackAllocator);
    b2BlockAllocatorDestroy(&p->m_blockAllocator);
}

B2_API
struct b2Body*
b2WorldCreateBody(
    struct b2World* p,
    const struct b2BodyDef* def)
{
    void* mem;
    struct b2Body* b;

    b2Assert(b2WorldIsLocked(p) == b2False);
    if (b2WorldIsLocked(p))
    {
        return NULL;
    }

    mem = b2BlockAllocatorAllocate(&p->m_blockAllocator, sizeof(struct b2Body));
    b = (struct b2Body*)(mem);
    b2BodyPrepare(b, def, p);

    // Add to world doubly linked list.
    b->m_prev = NULL;
    b->m_next = p->m_bodyList;
    if (p->m_bodyList)
    {
        p->m_bodyList->m_prev = b;
    }
    p->m_bodyList = b;
    ++p->m_bodyCount;

    return b;
}

B2_API
void
b2WorldDeleteBody(
    struct b2World* p,
    struct b2Body* b)
{
    struct b2JointEdge* je;
    struct b2ContactEdge* ce;
    struct b2Fixture* f;

    b2Assert(p->m_bodyCount > 0);
    b2Assert(b2WorldIsLocked(p) == b2False);
    if (b2WorldIsLocked(p))
    {
        return;
    }

    // Delete the attached joints.
    je = b->m_jointList;
    while (je)
    {
        struct b2JointEdge* je0;
        je0 = je;
        je = je->next;

        if (p->m_destructionListener)
        {
            b2DestructionListenerSayGoodbyeJoint(p->m_destructionListener, je0->joint);
        }

        b2WorldDeleteJoint(p, je0->joint);

        b->m_jointList = je;
    }
    b->m_jointList = NULL;

    // Delete the attached contacts.
    ce = b->m_contactList;
    while (ce)
    {
        struct b2ContactEdge* ce0;
        ce0 = ce;
        ce = ce->next;
        b2ContactManagerDelete(&p->m_contactManager, ce0->contact);
    }
    b->m_contactList = NULL;

    // Delete the attached fixtures. This destroys broad-phase proxies.
    f = b->m_fixtureList;
    while (f)
    {
        struct b2Fixture* f0;
        f0 = f;
        f = f->m_next;

        if (p->m_destructionListener)
        {
            b2DestructionListenerSayGoodbyeFixture(p->m_destructionListener, f0);
        }

        b2FixtureDeleteProxies(f0, &p->m_contactManager.m_broadPhase);
        b2FixtureDelete(f0, &p->m_blockAllocator);
        b2FixtureDestroy(f0);
        b2BlockAllocatorFree(&p->m_blockAllocator, f0, sizeof(struct b2Fixture));

        b->m_fixtureList = f;
        b->m_fixtureCount -= 1;
    }
    b->m_fixtureList = NULL;
    b->m_fixtureCount = 0;

    // Remove world body list.
    if (b->m_prev)
    {
        b->m_prev->m_next = b->m_next;
    }

    if (b->m_next)
    {
        b->m_next->m_prev = b->m_prev;
    }

    if (b == p->m_bodyList)
    {
        p->m_bodyList = b->m_next;
    }

    --p->m_bodyCount;
    b2BodyDiscard(b);
    b2BlockAllocatorFree(&p->m_blockAllocator, b, sizeof(struct b2Body));
}

B2_API
struct b2Joint*
b2WorldCreateJoint(
    struct b2World* p,
    const void* jointDef)
{
    struct b2Joint* j;

    struct b2Body* bodyA;
    struct b2Body* bodyB;

    const struct b2JointDef* def;

    b2Assert(b2WorldIsLocked(p) == b2False);
    if (b2WorldIsLocked(p))
    {
        return NULL;
    }

    def = (const struct b2JointDef*)(jointDef);

    j = b2JointCreate(def, &p->m_blockAllocator);

    // Connect to the world list.
    j->m_prev = NULL;
    j->m_next = p->m_jointList;
    if (p->m_jointList)
    {
        p->m_jointList->m_prev = j;
    }
    p->m_jointList = j;
    ++p->m_jointCount;

    // Connect to the bodies' doubly linked lists.
    j->m_edgeA.joint = j;
    j->m_edgeA.other = j->m_bodyB;
    j->m_edgeA.prev = NULL;
    j->m_edgeA.next = j->m_bodyA->m_jointList;
    if (j->m_bodyA->m_jointList) j->m_bodyA->m_jointList->prev = &j->m_edgeA;
    j->m_bodyA->m_jointList = &j->m_edgeA;

    j->m_edgeB.joint = j;
    j->m_edgeB.other = j->m_bodyA;
    j->m_edgeB.prev = NULL;
    j->m_edgeB.next = j->m_bodyB->m_jointList;
    if (j->m_bodyB->m_jointList) j->m_bodyB->m_jointList->prev = &j->m_edgeB;
    j->m_bodyB->m_jointList = &j->m_edgeB;

    bodyA = def->bodyA;
    bodyB = def->bodyB;

    // If the joint prevents collisions, then flag any contacts for filtering.
    if (def->collideConnected == b2False)
    {
        struct b2ContactEdge* edge;

        edge = b2BodyGetContactListRef(bodyB);
        while (edge)
        {
            if (edge->other == bodyA)
            {
                // Flag the contact for filtering at the next time step (where either
                // body is awake).
                b2ContactFlagForFiltering(edge->contact);
            }

            edge = edge->next;
        }
    }

    // Note: creating a joint doesn't wake the bodies.

    return j;
}

B2_API
void
b2WorldDeleteJoint(
    struct b2World* p,
    struct b2Joint* j)
{
    int collideConnected;

    struct b2Body* bodyA;
    struct b2Body* bodyB;

    b2Assert(b2WorldIsLocked(p) == b2False);
    if (b2WorldIsLocked(p))
    {
        return;
    }

    collideConnected = j->m_collideConnected;

    // Remove from the doubly linked list.
    if (j->m_prev)
    {
        j->m_prev->m_next = j->m_next;
    }

    if (j->m_next)
    {
        j->m_next->m_prev = j->m_prev;
    }

    if (j == p->m_jointList)
    {
        p->m_jointList = j->m_next;
    }

    // Disconnect from island graph.
    bodyA = j->m_bodyA;
    bodyB = j->m_bodyB;

    // Wake up connected bodies.
    b2BodySetAwake(bodyA, b2True);
    b2BodySetAwake(bodyB, b2True);

    // Remove from body 1.
    if (j->m_edgeA.prev)
    {
        j->m_edgeA.prev->next = j->m_edgeA.next;
    }

    if (j->m_edgeA.next)
    {
        j->m_edgeA.next->prev = j->m_edgeA.prev;
    }

    if (&j->m_edgeA == bodyA->m_jointList)
    {
        bodyA->m_jointList = j->m_edgeA.next;
    }

    j->m_edgeA.prev = NULL;
    j->m_edgeA.next = NULL;

    // Remove from body 2
    if (j->m_edgeB.prev)
    {
        j->m_edgeB.prev->next = j->m_edgeB.next;
    }

    if (j->m_edgeB.next)
    {
        j->m_edgeB.next->prev = j->m_edgeB.prev;
    }

    if (&j->m_edgeB == bodyB->m_jointList)
    {
        bodyB->m_jointList = j->m_edgeB.next;
    }

    j->m_edgeB.prev = NULL;
    j->m_edgeB.next = NULL;

    b2JointDelete(j, &p->m_blockAllocator);

    b2Assert(p->m_jointCount > 0);
    --p->m_jointCount;

    // If the joint prevents collisions, then flag any contacts for filtering.
    if (collideConnected == b2False)
    {
        struct b2ContactEdge* edge;
        edge = b2BodyGetContactListRef(bodyB);
        while (edge)
        {
            if (edge->other == bodyA)
            {
                // Flag the contact for filtering at the next time step (where either
                // body is awake).
                b2ContactFlagForFiltering(edge->contact);
            }

            edge = edge->next;
        }
    }
}

B2_API
void
b2WorldStep(
    struct b2World* p,
    float dt,
    int32 velocityIterations,
    int32 positionIterations)
{
    //mmUInt64_t us0, us1, us2, us3;
    float ms0, ms1, ms2, ms3;
    struct b2Timer stepTimer;

    struct b2TimeStep step;

    b2TimerMake(&stepTimer);
    ms0 = b2TimerGetMilliseconds(&stepTimer);

    // If new fixtures were added, we need to find the new contacts.
    if (p->m_newContacts)
    {
        b2ContactManagerFindNewContacts(&p->m_contactManager);
        p->m_newContacts = b2False;
    }

    p->m_locked = b2True;

    step.dt = dt;
    step.velocityIterations = velocityIterations;
    step.positionIterations = positionIterations;
    if (dt > 0.0f)
    {
        step.inv_dt = 1.0f / dt;
    }
    else
    {
        step.inv_dt = 0.0f;
    }

    step.dtRatio = p->m_inv_dt0 * dt;

    step.warmStarting = p->m_warmStarting;

    // Update contacts. This is where some contacts are destroyed.
    {
        ms2 = b2TimerGetMilliseconds(&stepTimer);
        b2ContactManagerCollide(&p->m_contactManager);
        ms3 = b2TimerGetMilliseconds(&stepTimer);
        p->m_profile.collide = (ms3 - ms2);
    }

    // Integrate velocities, solve velocity constraints, and integrate positions.
    if (p->m_stepComplete && step.dt > 0.0f)
    {
        ms2 = b2TimerGetMilliseconds(&stepTimer);
        b2WorldSolve(p, &step);
        ms3 = b2TimerGetMilliseconds(&stepTimer);
        p->m_profile.solve = (ms3 - ms2);
    }

    // Handle TOI events.
    if (p->m_continuousPhysics && step.dt > 0.0f)
    {
        ms2 = b2TimerGetMilliseconds(&stepTimer);
        b2WorldSolveTOI(p, &step);
        ms3 = b2TimerGetMilliseconds(&stepTimer);
        p->m_profile.solveTOI = (ms3 - ms2);
    }

    if (step.dt > 0.0f)
    {
        p->m_inv_dt0 = step.inv_dt;
    }

    if (p->m_clearForces)
    {
        b2WorldClearForces(p);
    }

    p->m_locked = b2False;

    ms1 = b2TimerGetMilliseconds(&stepTimer);
    p->m_profile.step = (ms1 - ms0);
}

B2_API
void
b2WorldClearForces(
    struct b2World* p)
{
    struct b2Body* body;

    for (body = p->m_bodyList; body; body = b2BodyGetNextRef(body))
    {
        b2Vec2SetZero(body->m_force);
        body->m_torque = 0.0f;
    }
}

B2_API
void
b2WorldDebugDraw(
    struct b2World* p)
{
    uint32 flags;

    if (p->m_debugDraw == NULL)
    {
        return;
    }

    flags = b2DrawGetFlags(p->m_debugDraw);

    if (flags & b2DrawBitShape)
    {
        struct b2Body* b;

        for (b = p->m_bodyList; b; b = b2BodyGetNextRef(b))
        {
            b2TransformConstRef xf;
            struct b2Fixture* f;

            xf = b2BodyGetTransform(b);
            for (f = b2BodyGetFixtureListRef(b); f; f = b2FixtureGetNextRef(f))
            {
                if (b2BodyGetType(b) == b2BodyTypeDynamic && b->m_mass == 0.0f)
                {
                    static const b2Color color = { 1.0f, 0.0f, 0.0f, 1.0f };
                    // Bad body
                    b2WorldDrawShape(p, f, xf, color);
                }
                else if (b2BodyIsEnabled(b) == b2False)
                {
                    static const b2Color color = { 0.5f, 0.5f, 0.3f, 1.0f };
                    b2WorldDrawShape(p, f, xf, color);
                }
                else if (b2BodyGetType(b) == b2BodyTypeStatic)
                {
                    static const b2Color color = { 0.5f, 0.9f, 0.5f, 1.0f };
                    b2WorldDrawShape(p, f, xf, color);
                }
                else if (b2BodyGetType(b) == b2BodyTypeKinematic)
                {
                    static const b2Color color = { 0.5f, 0.5f, 0.9f, 1.0f };
                    b2WorldDrawShape(p, f, xf, color);
                }
                else if (b2BodyIsAwake(b) == b2False)
                {
                    static const b2Color color = { 0.6f, 0.6f, 0.6f, 1.0f };
                    b2WorldDrawShape(p, f, xf, color);
                }
                else
                {
                    static const b2Color color = { 0.9f, 0.7f, 0.7f, 1.0f };
                    b2WorldDrawShape(p, f, xf, color);
                }
            }
        }
    }

    if (flags & b2DrawBitJoint)
    {
        struct b2Joint* j;

        for (j = p->m_jointList; j; j = b2JointGetNextRef(j))
        {
            b2JointDraw(j, p->m_debugDraw);
        }
    }

    if (flags & b2DrawBitPair)
    {
        static const b2Color color = { 0.3f, 0.9f, 0.9f, 1.0f };

        struct b2Contact* c;

        for (c = p->m_contactManager.m_contactList; c; c = b2ContactGetNextRef(c))
        {
            struct b2Fixture* fixtureA;
            struct b2Fixture* fixtureB;
            int32 indexA;
            int32 indexB;
            b2Vec2 cA;
            b2Vec2 cB;

            fixtureA = b2ContactGetFixtureARef(c);
            fixtureB = b2ContactGetFixtureBRef(c);
            indexA = b2ContactGetChildIndexA(c);
            indexB = b2ContactGetChildIndexB(c);
            b2AABBGetCenter(b2FixtureGetAABB(fixtureA, indexA), cA);
            b2AABBGetCenter(b2FixtureGetAABB(fixtureB, indexB), cB);

            b2DrawSegment(p->m_debugDraw, cA, cB, color);
        }
    }

    if (flags & b2DrawBitAabb)
    {
        static const b2Color color = { 0.9f, 0.3f, 0.9f, 1.0f };

        struct b2BroadPhase* bp;

        struct b2Body* b;

        bp = &p->m_contactManager.m_broadPhase;

        for (b = p->m_bodyList; b; b = b2BodyGetNextRef(b))
        {
            struct b2Fixture* f;

            if (b2BodyIsEnabled(b) == b2False)
            {
                continue;
            }

            for (f = b2BodyGetFixtureListRef(b); f; f = b2FixtureGetNextRef(f))
            {
                int32 i;

                for (i = 0; i < f->m_proxyCount; ++i)
                {
                    struct b2FixtureProxy* proxy;
                    const struct b2AABB* aabb;
                    b2Vec2 vs[4];

                    proxy = f->m_proxies + i;
                    aabb = b2BroadPhaseGetFatAABB(bp, proxy->proxyId);

                    b2Vec2Make(vs[0], aabb->lowerBound[0], aabb->lowerBound[1]);
                    b2Vec2Make(vs[1], aabb->upperBound[0], aabb->lowerBound[1]);
                    b2Vec2Make(vs[2], aabb->upperBound[0], aabb->upperBound[1]);
                    b2Vec2Make(vs[3], aabb->lowerBound[0], aabb->upperBound[1]);

                    b2DrawPolygon(p->m_debugDraw, vs, 4, color);
                }
            }
        }
    }

    if (flags & b2DrawBitCenterOfMass)
    {
        struct b2Body* b;

        for (b = p->m_bodyList; b; b = b2BodyGetNextRef(b))
        {
            b2Transform xf;
            b2TransformConstRef xfr;
            b2Vec2ConstRef wcr;
            xfr = b2BodyGetTransform(b);
            wcr = b2BodyGetWorldCenter(b);
            
            b2TransformAssign(xf, xfr);
            b2Vec2Assign(xf[0], wcr);
            b2DrawTransform(p->m_debugDraw, xf);
        }
    }
}

B2_API
void
b2WorldQueryAABB(
    const struct b2World* p,
    struct b2QueryCallback* callback,
    const struct b2AABB* aabb)
{
    struct b2WorldQueryWrapper wrapper;
    wrapper.broadPhase = &p->m_contactManager.m_broadPhase;
    wrapper.callback = callback;
    b2BroadPhaseQuery(
        &p->m_contactManager.m_broadPhase, 
        aabb, 
        &wrapper, 
        &b2WorldQueryWrapperQueryCallback);
}

B2_API
void
b2WorldRayCast(
    const struct b2World* p,
    struct b2RayCastCallback* callback,
    const b2Vec2 point1,
    const b2Vec2 point2)
{
    struct b2WorldRayCastWrapper wrapper;
    struct b2RayCastInput input;
    wrapper.broadPhase = &p->m_contactManager.m_broadPhase;
    wrapper.callback = callback;
    input.maxFraction = 1.0f;
    b2Vec2Assign(input.p1, point1);
    b2Vec2Assign(input.p2, point2);
    b2BroadPhaseRayCast(
        &p->m_contactManager.m_broadPhase, 
        &input, 
        &wrapper, 
        &b2WorldRayCastWrapperRayCastCallback);
}

B2_API
void
b2WorldSetAllowSleeping(
    struct b2World* p,
    int flag)
{
    if (flag == p->m_allowSleep)
    {
        return;
    }

    p->m_allowSleep = flag;
    if (p->m_allowSleep == b2False)
    {
        struct b2Body* b;

        for (b = p->m_bodyList; b; b = b->m_next)
        {
            b2BodySetAwake(b, b2True);
        }
    }
}

B2_API
int32
b2WorldGetProxyCount(
    const struct b2World* p)
{
    return b2BroadPhaseGetProxyCount(&p->m_contactManager.m_broadPhase);
}

B2_API
int32
b2WorldGetContactCount(
    const struct b2World* p)
{
    return p->m_contactManager.m_contactCount;
}

B2_API
int32
b2WorldGetTreeHeight(
    const struct b2World* p)
{
    return b2BroadPhaseGetTreeHeight(&p->m_contactManager.m_broadPhase);
}

B2_API
int32
b2WorldGetTreeBalance(
    const struct b2World* p)
{
    return b2BroadPhaseGetTreeBalance(&p->m_contactManager.m_broadPhase);
}

B2_API
float
b2WorldGetTreeQuality(
    const struct b2World* p)
{
    return b2BroadPhaseGetTreeQuality(&p->m_contactManager.m_broadPhase);
}

B2_API
void
b2WorldSetGravity(
    struct b2World* p,
    const b2Vec2 gravity)
{
    b2Vec2Assign(p->m_gravity, gravity);
}

B2_API
void
b2WorldShiftOrigin(
    struct b2World* p,
    const b2Vec2 newOrigin)
{
    struct b2Body* b;
    struct b2Joint* j;

    b2Assert(p->m_locked == b2False);
    if (p->m_locked)
    {
        return;
    }

    for (b = p->m_bodyList; b; b = b->m_next)
    {
        b2Vec2Sub(b->m_xf[0], b->m_xf[0], newOrigin);
        b2Vec2Sub(b->m_sweep.c0, b->m_sweep.c0, newOrigin);
        b2Vec2Sub(b->m_sweep.c, b->m_sweep.c, newOrigin);
    }

    for (j = p->m_jointList; j; j = j->m_next)
    {
        b2JointShiftOrigin(j, newOrigin);
    }

    b2BroadPhaseShiftOrigin(&p->m_contactManager.m_broadPhase, newOrigin);
}

B2_API
void
b2WorldDump(
    struct b2World* p)
{
    int32 i;

    struct b2Body* b;
    struct b2Joint* j;

    if (p->m_locked)
    {
        return;
    }

    b2OpenDump("box2d_dump.inl");

    b2Dump("b2Vec2 g(%.9g, %.9g);\n", p->m_gravity[0], p->m_gravity[1]);
    b2Dump("m_world->SetGravity(g);\n");

    b2Dump("b2Body** bodies = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", p->m_bodyCount);
    b2Dump("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", p->m_jointCount);

    i = 0;
    for (b = p->m_bodyList; b; b = b->m_next)
    {
        b->m_islandIndex = i;
        b2BodyDump(b);
        ++i;
    }

    i = 0;
    for (j = p->m_jointList; j; j = j->m_next)
    {
        j->m_index = i;
        ++i;
    }

    // First pass on joints, skip gear joints.
    for (j = p->m_jointList; j; j = j->m_next)
    {
        if (j->m_type == b2JointTypeGear)
        {
            continue;
        }

        b2Dump("{\n");
        b2JointDump(j);
        b2Dump("}\n");
    }

    // Second pass on joints, only gear joints.
    for (j = p->m_jointList; j; j = j->m_next)
    {
        if (j->m_type != b2JointTypeGear)
        {
            continue;
        }

        b2Dump("{\n");
        b2JointDump(j);
        b2Dump("}\n");
    }

    b2Dump("b2Free(joints);\n");
    b2Dump("b2Free(bodies);\n");
    b2Dump("joints = nullptr;\n");
    b2Dump("bodies = nullptr;\n");

    b2CloseDump();
}

// Find islands, integrate and solve constraints, solve position constraints
B2_API
void
b2WorldSolve(
    struct b2World* p,
    const struct b2TimeStep* step)
{
    struct b2Island island;

    struct b2Body* b;
    struct b2Contact* c;
    struct b2Joint* j;

    int32 stackSize;
    struct b2Body** stack;

    struct b2Body* seed;

    p->m_profile.solveInit = 0.0f;
    p->m_profile.solveVelocity = 0.0f;
    p->m_profile.solvePosition = 0.0f;

    // Size the island for the worst case.
    b2IslandPrepare(
        &island,
        p->m_bodyCount,
        p->m_contactManager.m_contactCount,
        p->m_jointCount,
        &p->m_stackAllocator,
        p->m_contactManager.m_contactListener);

    // Clear all the island flags.
    for (b = p->m_bodyList; b; b = b->m_next)
    {
        b->m_flags &= ~b2BodyFlagIsland;
    }
    for (c = p->m_contactManager.m_contactList; c; c = c->m_next)
    {
        c->m_flags &= ~b2ContactFlagIsland;
    }
    for (j = p->m_jointList; j; j = j->m_next)
    {
        j->m_islandFlag = b2False;
    }

    // Build and simulate all awake islands.
    stackSize = p->m_bodyCount;
    stack = (struct b2Body**)b2StackAllocatorAllocate(&p->m_stackAllocator, stackSize * sizeof(struct b2Body*));
    for (seed = p->m_bodyList; seed; seed = seed->m_next)
    {
        int32 stackCount;

        struct b2Profile profile;

        int32 i;

        if (seed->m_flags & b2BodyFlagIsland)
        {
            continue;
        }

        if (b2BodyIsAwake(seed) == b2False || b2BodyIsEnabled(seed) == b2False)
        {
            continue;
        }

        // The seed can be dynamic or kinematic.
        if (b2BodyGetType(seed) == b2BodyTypeStatic)
        {
            continue;
        }

        // Reset island and stack.
        b2IslandClear(&island);
        stackCount = 0;
        stack[stackCount++] = seed;
        seed->m_flags |= b2BodyFlagIsland;

        // Perform a depth first search (DFS) on the constraint graph.
        while (stackCount > 0)
        {
            struct b2Body* b;

            struct b2ContactEdge* ce;

            struct b2JointEdge* je;

            // Grab the next body off the stack and add it to the island.
            b = stack[--stackCount];
            b2Assert(b2BodyIsEnabled(b) == b2True);
            b2IslandAddBody(&island, b);

            // To keep islands as small as possible, we don't
            // propagate islands across static bodies.
            if (b2BodyGetType(b) == b2BodyTypeStatic)
            {
                continue;
            }

            // Make sure the body is awake (without resetting sleep timer).
            b->m_flags |= b2BodyFlagAwake;

            // Search all contacts connected to this body.
            for (ce = b->m_contactList; ce; ce = ce->next)
            {
                struct b2Contact* contact;

                int sensorA;
                int sensorB;

                struct b2Body* other;

                contact = ce->contact;

                // Has this contact already been added to an island?
                if (contact->m_flags & b2ContactFlagIsland)
                {
                    continue;
                }

                // Is this contact solid and touching?
                if (b2ContactIsEnabled(contact) == b2False ||
                    b2ContactIsTouching(contact) == b2False)
                {
                    continue;
                }

                // Skip sensors.
                sensorA = contact->m_fixtureA->m_isSensor;
                sensorB = contact->m_fixtureB->m_isSensor;
                if (sensorA || sensorB)
                {
                    continue;
                }

                b2IslandAddContact(&island, contact);
                contact->m_flags |= b2ContactFlagIsland;

                other = ce->other;

                // Was the other body already added to this island?
                if (other->m_flags & b2BodyFlagIsland)
                {
                    continue;
                }

                b2Assert(stackCount < stackSize);
                stack[stackCount++] = other;
                other->m_flags |= b2BodyFlagIsland;
            }

            // Search all joints connect to this body.
            for (je = b->m_jointList; je; je = je->next)
            {
                struct b2Body* other;

                if (je->joint->m_islandFlag == b2True)
                {
                    continue;
                }

                other = je->other;

                // Don't simulate joints connected to disabled bodies.
                if (b2BodyIsEnabled(other) == b2False)
                {
                    continue;
                }

                b2IslandAddJoint(&island, je->joint);
                je->joint->m_islandFlag = b2True;

                if (other->m_flags & b2BodyFlagIsland)
                {
                    continue;
                }

                b2Assert(stackCount < stackSize);
                stack[stackCount++] = other;
                other->m_flags |= b2BodyFlagIsland;
            }
        }

        b2IslandSolve(&island, &profile, step, p->m_gravity, p->m_allowSleep);
        p->m_profile.solveInit += profile.solveInit;
        p->m_profile.solveVelocity += profile.solveVelocity;
        p->m_profile.solvePosition += profile.solvePosition;

        // Post solve cleanup.
        for (i = 0; i < island.m_bodyCount; ++i)
        {
            struct b2Body* b;

            // Allow static bodies to participate in other islands.
            b = island.m_bodies[i];
            if (b2BodyGetType(b) == b2BodyTypeStatic)
            {
                b->m_flags &= ~b2BodyFlagIsland;
            }
        }
    }

    b2StackAllocatorFree(&p->m_stackAllocator, stack);

    {
        struct b2Timer timer;

        struct b2Body* b;

        b2TimerMake(&timer);

        // Synchronize fixtures, check for out of range bodies.
        for (b = p->m_bodyList; b; b = b2BodyGetNextRef(b))
        {
            // If a body was not in an island then it did not move.
            if ((b->m_flags & b2BodyFlagIsland) == 0)
            {
                continue;
            }

            if (b2BodyGetType(b) == b2BodyTypeStatic)
            {
                continue;
            }

            // Update fixtures (for broad-phase).
            b2BodySynchronizeFixtures(b);
        }

        // Look for new contacts.
        b2ContactManagerFindNewContacts(&p->m_contactManager);
        p->m_profile.broadphase = b2TimerGetMilliseconds(&timer);
    }

    b2IslandDiscard(&island);
}

// Find TOI contacts and solve them.
B2_API
void
b2WorldSolveTOI(
    struct b2World* p,
    const struct b2TimeStep* step)
{
    struct b2Island island;

    b2IslandPrepare(
        &island,
        2 * b2_maxTOIContacts, 
        b2_maxTOIContacts, 
        0, 
        &p->m_stackAllocator, 
        p->m_contactManager.m_contactListener);

    if (p->m_stepComplete)
    {
        struct b2Body* b;
        struct b2Contact* c;

        for (b = p->m_bodyList; b; b = b->m_next)
        {
            b->m_flags &= ~b2BodyFlagIsland;
            b->m_sweep.alpha0 = 0.0f;
        }

        for (c = p->m_contactManager.m_contactList; c; c = c->m_next)
        {
            // Invalidate TOI
            c->m_flags &= ~(b2ContactFlagToi | b2ContactFlagIsland);
            c->m_toiCount = 0;
            c->m_toi = 1.0f;
        }
    }

    // Find TOI events and solve them.
    for (;;)
    {
        struct b2Contact* minContact;
        float minAlpha;

        struct b2Contact* c;

        struct b2Fixture* fA;
        struct b2Fixture* fB;
        struct b2Body* bA;
        struct b2Body* bB;

        struct b2Sweep backup1;
        struct b2Sweep backup2;

        struct b2Body* bodies[2];

        int32 i;

        struct b2TimeStep subStep;

        // Find the first TOI.
        minContact = NULL;
        minAlpha = 1.0f;

        for (c = p->m_contactManager.m_contactList; c; c = c->m_next)
        {
            // Is this contact disabled?
            if (b2ContactIsEnabled(c) == b2False)
            {
                continue;
            }

            // Prevent excessive sub-stepping.
            if (c->m_toiCount > b2_maxSubSteps)
            {
                continue;
            }

            float alpha = 1.0f;
            if (c->m_flags & b2ContactFlagToi)
            {
                // This contact has a valid cached TOI.
                alpha = c->m_toi;
            }
            else
            {
                struct b2Fixture* fA;
                struct b2Fixture* fB;

                struct b2Body* bA;
                struct b2Body* bB;

                enum b2BodyType typeA;
                enum b2BodyType typeB;

                int activeA;
                int activeB;

                int collideA;
                int collideB;

                float alpha0;

                int32 indexA;
                int32 indexB;

                struct b2TOIInput input;
                struct b2TOIOutput output;

                float beta;

                b2TOIInputReset(&input);

                fA = b2ContactGetFixtureARef(c);
                fB = b2ContactGetFixtureBRef(c);

                // Is there a sensor?
                if (b2FixtureIsSensor(fA) || b2FixtureIsSensor(fB))
                {
                    continue;
                }

                bA = b2FixtureGetBodyRef(fA);
                bB = b2FixtureGetBodyRef(fB);

                typeA = bA->m_type;
                typeB = bB->m_type;
                b2Assert(typeA == b2BodyTypeDynamic || typeB == b2BodyTypeDynamic);

                activeA = b2BodyIsAwake(bA) && typeA != b2BodyTypeStatic;
                activeB = b2BodyIsAwake(bB) && typeB != b2BodyTypeStatic;

                // Is at least one body active (awake and dynamic or kinematic)?
                if (activeA == b2False && activeB == b2False)
                {
                    continue;
                }

                collideA = b2BodyIsBullet(bA) || typeA != b2BodyTypeDynamic;
                collideB = b2BodyIsBullet(bB) || typeB != b2BodyTypeDynamic;

                // Are these two non-bullet dynamic bodies?
                if (collideA == b2False && collideB == b2False)
                {
                    continue;
                }

                // Compute the TOI for this contact.
                // Put the sweeps onto the same time interval.
                alpha0 = bA->m_sweep.alpha0;

                if (bA->m_sweep.alpha0 < bB->m_sweep.alpha0)
                {
                    alpha0 = bB->m_sweep.alpha0;
                    b2SweepAdvance(&bA->m_sweep, alpha0);
                }
                else if (bB->m_sweep.alpha0 < bA->m_sweep.alpha0)
                {
                    alpha0 = bA->m_sweep.alpha0;
                    b2SweepAdvance(&bB->m_sweep, alpha0);
                }

                b2Assert(alpha0 < 1.0f);

                indexA = b2ContactGetChildIndexA(c);
                indexB = b2ContactGetChildIndexB(c);

                // Compute the time of impact in interval [0, minTOI]
                b2DistanceProxySetShape(&input.proxyA, b2FixtureGetShape(fA), indexA);
                b2DistanceProxySetShape(&input.proxyB, b2FixtureGetShape(fB), indexB);
                input.sweepA = bA->m_sweep;
                input.sweepB = bB->m_sweep;
                input.tMax = 1.0f;

                b2TimeOfImpact(&output, &input);

                // Beta is the fraction of the remaining portion of the .
                beta = output.t;
                if (output.state == b2TOIOutputTouching)
                {
                    alpha = b2MinFloat(alpha0 + (1.0f - alpha0) * beta, 1.0f);
                }
                else
                {
                    alpha = 1.0f;
                }

                c->m_toi = alpha;
                c->m_flags |= b2ContactFlagToi;
            }

            if (alpha < minAlpha)
            {
                // This is the minimum TOI found so far.
                minContact = c;
                minAlpha = alpha;
            }
        }

        if (minContact == NULL || 1.0f - 10.0f * b2_epsilon < minAlpha)
        {
            // No more TOI events. Done!
            p->m_stepComplete = b2True;
            break;
        }

        // Advance the bodies to the TOI.
        fA = b2ContactGetFixtureARef(minContact);
        fB = b2ContactGetFixtureBRef(minContact);
        bA = b2FixtureGetBodyRef(fA);
        bB = b2FixtureGetBodyRef(fB);

        backup1 = bA->m_sweep;
        backup2 = bB->m_sweep;

        b2BodyAdvance(bA, minAlpha);
        b2BodyAdvance(bB, minAlpha);

        // The TOI contact likely has some new contact points.
        b2ContactUpdate(minContact, p->m_contactManager.m_contactListener);
        minContact->m_flags &= ~b2ContactFlagToi;
        ++minContact->m_toiCount;

        // Is the contact solid?
        if (b2ContactIsEnabled(minContact) == b2False || b2ContactIsTouching(minContact) == b2False)
        {
            // Restore the sweeps.
            b2ContactSetEnabled(minContact, b2False);
            bA->m_sweep = backup1;
            bB->m_sweep = backup2;
            b2BodySynchronizeTransform(bA);
            b2BodySynchronizeTransform(bB);
            continue;
        }

        b2BodySetAwake(bA, b2True);
        b2BodySetAwake(bB, b2True);

        // Build the island
        b2IslandClear(&island);
        b2IslandAddBody(&island, bA);
        b2IslandAddBody(&island, bB);
        b2IslandAddContact(&island, minContact);

        bA->m_flags |= b2BodyFlagIsland;
        bB->m_flags |= b2BodyFlagIsland;
        minContact->m_flags |= b2ContactFlagIsland;

        // Get contacts on bodyA and bodyB.
        bodies[0] = bA;
        bodies[1] = bB;
        for (i = 0; i < 2; ++i)
        {
            struct b2Body* body;

            body = bodies[i];
            if (body->m_type == b2BodyTypeDynamic)
            {
                struct b2ContactEdge* ce;

                for (ce = body->m_contactList; ce; ce = ce->next)
                {
                    struct b2Contact* contact;

                    struct b2Body* other;

                    int sensorA;
                    int sensorB;

                    struct b2Sweep backup;

                    if (island.m_bodyCount == island.m_bodyCapacity)
                    {
                        break;
                    }

                    if (island.m_contactCount == island.m_contactCapacity)
                    {
                        break;
                    }

                    contact = ce->contact;

                    // Has this contact already been added to the island?
                    if (contact->m_flags & b2ContactFlagIsland)
                    {
                        continue;
                    }

                    // Only add static, kinematic, or bullet bodies.
                    other = ce->other;
                    if (other->m_type == b2BodyTypeDynamic &&
                        b2BodyIsBullet(body) == b2False && b2BodyIsBullet(other) == b2False)
                    {
                        continue;
                    }

                    // Skip sensors.
                    sensorA = contact->m_fixtureA->m_isSensor;
                    sensorB = contact->m_fixtureB->m_isSensor;
                    if (sensorA || sensorB)
                    {
                        continue;
                    }

                    // Tentatively advance the body to the TOI.
                    backup = other->m_sweep;
                    if ((other->m_flags & b2BodyFlagIsland) == 0)
                    {
                        b2BodyAdvance(other, minAlpha);
                    }

                    // Update the contact points
                    b2ContactUpdate(contact, p->m_contactManager.m_contactListener);

                    // Was the contact disabled by the user?
                    if (b2ContactIsEnabled(contact) == b2False)
                    {
                        other->m_sweep = backup;
                        b2BodySynchronizeTransform(other);
                        continue;
                    }

                    // Are there contact points?
                    if (b2ContactIsTouching(contact) == b2False)
                    {
                        other->m_sweep = backup;
                        b2BodySynchronizeTransform(other);
                        continue;
                    }

                    // Add the contact to the island
                    contact->m_flags |= b2ContactFlagIsland;
                    b2IslandAddContact(&island, contact);

                    // Has the other body already been added to the island?
                    if (other->m_flags & b2BodyFlagIsland)
                    {
                        continue;
                    }

                    // Add the other body to the island.
                    other->m_flags |= b2BodyFlagIsland;

                    if (other->m_type != b2BodyTypeStatic)
                    {
                        b2BodySetAwake(other, b2True);
                    }

                    b2IslandAddBody(&island, other);
                }
            }
        }

        subStep.dt = (1.0f - minAlpha) * step->dt;
        subStep.inv_dt = 1.0f / subStep.dt;
        subStep.dtRatio = 1.0f;
        subStep.positionIterations = 20;
        subStep.velocityIterations = step->velocityIterations;
        subStep.warmStarting = b2False;
        b2IslandSolveTOI(&island, &subStep, bA->m_islandIndex, bB->m_islandIndex);

        // Reset island flags and synchronize broad-phase proxies.
        for (i = 0; i < island.m_bodyCount; ++i)
        {
            struct b2Body* body;

            struct b2ContactEdge* ce;

            body = island.m_bodies[i];
            body->m_flags &= ~b2BodyFlagIsland;

            if (body->m_type != b2BodyTypeDynamic)
            {
                continue;
            }

            b2BodySynchronizeFixtures(body);

            // Invalidate all contact TOIs on this displaced body.
            for (ce = body->m_contactList; ce; ce = ce->next)
            {
                ce->contact->m_flags &= ~(b2ContactFlagToi | b2ContactFlagIsland);
            }
        }

        // Commit fixture proxy movements to the broad-phase so that new contacts are created.
        // Also, some contacts can be destroyed.
        b2ContactManagerFindNewContacts(&p->m_contactManager);

        if (p->m_subStepping)
        {
            p->m_stepComplete = b2False;
            break;
        }
    }

    b2IslandDiscard(&island);
}

B2_API
void
b2WorldDrawShape(
    const struct b2World* p,
    struct b2Fixture* fixture,
    const b2Transform xf,
    const b2Color color)
{
    switch (b2FixtureGetType(fixture))
    {
    case b2ShapeTypeCircle:
    {
        static const float cUnitX[2] = { +1.0f,  0.0f, };

        struct b2ShapeCircle* circle;

        b2Vec2 center;
        float radius;
        b2Vec2 axis;

        circle = (struct b2ShapeCircle*)b2FixtureGetShape(fixture);
        b2TransformMulVec2(center, xf, circle->m_p);
        radius = circle->m_radius;
        b2RotMulVec2(axis, xf[1], cUnitX);

        b2DrawSolidCircle(p->m_debugDraw, center, radius, axis, color);
    }
    break;

    case b2ShapeTypeEdge:
    {
        struct b2ShapeEdge* edge;
        b2Vec2 v1;
        b2Vec2 v2;


        edge = (struct b2ShapeEdge*)b2FixtureGetShape(fixture);
        b2TransformMulVec2(v1, xf, edge->m_vertex1);
        b2TransformMulVec2(v2, xf, edge->m_vertex2);
        b2DrawSegment(p->m_debugDraw, v1, v2, color);

        if (edge->m_oneSided == b2False)
        {
            b2DrawPoint(p->m_debugDraw, v1, 4.0f, color);
            b2DrawPoint(p->m_debugDraw, v2, 4.0f, color);
        }
    }
    break;

    case b2ShapeTypeChain:
    {
        struct b2ShapeChain* chain;
        int32 count;
        const b2Vec2* vertices;

        int32 i;

        b2Vec2 v1;
        b2Vec2 v2;

        chain = (struct b2ShapeChain*)b2FixtureGetShape(fixture);
        count = chain->m_count;
        vertices = chain->m_vertices;

        b2TransformMulVec2(v1, xf, vertices[0]);
        for (i = 1; i < count; ++i)
        {
            b2TransformMulVec2(v2, xf, vertices[i]);
            b2DrawSegment(p->m_debugDraw, v1, v2, color);
            b2Vec2Assign(v1, v2);
        }
    }
    break;

    case b2ShapeTypePolygon:
    {
        struct b2ShapePolygon* poly;
        int32 vertexCount;
        b2Vec2 vertices[b2_maxPolygonVertices];

        int32 i;

        poly = (struct b2ShapePolygon*)b2FixtureGetShape(fixture);
        vertexCount = poly->m_count;
        b2Assert(vertexCount <= b2_maxPolygonVertices);

        for (i = 0; i < vertexCount; ++i)
        {
            b2TransformMulVec2(vertices[i], xf, poly->m_vertices[i]);
        }

        b2DrawSolidPolygon(p->m_debugDraw, vertices, vertexCount, color);
    }
    break;

    default:
        break;
    }
}
