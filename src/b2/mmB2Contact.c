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

#include "mmB2Contact.h"
#include "mmB2BlockAllocator.h"
#include "mmB2Shape.h"
#include "mmB2Fixture.h"
#include "mmB2WorldCallbacks.h"
#include "mmB2MetaAllocator.h"

#include "mmB2ContactChainCircle.h"
#include "mmB2ContactChainPolygon.h"
#include "mmB2ContactCircleCircle.h"
#include "mmB2ContactEdgeCircle.h"
#include "mmB2ContactEdgePolygon.h"
#include "mmB2ContactPolygonCircle.h"
#include "mmB2ContactPolygonPolygon.h"

#include <assert.h>

B2_API
void
b2ContactGetWorldManifold(
    const struct b2Contact* p,
    struct b2WorldManifold* worldManifold)
{
    const struct b2Body* bodyA = b2FixtureGetBody(p->m_fixtureA);
    const struct b2Body* bodyB = b2FixtureGetBody(p->m_fixtureB);
    const struct b2Shape* shapeA = b2FixtureGetShape(p->m_fixtureA);
    const struct b2Shape* shapeB = b2FixtureGetShape(p->m_fixtureB);

    b2WorldManifoldInitialize(
        worldManifold, 
        &p->m_manifold, 
        b2BodyGetTransform(bodyA), 
        shapeA->m_radius, 
        b2BodyGetTransform(bodyB), 
        shapeB->m_radius);
}

B2_API
int
b2ContactIsTouching(
    const struct b2Contact* p)
{
    return (p->m_flags & b2ContactFlagTouching) == b2ContactFlagTouching;
}

B2_API
void
b2ContactSetEnabled(
    struct b2Contact* p,
    int flag)
{
    if (flag)
    {
        p->m_flags |= b2ContactFlagEnabled;
    }
    else
    {
        p->m_flags &= ~b2ContactFlagEnabled;
    }
}

B2_API
int
b2ContactIsEnabled(
    const struct b2Contact* p)
{
    return (p->m_flags & b2ContactFlagEnabled) == b2ContactFlagEnabled;
}

B2_API
void
b2ContactResetFriction(
    struct b2Contact* p)
{
    p->m_friction = b2MixFriction(
        p->m_fixtureA->m_friction, 
        p->m_fixtureB->m_friction);
}

B2_API
void
b2ContactResetRestitution(
    struct b2Contact* p)
{
    p->m_restitution = b2MixRestitution(
        p->m_fixtureA->m_restitution, 
        p->m_fixtureB->m_restitution);
}

B2_API
void
b2ContactResetRestitutionThreshold(
    struct b2Contact* p)
{
    p->m_restitutionThreshold = b2MixRestitutionThreshold(
        p->m_fixtureA->m_restitutionThreshold, 
        p->m_fixtureB->m_restitutionThreshold);
}

B2_API
void
b2ContactEvaluate(
    struct b2Contact* obj,
    struct b2Manifold* manifold,
    const b2Transform xfA,
    const b2Transform xfB)
{
    typedef
    void (*Evaluate)(
        void* obj,
        struct b2Manifold* manifold, 
        const b2Transform xfA, 
        const b2Transform xfB);

    (*((Evaluate)obj->Meta->Evaluate))(
        obj,
        manifold,
        xfA,
        xfB);
}

/// Flag this contact for filtering. Filtering will occur the next time step.
B2_API
void
b2ContactFlagForFiltering(
    struct b2Contact* p)
{
    p->m_flags |= b2ContactFlagFilter;
}

B2_API
void
b2ContactPrepare(
    struct b2Contact* p,
    struct b2Fixture* fixtureA, int32 indexA,
    struct b2Fixture* fixtureB, int32 indexB)
{
    p->m_flags = b2ContactFlagEnabled;

    p->m_fixtureA = fixtureA;
    p->m_fixtureB = fixtureB;

    p->m_indexA = indexA;
    p->m_indexB = indexB;

    p->m_manifold.pointCount = 0;

    p->m_prev = NULL;
    p->m_next = NULL;

    p->m_nodeA.contact = NULL;
    p->m_nodeA.prev = NULL;
    p->m_nodeA.next = NULL;
    p->m_nodeA.other = NULL;

    p->m_nodeB.contact = NULL;
    p->m_nodeB.prev = NULL;
    p->m_nodeB.next = NULL;
    p->m_nodeB.other = NULL;

    p->m_toiCount = 0;

    p->m_friction = b2MixFriction(p->m_fixtureA->m_friction, p->m_fixtureB->m_friction);
    p->m_restitution = b2MixRestitution(p->m_fixtureA->m_restitution, p->m_fixtureB->m_restitution);
    p->m_restitutionThreshold = b2MixRestitutionThreshold(p->m_fixtureA->m_restitutionThreshold, p->m_fixtureB->m_restitutionThreshold);

    p->m_tangentSpeed = 0.0f;
}

B2_API
void
b2ContactDiscard(
    struct b2Contact* p)
{

}

#define P(a, b) &b2MetaAllocatorContact ## a ## And ## b, 1,
#define R(a, b) &b2MetaAllocatorContact ## b ## And ## a, 0,
#define E(a, b)                                     NULL, 0,

const struct b2ContactRegister s_registers[b2ShapeTypeCount][b2ShapeTypeCount] =
{
    { {P( Circle, Circle)}, {R( Circle, Edge)},{R( Circle, Polygon)},{R( Circle, Chain)}, },
    { {P(   Edge, Circle)}, {E(   Edge, Edge)},{P(   Edge, Polygon)},{E(   Edge, Chain)}, },
    { {P(Polygon, Circle)}, {R(Polygon, Edge)},{P(Polygon, Polygon)},{R(Polygon, Chain)}, },
    { {P(  Chain, Circle)}, {E(  Chain, Edge)},{P(  Chain, Polygon)},{E(  Chain, Chain)}, },
};

#undef P
#undef R
#undef E

static
struct b2Contact*
b2ContactAllocatorCreate(
    struct b2Fixture* fixtureA, int32 indexA,
    struct b2Fixture* fixtureB, int32 indexB,
    struct b2BlockAllocator* allocator,
    const struct b2MetaAllocator* meta)
{
    typedef
    struct b2Contact*
    (*Produce)(
        struct b2Contact* obj,
        struct b2Fixture* fixtureA, int32 indexA,
        struct b2Fixture* fixtureB, int32 indexB);

    void* mem;
    struct b2Contact* contact;
    mem = b2BlockAllocatorAllocate(allocator, (int32)meta->TypeSize);
    contact = (struct b2Contact*)(mem);
    (*((Produce)meta->Produce))(contact, fixtureA, indexA, fixtureB, indexB);
    return contact;
}

static
void
b2ContactAllocatorDelete(
    struct b2Contact* contact,
    struct b2BlockAllocator* allocator,
    const struct b2MetaAllocator* meta)
{
    typedef
    void
    (*Recycle)(
        struct b2Contact* contact);

    (*((Recycle)meta->Recycle))(contact);
    b2BlockAllocatorFree(allocator, contact, (int32)meta->TypeSize);
}

B2_API
struct b2Contact*
b2ContactCreate(
    struct b2Fixture* fixtureA, int32 indexA,
    struct b2Fixture* fixtureB, int32 indexB,
    struct b2BlockAllocator* allocator)
{
    const struct b2ContactRegister* r;

    enum b2ShapeType type1;
    enum b2ShapeType type2;

    type1 = b2FixtureGetType(fixtureA);
    type2 = b2FixtureGetType(fixtureB);

    b2Assert(0 <= type1 && type1 < b2ShapeTypeCount);
    b2Assert(0 <= type2 && type2 < b2ShapeTypeCount);

    r = &s_registers[type1][type2];
    if (NULL != r->meta)
    {
        if (r->primary)
        {
            return b2ContactAllocatorCreate(
                fixtureA, indexA, 
                fixtureB, indexB, 
                allocator, 
                r->meta);
        }
        else
        {
            return b2ContactAllocatorCreate(
                fixtureB, indexB, 
                fixtureA, indexA, 
                allocator, 
                r->meta);
        }
    }
    else
    {
        return NULL;
    }
}

B2_API
void
b2ContactDelete(
    struct b2Contact* contact,
    struct b2BlockAllocator* allocator)
{
    const struct b2ContactRegister* r;

    struct b2Fixture* fixtureA;
    struct b2Fixture* fixtureB;

    enum b2ShapeType typeA;
    enum b2ShapeType typeB;

    fixtureA = contact->m_fixtureA;
    fixtureB = contact->m_fixtureB;

    if (contact->m_manifold.pointCount > 0 &&
        b2FixtureIsSensor(fixtureA) == b2False &&
        b2FixtureIsSensor(fixtureB) == b2False)
    {
        b2BodySetAwake(b2FixtureGetBodyRef(fixtureA), b2True);
        b2BodySetAwake(b2FixtureGetBodyRef(fixtureB), b2True);
    }

    typeA = b2FixtureGetType(fixtureA);
    typeB = b2FixtureGetType(fixtureB);

    b2Assert(0 <= typeA && typeA < b2ShapeTypeCount);
    b2Assert(0 <= typeB && typeB < b2ShapeTypeCount);

    r = &s_registers[typeA][typeB];
    b2Assert(NULL != r->meta);
    b2ContactAllocatorDelete(contact, allocator, r->meta);
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
B2_API
void
b2ContactUpdate(
    struct b2Contact* p,
    struct b2ContactListener* listener)
{
    struct b2Manifold oldManifold;

    int touching;
    int wasTouching;

    int sensorA;
    int sensorB;
    int sensor;

    struct b2Body* bodyA;
    struct b2Body* bodyB;
    b2TransformConstRef xfA;
    b2TransformConstRef xfB;

    oldManifold = p->m_manifold;

    // Re-enable this contact.
    p->m_flags |= b2ContactFlagEnabled;

    touching = b2False;
    wasTouching = (p->m_flags & b2ContactFlagTouching) == b2ContactFlagTouching;

    sensorA = b2FixtureIsSensor(p->m_fixtureA);
    sensorB = b2FixtureIsSensor(p->m_fixtureB);
    sensor = sensorA || sensorB;

    bodyA = b2FixtureGetBodyRef(p->m_fixtureA);
    bodyB = b2FixtureGetBodyRef(p->m_fixtureB);
    xfA = b2BodyGetTransform(bodyA);
    xfB = b2BodyGetTransform(bodyB);

    // Is this contact a sensor?
    if (sensor)
    {
        const struct b2Shape* shapeA;
        const struct b2Shape* shapeB;

        shapeA = b2FixtureGetShape(p->m_fixtureA);
        shapeB = b2FixtureGetShape(p->m_fixtureB);
        touching = b2TestOverlap(shapeA, p->m_indexA, shapeB, p->m_indexB, xfA, xfB);

        // Sensors don't generate manifolds.
        p->m_manifold.pointCount = 0;
    }
    else
    {
        int32 i, j;

        b2ContactEvaluate(p, &p->m_manifold, xfA, xfB);
        touching = p->m_manifold.pointCount > 0;

        // Match old contact ids to new contact ids and copy the
        // stored impulses to warm start the solver.
        for (i = 0; i < p->m_manifold.pointCount; ++i)
        {
            struct b2ManifoldPoint* mp2;
            union b2ContactID id2;

            mp2 = p->m_manifold.points + i;
            mp2->normalImpulse = 0.0f;
            mp2->tangentImpulse = 0.0f;
            id2 = mp2->id;

            for (j = 0; j < oldManifold.pointCount; ++j)
            {
                struct b2ManifoldPoint* mp1;

                mp1 = oldManifold.points + j;

                if (mp1->id.key == id2.key)
                {
                    mp2->normalImpulse = mp1->normalImpulse;
                    mp2->tangentImpulse = mp1->tangentImpulse;
                    break;
                }
            }
        }

        if (touching != wasTouching)
        {
            b2BodySetAwake(bodyA, b2True);
            b2BodySetAwake(bodyB, b2True);
        }
    }

    if (touching)
    {
        p->m_flags |= b2ContactFlagTouching;
    }
    else
    {
        p->m_flags &= ~b2ContactFlagTouching;
    }

    if (wasTouching == b2False && touching == b2True && listener)
    {
        b2ContactListenerBeginContact(listener, p);
    }

    if (wasTouching == b2True && touching == b2False && listener)
    {
        b2ContactListenerEndContact(listener, p);
    }

    if (sensor == b2False && touching && listener)
    {
        b2ContactListenerPreSolve(listener, p, &oldManifold);
    }
}
