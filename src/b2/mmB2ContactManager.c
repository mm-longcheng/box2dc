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

#include "mmB2ContactManager.h"
#include "mmB2WorldCallbacks.h"
#include "mmB2Fixture.h"
#include "mmB2Contact.h"
#include "mmB2Body.h"
#include "mmB2WorldCallbacks.h"

#include <assert.h>
#include <stddef.h>

struct b2ContactFilter b2_defaultFilter = 
{
    &b2ContactFilterMetaDefault,
    NULL,
};

struct b2ContactListener b2_defaultListener = 
{
    &b2ContactListenerMetaDefault,
    NULL,
};

B2_API
void
b2ContactManagerInit(
    struct b2ContactManager* p)
{
    b2BroadPhaseInit(&p->m_broadPhase);

    p->m_contactList = NULL;
    p->m_contactCount = 0;
    p->m_contactFilter = &b2_defaultFilter;
    p->m_contactListener = &b2_defaultListener;
    p->m_allocator = NULL;
}

B2_API
void
b2ContactManagerDestroy(
    struct b2ContactManager* p)
{
    // Not need checking contact count.
    // b2Assert(0 == p->m_contactCount);

    p->m_allocator = NULL;
    p->m_contactListener = &b2_defaultListener;
    p->m_contactFilter = &b2_defaultFilter;
    p->m_contactCount = 0;
    p->m_contactList = NULL;

    b2BroadPhaseDestroy(&p->m_broadPhase);
}

B2_API
void
b2ContactManagerAddPair(
    struct b2ContactManager* p,
    void* proxyUserDataA,
    void* proxyUserDataB)
{
    struct b2FixtureProxy* proxyA;
    struct b2FixtureProxy* proxyB;

    struct b2Fixture* fixtureA;
    struct b2Fixture* fixtureB;

    int32 indexA;
    int32 indexB;

    struct b2Body* bodyA;
    struct b2Body* bodyB;

    struct b2ContactEdge* edge;

    struct b2Contact* c;

    proxyA = (struct b2FixtureProxy*)proxyUserDataA;
    proxyB = (struct b2FixtureProxy*)proxyUserDataB;

    fixtureA = proxyA->fixture;
    fixtureB = proxyB->fixture;

    indexA = proxyA->childIndex;
    indexB = proxyB->childIndex;

    bodyA = b2FixtureGetBodyRef(fixtureA);
    bodyB = b2FixtureGetBodyRef(fixtureB);

    // Are the fixtures on the same body?
    if (bodyA == bodyB)
    {
        return;
    }

    // TODO_ERIN use a hash table to remove a potential bottleneck when both
    // bodies have a lot of contacts.
    // Does a contact already exist?
    edge = b2BodyGetContactListRef(bodyB);
    while (edge)
    {
        if (edge->other == bodyA)
        {
            struct b2Fixture* fA;
            struct b2Fixture* fB;
            int32 iA;
            int32 iB;

            fA = b2ContactGetFixtureARef(edge->contact);
            fB = b2ContactGetFixtureBRef(edge->contact);
            iA = b2ContactGetChildIndexA(edge->contact);
            iB = b2ContactGetChildIndexB(edge->contact);

            if (fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB)
            {
                // A contact already exists.
                return;
            }

            if (fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA)
            {
                // A contact already exists.
                return;
            }
        }

        edge = edge->next;
    }

    // Does a joint override collision? Is at least one body dynamic?
    if (b2BodyShouldCollide(bodyB, bodyA) == b2False)
    {
        return;
    }

    // Check user filtering.
    if (p->m_contactFilter && b2ContactFilterShouldCollide(p->m_contactFilter, fixtureA, fixtureB) == b2False)
    {
        return;
    }

    // Call the factory.
    c = b2ContactCreate(fixtureA, indexA, fixtureB, indexB, p->m_allocator);
    if (c == NULL)
    {
        return;
    }

    // Contact creation may swap fixtures.
    fixtureA = b2ContactGetFixtureARef(c);
    fixtureB = b2ContactGetFixtureBRef(c);
    indexA = b2ContactGetChildIndexA(c);
    indexB = b2ContactGetChildIndexB(c);
    bodyA = b2FixtureGetBodyRef(fixtureA);
    bodyB = b2FixtureGetBodyRef(fixtureB);

    // Insert into the world.
    c->m_prev = NULL;
    c->m_next = p->m_contactList;
    if (p->m_contactList != NULL)
    {
        p->m_contactList->m_prev = c;
    }
    p->m_contactList = c;

    // Connect to island graph.

    // Connect to body A
    c->m_nodeA.contact = c;
    c->m_nodeA.other = bodyB;

    c->m_nodeA.prev = NULL;
    c->m_nodeA.next = bodyA->m_contactList;
    if (bodyA->m_contactList != NULL)
    {
        bodyA->m_contactList->prev = &c->m_nodeA;
    }
    bodyA->m_contactList = &c->m_nodeA;

    // Connect to body B
    c->m_nodeB.contact = c;
    c->m_nodeB.other = bodyA;

    c->m_nodeB.prev = NULL;
    c->m_nodeB.next = bodyB->m_contactList;
    if (bodyB->m_contactList != NULL)
    {
        bodyB->m_contactList->prev = &c->m_nodeB;
    }
    bodyB->m_contactList = &c->m_nodeB;

    ++p->m_contactCount;
}

B2_API
void
b2ContactManagerFindNewContacts(
    struct b2ContactManager* p)
{
    b2BroadPhaseUpdatePairs(&p->m_broadPhase, p, &b2ContactManagerAddPair);
}

B2_API
void
b2ContactManagerDelete(
    struct b2ContactManager* p,
    struct b2Contact* c)
{
    struct b2Fixture* fixtureA;
    struct b2Fixture* fixtureB;
    struct b2Body* bodyA;
    struct b2Body* bodyB;

    fixtureA = b2ContactGetFixtureARef(c);
    fixtureB = b2ContactGetFixtureBRef(c);
    bodyA = b2FixtureGetBodyRef(fixtureA);
    bodyB = b2FixtureGetBodyRef(fixtureB);

    if (p->m_contactListener && b2ContactIsTouching(c))
    {
        b2ContactListenerEndContact(p->m_contactListener, c);
    }

    // Remove from the world.
    if (c->m_prev)
    {
        c->m_prev->m_next = c->m_next;
    }

    if (c->m_next)
    {
        c->m_next->m_prev = c->m_prev;
    }

    if (c == p->m_contactList)
    {
        p->m_contactList = c->m_next;
    }

    // Remove from body 1
    if (c->m_nodeA.prev)
    {
        c->m_nodeA.prev->next = c->m_nodeA.next;
    }

    if (c->m_nodeA.next)
    {
        c->m_nodeA.next->prev = c->m_nodeA.prev;
    }

    if (&c->m_nodeA == bodyA->m_contactList)
    {
        bodyA->m_contactList = c->m_nodeA.next;
    }

    // Remove from body 2
    if (c->m_nodeB.prev)
    {
        c->m_nodeB.prev->next = c->m_nodeB.next;
    }

    if (c->m_nodeB.next)
    {
        c->m_nodeB.next->prev = c->m_nodeB.prev;
    }

    if (&c->m_nodeB == bodyB->m_contactList)
    {
        bodyB->m_contactList = c->m_nodeB.next;
    }

    // Call the factory.
    b2ContactDelete(c, p->m_allocator);
    --p->m_contactCount;
}

// This is the top level collision call for the time step. Here
// all the narrow phase collision is processed for the world
// contact list.
B2_API
void
b2ContactManagerCollide(
    struct b2ContactManager* p)
{
    struct b2Contact* c;
    // Update awake contacts.
    c = p->m_contactList;
    while (c)
    {
        struct b2Fixture* fixtureA;
        struct b2Fixture* fixtureB;
        int32 indexA;
        int32 indexB;
        struct b2Body* bodyA;
        struct b2Body* bodyB;

        int activeA;
        int activeB;

        int32 proxyIdA;
        int32 proxyIdB;
        int overlap;

        fixtureA = b2ContactGetFixtureARef(c);
        fixtureB = b2ContactGetFixtureBRef(c);
        indexA = b2ContactGetChildIndexA(c);
        indexB = b2ContactGetChildIndexB(c);
        bodyA = b2FixtureGetBodyRef(fixtureA);
        bodyB = b2FixtureGetBodyRef(fixtureB);

        // Is this contact flagged for filtering?
        if (c->m_flags & b2ContactFlagFilter)
        {
            // Should these bodies collide?
            if (b2BodyShouldCollide(bodyB, bodyA) == b2False)
            {
                struct b2Contact* cNuke = c;
                c = b2ContactGetNextRef(cNuke);
                b2ContactManagerDelete(p, cNuke);
                continue;
            }

            // Check user filtering.
            if (p->m_contactFilter && b2ContactFilterShouldCollide(p->m_contactFilter, fixtureA, fixtureB) == b2False)
            {
                struct b2Contact* cNuke = c;
                c = b2ContactGetNextRef(cNuke);
                b2ContactManagerDelete(p, cNuke);
                continue;
            }

            // Clear the filtering flag.
            c->m_flags &= ~b2ContactFlagFilter;
        }

        activeA = b2BodyIsAwake(bodyA) && bodyA->m_type != b2BodyTypeStatic;
        activeB = b2BodyIsAwake(bodyB) && bodyB->m_type != b2BodyTypeStatic;

        // At least one body must be awake and it must be dynamic or kinematic.
        if (activeA == b2False && activeB == b2False)
        {
            c = b2ContactGetNextRef(c);
            continue;
        }

        proxyIdA = fixtureA->m_proxies[indexA].proxyId;
        proxyIdB = fixtureB->m_proxies[indexB].proxyId;
        overlap = b2BroadPhaseTestOverlap(&p->m_broadPhase, proxyIdA, proxyIdB);

        // Here we destroy contacts that cease to overlap in the broad-phase.
        if (overlap == b2False)
        {
            struct b2Contact* cNuke = c;
            c = b2ContactGetNextRef(cNuke);
            b2ContactManagerDelete(p, cNuke);
            continue;
        }

        // The contact persists.
        b2ContactUpdate(c, p->m_contactListener);
        c = b2ContactGetNextRef(c);
    }
}
