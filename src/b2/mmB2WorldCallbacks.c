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

#include "mmB2WorldCallbacks.h"
#include "mmB2Fixture.h"

B2_API
void
b2DestructionListenerSayGoodbyeJoint(
    struct b2DestructionListener* p,
    struct b2Joint* joint)
{
    typedef
    void(*SayGoodbyeJoint)(
        void* obj,
        struct b2Joint* joint);

    (*((SayGoodbyeJoint)p->m->SayGoodbyeJoint))(
        p->o, 
        joint);
}

B2_API
void
b2DestructionListenerSayGoodbyeFixture(
    struct b2DestructionListener* p,
    struct b2Fixture* fixture)
{
    typedef
    void(*SayGoodbyeFixture)(
        void* obj,
        struct b2Fixture* fixture);

    (*((SayGoodbyeFixture)p->m->SayGoodbyeFixture))(
        p->o,
        fixture);
}

// Return true if contact calculations should be performed between these two shapes.
// If you implement your own collision filter you may want to build from this implementation.
static
int 
b2ContactFilterMetaDefaultShouldCollide(
    void* obj, 
    struct b2Fixture* fixtureA, 
    struct b2Fixture* fixtureB)
{
    int collide;

    const struct b2Filter* filterA = b2FixtureGetFilterData(fixtureA);
    const struct b2Filter* filterB = b2FixtureGetFilterData(fixtureB);

    if (filterA->groupIndex == filterB->groupIndex && filterA->groupIndex != 0)
    {
        return filterA->groupIndex > 0;
    }

    collide = (filterA->maskBits & filterB->categoryBits) != 0 && (filterA->categoryBits & filterB->maskBits) != 0;
    return collide;
}

B2_API const struct b2ContactFilterMeta b2ContactFilterMetaDefault =
{
    &b2ContactFilterMetaDefaultShouldCollide,
};

B2_API
int
b2ContactFilterShouldCollide(
    struct b2ContactFilter* p,
    struct b2Fixture* fixtureA,
    struct b2Fixture* fixtureB)
{
    typedef
    int(*ShouldCollide)(
        void* obj,
        struct b2Fixture* fixtureA,
        struct b2Fixture* fixtureB);

    return (*((ShouldCollide)p->m->ShouldCollide))(
        p->o,
        fixtureA,
        fixtureB);
}

static
void 
b2ContactListenerMetaDefaultBeginContact(
    void* obj,
    struct b2Contact* contact)
{
    B2_NOT_USED(obj);
    B2_NOT_USED(contact);
}

static
void 
b2ContactListenerMetaDefaultEndContact(
    void* obj, 
    struct b2Contact* contact)
{
    B2_NOT_USED(obj);
    B2_NOT_USED(contact);
}

static
void 
b2ContactListenerMetaDefaultPreSolve(
    void* obj, 
    struct b2Contact* contact, 
    const struct b2Manifold* oldManifold)
{
    B2_NOT_USED(obj);
    B2_NOT_USED(contact);
    B2_NOT_USED(oldManifold);
}

static
void 
b2ContactListenerMetaDefaultPostSolve(
    void* obj, 
    struct b2Contact* contact, 
    const struct b2ContactImpulse* impulse)
{
    B2_NOT_USED(obj);
    B2_NOT_USED(contact);
    B2_NOT_USED(impulse);
}

B2_API const struct b2ContactListenerMeta b2ContactListenerMetaDefault =
{
    &b2ContactListenerMetaDefaultBeginContact,
    &b2ContactListenerMetaDefaultEndContact,
    &b2ContactListenerMetaDefaultPreSolve,
    &b2ContactListenerMetaDefaultPostSolve,
};

B2_API
void
b2ContactListenerBeginContact(
    struct b2ContactListener* p,
    struct b2Contact* contact)
{
    typedef
    void(*BeginContact)(
        void* obj,
        struct b2Contact* contact);

    (*((BeginContact)p->m->BeginContact))(
        p->o,
        contact);
}

B2_API
void
b2ContactListenerEndContact(
    struct b2ContactListener* p,
    struct b2Contact* contact)
{
    typedef
    void(*EndContact)(
        void* obj,
        struct b2Contact* contact);

    (*((EndContact)p->m->EndContact))(
        p->o,
        contact);
}

B2_API
void
b2ContactListenerPreSolve(
    struct b2ContactListener* p,
    struct b2Contact* contact,
    const struct b2Manifold* oldManifold)
{
    typedef
    void(*PreSolve)(
        void* obj,
        struct b2Contact* contact,
        const struct b2Manifold* oldManifold);

    (*((PreSolve)p->m->PreSolve))(
        p->o,
        contact,
        oldManifold);
}

B2_API
void
b2ContactListenerPostSolve(
    struct b2ContactListener* p,
    struct b2Contact* contact,
    const struct b2ContactImpulse* impulse)
{
    typedef
    void(*PostSolve)(
        void* obj,
        struct b2Contact* contact,
        const struct b2ContactImpulse* impulse);

    (*((PostSolve)p->m->PostSolve))(
        p->o,
        contact,
        impulse);
}

B2_API
int
b2QueryCallbackReportFixture(
    struct b2QueryCallback* p,
    struct b2Fixture* fixture)
{
    typedef
    int (*ReportFixture)(
        void* obj,
        struct b2Fixture* fixture);

    return (*((ReportFixture)p->m->ReportFixture))(
        p->o,
        fixture);
}

B2_API
float
b2RayCastCallbackReportFixture(
    struct b2RayCastCallback* p,
    struct b2Fixture* fixture,
    const b2Vec2 point,
    const b2Vec2 normal,
    float fraction)
{
    typedef
    float (*ReportFixture)(
        void* obj,
        struct b2Fixture* fixture, 
        const b2Vec2 point,
        const b2Vec2 normal, 
        float fraction);

    return (*((ReportFixture)p->m->ReportFixture))(
        p->o,
        fixture,
        point,
        normal,
        fraction);
}
