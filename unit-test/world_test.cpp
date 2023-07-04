// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

//#include "box2d/box2d.h"
#include "b2/mmB2Header.h"
#include "doctest.h"
#include <stdio.h>

static bool begin_contact = false;

class MyContactListener;

static
void
MyContactListenerMetaBeginContact(
    void* obj,
    struct b2Contact* contact)
{
    begin_contact = true;
}

static
void
MyContactListenerMetaEndContact(
    void* obj,
    struct b2Contact* contact)
{
    B2_NOT_USED(obj);
    B2_NOT_USED(contact);
}

static
void
MyContactListenerMetaPreSolve(
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
MyContactListenerMetaPostSolve(
    void* obj,
    struct b2Contact* contact,
    const struct b2ContactImpulse* impulse)
{
    B2_NOT_USED(obj);
    B2_NOT_USED(contact);
    B2_NOT_USED(impulse);
}

const struct b2ContactListenerMeta b2ContactListenerMetaMy = 
{
    &MyContactListenerMetaBeginContact,
    &MyContactListenerMetaEndContact,
    &MyContactListenerMetaPreSolve,
    &MyContactListenerMetaPostSolve,
};

class MyContactListener : public b2ContactListener
{
public:
    MyContactListener(void)
    {
        this->m = &b2ContactListenerMetaMy;
        this->o = this;
    }
};

DOCTEST_TEST_CASE("begin contact")
{
    b2Vec2 gravity = { 0, -10.0f };

    struct b2World world;
    b2WorldInit(&world);
    b2WorldSetGravity(&world, gravity);
	MyContactListener listener;
	b2WorldSetContactListener(&world, &listener);

    struct b2ShapeCircle circle;
    b2ShapeCircleReset(&circle);
    circle.m_radius = 5.f;

    struct b2BodyDef bodyDef;
    b2BodyDefReset(&bodyDef);
	bodyDef.type = b2BodyTypeDynamic;

    struct b2Body* bodyA = b2WorldCreateBody(&world, &bodyDef);
    struct b2Body* bodyB = b2WorldCreateBody(&world, &bodyDef);
    b2BodyCreateFixtureFromShape(bodyA, &circle, 0.0f);
    b2BodyCreateFixtureFromShape(bodyB, &circle, 0.0f);

    b2Vec2 pos1 = { 0.f, 0.f };
    b2Vec2 pos2 = { 100.f, 0.f };
    b2BodySetTransform(bodyA, pos1, 0.f);
	b2BodySetTransform(bodyB, pos2, 0.f);

	const float timeStep = 1.f / 60.f;
	const int32 velocityIterations = 6;
	const int32 positionIterations = 2;

	b2WorldStep(&world, timeStep, velocityIterations, positionIterations);

	CHECK(b2WorldGetContactList(&world) == nullptr);
	CHECK(begin_contact == false);
	
    b2Vec2 pos3 = { 1.f, 0.f };
	b2BodySetTransform(bodyB, pos3, 0.f);

	b2WorldStep(&world, timeStep, velocityIterations, positionIterations);

	CHECK(b2WorldGetContactList(&world) != nullptr);
	CHECK(begin_contact == true);

    b2WorldDeleteBody(&world, bodyB);
    b2WorldDeleteBody(&world, bodyA);

    b2WorldDestroy(&world);
}
