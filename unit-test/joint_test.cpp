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

DOCTEST_TEST_CASE("joint reactions")
{
    b2Vec2 gravity = { 0, -10.0f };
    struct b2World world;
    b2WorldInit(&world);
    b2WorldSetGravity(&world, gravity);

    struct b2BodyDef bodyDef;
    b2BodyDefReset(&bodyDef);
    struct b2Body* ground = b2WorldCreateBody(&world, &bodyDef);

    struct b2ShapeCircle circle;
    b2ShapeCircleReset(&circle);
    circle.m_radius = 1.0f;

    struct b2FixtureDef fixtureDef;
    b2FixtureDefReset(&fixtureDef);

	// Disable collision
	fixtureDef.filter.maskBits = 0;
	fixtureDef.density = 1.0f;
	fixtureDef.shape = (struct b2Shape*)&circle;

	bodyDef.type = b2BodyTypeDynamic;
	b2Vec2Make(bodyDef.position, -2.0f, 3.0f);

    struct b2Body* bodyA = b2WorldCreateBody(&world, &bodyDef);
    struct b2Body* bodyB = b2WorldCreateBody(&world, &bodyDef);
    struct b2Body* bodyC = b2WorldCreateBody(&world, &bodyDef);

    struct b2MassData massData;
	b2ShapeCircleComputeMass(&circle, &massData, fixtureDef.density);
	const float mg = massData.mass * gravity[1];

	b2BodyCreateFixtureFromDef(bodyA, &fixtureDef);
	b2BodyCreateFixtureFromDef(bodyB, &fixtureDef);
	b2BodyCreateFixtureFromDef(bodyC, &fixtureDef);

    b2Vec2 pos1 = { 0.0f, 4.0f };
    b2Vec2 v;
    struct b2JointDistanceDef distanceJointDef;
    b2JointDistanceDefReset(&distanceJointDef);
    b2Vec2Add(v, bodyDef.position, pos1);
	b2JointDistanceDefInitialize(&distanceJointDef, ground, bodyA, v, bodyDef.position);
	distanceJointDef.minLength = distanceJointDef.length;
	distanceJointDef.maxLength = distanceJointDef.length;

    b2Vec2 axis1 = { 1.0f, 0.0f };
    struct b2JointPrismaticDef prismaticJointDef;
    b2JointPrismaticDefReset(&prismaticJointDef);
	b2JointPrismaticDefInitialize(&prismaticJointDef, ground, bodyB, bodyDef.position, axis1);

    struct b2JointRevoluteDef revoluteJointDef;
    b2JointRevoluteDefReset(&revoluteJointDef);
	b2JointRevoluteDefInitialize(&revoluteJointDef, ground, bodyC, bodyDef.position);

    struct b2JointDistance* distanceJoint = (struct b2JointDistance*)b2WorldCreateJoint(&world, &distanceJointDef);
    struct b2JointPrismatic* prismaticJoint = (struct b2JointPrismatic*)b2WorldCreateJoint(&world, &prismaticJointDef);
    struct b2JointRevolute* revoluteJoint = (struct b2JointRevolute*)b2WorldCreateJoint(&world, &revoluteJointDef);

	const float timeStep = 1.f / 60.f;
	const float invTimeStep = 60.0f;
	const int32 velocityIterations = 6;
	const int32 positionIterations = 2;

	b2WorldStep(&world, timeStep, velocityIterations, positionIterations);

	const float tol = 1e-5f;
	{
        b2Vec2 F;
        float T;

		b2JointDistanceGetReactionForce(distanceJoint, invTimeStep, F);
		T = b2JointDistanceGetReactionTorque(distanceJoint, invTimeStep);
		CHECK(F[0] == 0.0f);
		CHECK(b2AbsFloat(F[1] + mg) < tol);
		CHECK(T == 0.0f);
	}

	{
        b2Vec2 F;
        float T;

		b2JointPrismaticGetReactionForce(prismaticJoint, invTimeStep, F);
		T = b2JointPrismaticGetReactionTorque(prismaticJoint, invTimeStep);
		CHECK(F[0] == 0.0f);
		CHECK(b2AbsFloat(F[1] + mg) < tol);
		CHECK(T == 0.0f);
	}

	{
        b2Vec2 F;
        float T;

		b2JointRevoluteGetReactionForce(revoluteJoint, invTimeStep, F);
		T = b2JointRevoluteGetReactionTorque(revoluteJoint, invTimeStep);
		CHECK(F[0] == 0.0f);
		CHECK(b2AbsFloat(F[1] + mg) < tol);
		CHECK(T == 0.0f);
	}

    b2WorldDeleteJoint(&world, (struct b2Joint*)distanceJoint);
    b2WorldDeleteJoint(&world, (struct b2Joint*)prismaticJoint);
    b2WorldDeleteJoint(&world, (struct b2Joint*)revoluteJoint);

    b2WorldDeleteBody(&world, bodyC);
    b2WorldDeleteBody(&world, bodyB);
    b2WorldDeleteBody(&world, bodyA);
    b2WorldDeleteBody(&world, ground);

    b2WorldDestroy(&world);
}
