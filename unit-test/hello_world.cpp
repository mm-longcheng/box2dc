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
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include <stdio.h>

// This is a simple example of building and running a simulation
// using Box2D. Here we create a large ground box and a small dynamic
// box.
// There are no graphics for this example. Box2D is meant to be used
// with your rendering engine in your game engine.
DOCTEST_TEST_CASE("hello world")
{
	// Define the gravity vector.
    b2Vec2 gravity = { 0.0f, -10.0f };

	// Construct a world object, which will hold and simulate the rigid bodies.
    struct b2World world;
    b2WorldInit(&world);
    b2WorldSetGravity(&world, gravity);

	// Define the ground body.
    struct b2BodyDef groundBodyDef;
    b2BodyDefReset(&groundBodyDef);
    b2Vec2Make(groundBodyDef.position, 0.0f, -10.0f);

	// Call the body factory which allocates memory for the ground body
	// from a pool and creates the ground box shape (also from a pool).
	// The body is also added to the world.
	struct b2Body* groundBody = b2WorldCreateBody(&world, &groundBodyDef);

	// Define the ground box shape.
    struct b2ShapePolygon groundBox;
    b2ShapePolygonReset(&groundBox);

	// The extents are the half-widths of the box.
	b2ShapePolygonSetAsBox(&groundBox, 50.0f, 10.0f);

	// Add the ground fixture to the ground body.
    struct b2Fixture* fixture0 = b2BodyCreateFixtureFromShape(groundBody, &groundBox, 0.0f);

	// Define the dynamic body. We set its position and call the body factory.
    struct b2BodyDef bodyDef;
    b2BodyDefReset(&bodyDef);
	bodyDef.type = b2BodyTypeDynamic;
    b2Vec2Make(bodyDef.position, 0.0f, 4.0f);
    struct b2Body* body = b2WorldCreateBody(&world, &bodyDef);

	// Define another box shape for our dynamic body.
    struct b2ShapePolygon dynamicBox;
    b2ShapePolygonReset(&dynamicBox);
    b2ShapePolygonSetAsBox(&dynamicBox, 1.0f, 1.0f);

	// Define the dynamic body fixture.
    struct b2FixtureDef fixtureDef;
    b2FixtureDefReset(&fixtureDef);
	fixtureDef.shape = (struct b2Shape*)&dynamicBox;

	// Set the box density to be non-zero, so it will be dynamic.
	fixtureDef.density = 1.0f;

	// Override the default friction.
	fixtureDef.friction = 0.3f;

	// Add the shape to the body.
    struct b2Fixture* fixture1 = b2BodyCreateFixtureFromDef(body, &fixtureDef);

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 10 iterations. This provides a high quality simulation
	// in most game scenarios.
	float timeStep = 1.0f / 60.0f;
	int32 velocityIterations = 6;
	int32 positionIterations = 2;

    b2Vec2 position;
	b2Vec2ConstRef positionRef = b2BodyGetPosition(body);
    b2Vec2Assign(position, positionRef);
	float angle = b2BodyGetAngle(body);

	// This is our little game loop.
	for (int32 i = 0; i < 60; ++i)
	{
		// Instruct the world to perform a single step of simulation.
		// It is generally best to keep the time step and iterations fixed.
		b2WorldStep(&world, timeStep, velocityIterations, positionIterations);

		// Now print the position and angle of the body.
        positionRef = b2BodyGetPosition(body);
        b2Vec2Assign(position, positionRef);
		angle = b2BodyGetAngle(body);

		printf("%4.2f %4.2f %4.2f\n", position[0], position[1], angle);
	}

	// When the world destructor is called, all bodies and joints are freed. This can
	// create orphaned pointers, so be careful about your world management.

	CHECK(b2AbsFloat(position[0]) < 0.01f);
	CHECK(b2AbsFloat(position[1] - 1.01f) < 0.01f);
	CHECK(b2AbsFloat(angle) < 0.01f);

    //b2BodyDeleteFixture(groundBody, fixture0);
    //b2BodyDeleteFixture(body, fixture1);

    b2WorldDeleteBody(&world, body);
    b2WorldDeleteBody(&world, groundBody);

    b2WorldDestroy(&world);
}
