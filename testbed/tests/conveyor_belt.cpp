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

#include "test.h"

class ConveyorBelt : public Test
{
public:

	ConveyorBelt()
	{
		// Ground
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

            b2Vec2 a = { -20.0f, 0.0f };
            b2Vec2 b = { 20.0f, 0.0f };
            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
			b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

		// Platform
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, -5.0f, 5.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 10.0f, 0.5f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.friction = 0.8f;
			m_platform = b2BodyCreateFixtureFromDef(body, &fd);
		}

		// Boxes
		for (int32 i = 0; i < 5; ++i)
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, -10.0f + 2.0f * i, 7.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.5f, 0.5f);
			b2BodyCreateFixtureFromShape(body, &shape, 20.0f);
		}
	}

	void PreSolve(struct b2Contact* contact, const struct b2Manifold* oldManifold) override
	{
		Test::PreSolve(contact, oldManifold);

        struct b2Fixture* fixtureA = b2ContactGetFixtureARef(contact);
        struct b2Fixture* fixtureB = b2ContactGetFixtureBRef(contact);

		if (fixtureA == m_platform)
		{
			b2ContactSetTangentSpeed(contact, 5.0f);
		}

		if (fixtureB == m_platform)
		{
			b2ContactSetTangentSpeed(contact, -5.0f);
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new ConveyorBelt;
	}

    struct b2Fixture* m_platform;
};

static int testIndex = RegisterTest("Examples", "Conveyor Belt", ConveyorBelt::Create);
