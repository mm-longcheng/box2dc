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

class Platformer : public Test
{
public:

	enum State
	{
		e_unknown,
		e_above,
		e_below
	};

	Platformer()
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
			b2Vec2Make(bd.position, 0.0f, 10.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 3.0f, 0.5f);
			m_platform = b2BodyCreateFixtureFromShape(body, &shape, 0.0f);

			m_bottom = 10.0f - 0.5f;
			m_top = 10.0f + 0.5f;
		}

		// Actor
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 0.0f, 12.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

			m_radius = 0.5f;
            struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
			shape.m_radius = m_radius;
			m_character = b2BodyCreateFixtureFromShape(body, &shape, 20.0f);

            b2Vec2 LinearVelocity = { 0.0f, -50.0f };
			b2BodySetLinearVelocity(body, LinearVelocity);

			m_state = e_unknown;
		}
	}

	void PreSolve(struct b2Contact* contact, const struct b2Manifold* oldManifold) override
	{
		Test::PreSolve(contact, oldManifold);

        struct b2Fixture* fixtureA = b2ContactGetFixtureARef(contact);
        struct b2Fixture* fixtureB = b2ContactGetFixtureBRef(contact);

		if (fixtureA != m_platform && fixtureA != m_character)
		{
			return;
		}

		if (fixtureB != m_platform && fixtureB != m_character)
		{
			return;
		}

#if 1
		b2Vec2ConstRef position = b2BodyGetPosition(b2FixtureGetBody(m_character));

		if (position[1] < m_top + m_radius - 3.0f * b2_linearSlop)
		{
			b2ContactSetEnabled(contact, false);
		}
#else
        b2Vec2ConstRef v = b2BodyGetLinearVelocity(b2FixtureGetBody(m_character));
        if (v[1] > 0.0f)
		{
            b2ContactSetEnabled(contact, false);
        }
#endif
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		b2Vec2ConstRef v = b2BodyGetLinearVelocity(b2FixtureGetBody(m_character));
        g_debugDraw.DrawString(5, m_textLine, "Character Linear Velocity: %f", v[1]);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new Platformer;
	}

	float m_radius, m_top, m_bottom;
	State m_state;
	struct b2Fixture* m_platform;
    struct b2Fixture* m_character;
};

static int testIndex = RegisterTest("Examples", "Platformer", Platformer::Create);
