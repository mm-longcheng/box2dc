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

// This is used to test sensor shapes.
class Breakable : public Test
{
public:

	enum
	{
		e_count = 7
	};

	Breakable()
	{
        b2ShapePolygonReset(&this->m_shape1);
        b2ShapePolygonReset(&this->m_shape2);

		// Ground body
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

            b2Vec2 a;
            b2Vec2 b;
            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
            b2Vec2Make(a, -40.0f, 0.0f);
            b2Vec2Make(b, 40.0f, 0.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

		// Breakable dynamic body
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 0.0f, 40.0f);
			bd.angle = 0.25f * b2_pi;
			m_body1 = b2WorldCreateBody(m_world, &bd);

            b2Vec2 center1;
            b2Vec2 center2;

            b2Vec2Make(center1, -0.5f, 0.0f);
            b2ShapePolygonSetAsBoxDetail(&m_shape1, 0.5f, 0.5f, center1, 0.0f);
			m_piece1 = b2BodyCreateFixtureFromShape(m_body1, &m_shape1, 1.0f);

            b2Vec2Make(center2, 0.5f, 0.0f);
			b2ShapePolygonSetAsBoxDetail(&m_shape2, 0.5f, 0.5f, center2, 0.0f);
			m_piece2 = b2BodyCreateFixtureFromShape(m_body1, &m_shape2, 1.0f);
		}

		m_break = false;
		m_broke = false;
	}

	void PostSolve(struct b2Contact* contact, const struct b2ContactImpulse* impulse) override
	{
		if (m_broke)
		{
			// The body already broke.
			return;
		}

		// Should the body break?
		int32 count = b2ContactGetManifold(contact)->pointCount;

		float maxImpulse = 0.0f;
		for (int32 i = 0; i < count; ++i)
		{
			maxImpulse = b2MaxFloat(maxImpulse, impulse->normalImpulses[i]);
		}

		if (maxImpulse > 40.0f)
		{
			// Flag the body for breaking.
			m_break = true;
		}
	}

	void Break()
	{
		// Create two bodies from one.
		struct b2Body* body1 = b2FixtureGetBodyRef(m_piece1);
		b2Vec2ConstRef center = b2BodyGetWorldCenter(body1);

		b2BodyDeleteFixture(body1, m_piece2);
		m_piece2 = NULL;

        struct b2BodyDef bd;
        b2BodyDefReset(&bd);
		bd.type = b2BodyTypeDynamic;
		b2Vec2Assign(bd.position, b2BodyGetPosition(body1));
		bd.angle = b2BodyGetAngle(body1);

        struct b2Body* body2 = b2WorldCreateBody(m_world, &bd);
		m_piece2 = b2BodyCreateFixtureFromShape(body2, &m_shape2, 1.0f);

		// Compute consistent velocities for new bodies based on
		// cached velocity.
		b2Vec2ConstRef center1 = b2BodyGetWorldCenter(body1);
        b2Vec2ConstRef center2 = b2BodyGetWorldCenter(body2);
		
        b2Vec2 v;
        b2Vec2 velocity1;
        b2Vec2 velocity2;
        b2Vec2Sub(v, center1, center);
        b2Vec2CrossProductKL(v, m_angularVelocity, v);
        b2Vec2Add(velocity1, m_velocity, v);
        b2Vec2Sub(v, center2, center);
        b2Vec2CrossProductKL(v, m_angularVelocity, v);
        b2Vec2Add(velocity2, m_velocity, v);

		b2BodySetAngularVelocity(body1, m_angularVelocity);
		b2BodySetLinearVelocity(body1, velocity1);

		b2BodySetAngularVelocity(body2, m_angularVelocity);
		b2BodySetLinearVelocity(body2, velocity2);
	}

	void Step(Settings& settings) override
	{
		if (m_break)
		{
			Break();
			m_broke = true;
			m_break = false;
		}

		// Cache velocities to improve movement on breakage.
		if (m_broke == false)
		{
            b2Vec2Assign(m_velocity, b2BodyGetLinearVelocity(m_body1));
            m_angularVelocity = b2BodyGetAngularVelocity(m_body1);
		}

		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Breakable;
	}

    struct b2Body* m_body1;
	b2Vec2 m_velocity;
	float m_angularVelocity;
	struct b2ShapePolygon m_shape1;
    struct b2ShapePolygon m_shape2;
    struct b2Fixture* m_piece1;
    struct b2Fixture* m_piece2;

	bool m_broke;
	bool m_break;
};

static int testIndex = RegisterTest("Examples", "Breakable", Breakable::Create);
