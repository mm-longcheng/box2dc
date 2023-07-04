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

#include "b2/mmB2Distance.h"
#include "b2/mmB2TimeOfImpact.h"

class BulletTest : public Test
{
public:

	BulletTest()
	{
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, 0.0f, 0.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            b2Vec2 a;
            b2Vec2 b;
            struct b2ShapeEdge edge;
            b2ShapeEdgeReset(&edge);
            b2Vec2Make(a, -10.0f, 0.0f);
            b2Vec2Make(b, 10.0f, 0.0f);
            b2ShapeEdgeSetTwoSided(&edge, a, b);
			b2BodyCreateFixtureFromShape(body, &edge, 0.0f);

            b2Vec2 center;
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
            b2Vec2Make(center, 0.5f, 1.0f);
			b2ShapePolygonSetAsBoxDetail(&shape, 0.2f, 1.0f, center, 0.0f);
            b2BodyCreateFixtureFromShape(body, &shape, 0.0f);
		}

		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 0.0f, 4.0f);

            struct b2ShapePolygon box;
            b2ShapePolygonReset(&box);
			b2ShapePolygonSetAsBox(&box, 2.0f, 0.1f);

			m_body = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(m_body, &box, 1.0f);

			b2ShapePolygonSetAsBox(&box, 0.25f, 0.25f);

			//m_x = RandomFloat(-1.0f, 1.0f);
			m_x = 0.20352793f;
			b2Vec2Make(bd.position, m_x, 10.0f);
			bd.bullet = true;

			m_bullet = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(m_bullet, &box, 100.0f);

            b2Vec2 LinearVelocity = { 0.0f, -50.0f };
            b2BodySetLinearVelocity(m_bullet, LinearVelocity);
		}
	}

	void Launch()
	{
        b2Vec2 position1 = { 0.0f, 4.0f };
		b2BodySetTransform(m_body, position1, 0.0f);
		b2BodySetLinearVelocity(m_body, b2Vec2Zero);
		b2BodySetAngularVelocity(m_body, 0.0f);

		m_x = RandomFloat(-1.0f, 1.0f);
        b2Vec2 position2 = { m_x, 10.0f };
		b2BodySetTransform(m_bullet, position2, 0.0f);
        b2Vec2 LinearVelocity2 = { 0.0f, -50.0f };
		b2BodySetLinearVelocity(m_bullet, LinearVelocity2);
		b2BodySetAngularVelocity(m_bullet, 0.0f);

		//extern MM_EXPORT_B2 int32 b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;
		//extern MM_EXPORT_B2 int32 b2_toiCalls, b2_toiIters, b2_toiMaxIters;
		//extern MM_EXPORT_B2 int32 b2_toiRootIters, b2_toiMaxRootIters;

		b2_gjkCalls = 0;
		b2_gjkIters = 0;
		b2_gjkMaxIters = 0;

		b2_toiCalls = 0;
		b2_toiIters = 0;
		b2_toiMaxIters = 0;
		b2_toiRootIters = 0;
		b2_toiMaxRootIters = 0;
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		//extern MM_EXPORT_B2 int32 b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;
		//extern MM_EXPORT_B2 int32 b2_toiCalls, b2_toiIters;
		//extern MM_EXPORT_B2 int32 b2_toiRootIters, b2_toiMaxRootIters;

		if (b2_gjkCalls > 0)
		{
			g_debugDraw.DrawString(5, m_textLine, "gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
				b2_gjkCalls, b2_gjkIters / float(b2_gjkCalls), b2_gjkMaxIters);
			m_textLine += m_textIncrement;
		}

		if (b2_toiCalls > 0)
		{
			g_debugDraw.DrawString(5, m_textLine, "toi calls = %d, ave toi iters = %3.1f, max toi iters = %d",
				b2_toiCalls, b2_toiIters / float(b2_toiCalls), b2_toiMaxRootIters);
			m_textLine += m_textIncrement;

			g_debugDraw.DrawString(5, m_textLine, "ave toi root iters = %3.1f, max toi root iters = %d",
				b2_toiRootIters / float(b2_toiCalls), b2_toiMaxRootIters);
			m_textLine += m_textIncrement;
		}

		if (m_stepCount % 60 == 0)
		{
			Launch();
		}
	}

	static Test* Create()
	{
		return new BulletTest;
	}

    struct b2Body* m_body;
    struct b2Body* m_bullet;
	float m_x;
};

static int testIndex = RegisterTest("Continuous", "Bullet Test", BulletTest::Create);
