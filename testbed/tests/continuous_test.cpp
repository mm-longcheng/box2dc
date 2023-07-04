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

class ContinuousTest : public Test
{
public:

	ContinuousTest()
	{
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, 0.0f, 0.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeEdge edge;
            b2ShapeEdgeReset(&edge);

            b2Vec2 a = { -10.0f, 0.0f };
            b2Vec2 b = { 10.0f, 0.0f };

			b2ShapeEdgeSetTwoSided(&edge, a, b);
			b2BodyCreateFixtureFromShape(body, &edge, 0.0f);

            b2Vec2 center = { 0.5f, 1.0f };
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBoxDetail(&shape, 0.2f, 1.0f, center, 0.0f);
			b2BodyCreateFixtureFromShape(body, &shape, 0.0f);
		}

#if 1
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 0.0f, 20.0f);
			//bd.angle = 0.1f;

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 2.0f, 0.1f);

			m_body = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(m_body, &shape, 1.0f);

			m_angularVelocity = RandomFloat(-50.0f, 50.0f);
			//m_angularVelocity = 46.661274f;
            b2Vec2 LinearVelocity = { 0.0f, -100.0f };
            b2BodySetLinearVelocity(m_body, LinearVelocity);
			b2BodySetAngularVelocity(m_body, m_angularVelocity);
		}
#else
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 0.0f, 2.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
			b2Vec2SetZero(shape.m_p);
			shape.m_radius = 0.5f;
			b2BodyCreateFixtureFromShape(body, &shape, 1.0f);

			bd.bullet = true;
			b2Vec2Make(bd.position, 0.0f, 10.0f);
			body = b2WorldCreateBody(m_world, &bd);
            b2BodyCreateFixtureFromShape(body, &shape, 1.0f);
            b2Vec2 LinearVelocity = { 0.0f, -100.0f };
			b2BodySetLinearVelocity(body, LinearVelocity);
		}
#endif

		//extern MM_EXPORT_B2 int32 b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;
		//extern MM_EXPORT_B2 int32 b2_toiCalls, b2_toiIters;
		//extern MM_EXPORT_B2 int32 b2_toiRootIters, b2_toiMaxRootIters;
		//extern MM_EXPORT_B2 float b2_toiTime, b2_toiMaxTime;

		b2_gjkCalls = 0; b2_gjkIters = 0; b2_gjkMaxIters = 0;
		b2_toiCalls = 0; b2_toiIters = 0;
		b2_toiRootIters = 0; b2_toiMaxRootIters = 0;
		b2_toiTime = 0.0f; b2_toiMaxTime = 0.0f;
	}

	void Launch()
	{
		//extern MM_EXPORT_B2 int32 b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;
		//extern MM_EXPORT_B2 int32 b2_toiCalls, b2_toiIters;
		//extern MM_EXPORT_B2 int32 b2_toiRootIters, b2_toiMaxRootIters;
		//extern MM_EXPORT_B2 float b2_toiTime, b2_toiMaxTime;

		b2_gjkCalls = 0; b2_gjkIters = 0; b2_gjkMaxIters = 0;
		b2_toiCalls = 0; b2_toiIters = 0;
		b2_toiRootIters = 0; b2_toiMaxRootIters = 0;
		b2_toiTime = 0.0f; b2_toiMaxTime = 0.0f;

        b2Vec2 pos1 = { 0.0f, 20.0f };
		b2BodySetTransform(m_body, pos1, 0.0f);
		m_angularVelocity = RandomFloat(-50.0f, 50.0f);
        b2Vec2 LinearVelocity = { 0.0f, -100.0f };
        b2BodySetLinearVelocity(m_body, LinearVelocity);
        b2BodySetAngularVelocity(m_body, m_angularVelocity);
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		//extern MM_EXPORT_B2 int32 b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;

		if (b2_gjkCalls > 0)
		{
			g_debugDraw.DrawString(5, m_textLine, "gjk calls = %d, ave gjk iters = %3.1f, max gjk iters = %d",
				b2_gjkCalls, b2_gjkIters / float(b2_gjkCalls), b2_gjkMaxIters);
			m_textLine += m_textIncrement;
		}

		//extern MM_EXPORT_B2 int32 b2_toiCalls, b2_toiIters;
		//extern MM_EXPORT_B2 int32 b2_toiRootIters, b2_toiMaxRootIters;
		//extern MM_EXPORT_B2 float b2_toiTime, b2_toiMaxTime;

		if (b2_toiCalls > 0)
		{
			g_debugDraw.DrawString(5, m_textLine, "toi calls = %d, ave [max] toi iters = %3.1f [%d]",
								b2_toiCalls, b2_toiIters / float(b2_toiCalls), b2_toiMaxRootIters);
			m_textLine += m_textIncrement;
			
			g_debugDraw.DrawString(5, m_textLine, "ave [max] toi root iters = %3.1f [%d]",
				b2_toiRootIters / float(b2_toiCalls), b2_toiMaxRootIters);
			m_textLine += m_textIncrement;

			g_debugDraw.DrawString(5, m_textLine, "ave [max] toi time = %.1f [%.1f] (microseconds)",
				1000.0f * b2_toiTime / float(b2_toiCalls), 1000.0f * b2_toiMaxTime);
			m_textLine += m_textIncrement;
		}

		if (m_stepCount % 60 == 0)
		{
			//Launch();
		}
	}

	static Test* Create()
	{
		return new ContinuousTest;
	}

	struct b2Body* m_body;
	float m_angularVelocity;
};

static int testIndex = RegisterTest("Continuous", "Continuous Test", ContinuousTest::Create);
