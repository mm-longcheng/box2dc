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
//#include "box2d/b2_time_of_impact.h"
#include "b2/mmB2TimeOfImpact.h"

class TimeOfImpact : public Test
{
public:
	TimeOfImpact()
	{
        b2ShapePolygonReset(&this->m_shapeA);
        b2ShapePolygonReset(&this->m_shapeB);

		b2ShapePolygonSetAsBox(&m_shapeA, 25.0f, 5.0f);
		b2ShapePolygonSetAsBox(&m_shapeB, 2.5f, 2.5f);
	}

	static Test* Create()
	{
		return new TimeOfImpact;
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

        struct b2Sweep sweepA;
		b2Vec2Make(sweepA.c0, 24.0f, -60.0f);
		sweepA.a0 = 2.95f;
		b2Vec2Assign(sweepA.c, sweepA.c0);
		sweepA.a = sweepA.a0;
		b2Vec2SetZero(sweepA.localCenter);

        struct b2Sweep sweepB;
		b2Vec2Make(sweepB.c0, 53.474274f, -50.252514f);
		sweepB.a0 = 513.36676f; // - 162.0f * b2_pi;
		b2Vec2Make(sweepB.c, 54.595478f, -51.083473f);
		sweepB.a = 513.62781f; //  - 162.0f * b2_pi;
		b2Vec2SetZero(sweepB.localCenter);

		//sweepB.a0 -= 300.0f * b2_pi;
		//sweepB.a -= 300.0f * b2_pi;

        struct b2TOIInput input;
        b2TOIInputReset(&input);
		b2DistanceProxySetShape(&input.proxyA, (const struct b2Shape*)&m_shapeA, 0);
		b2DistanceProxySetShape(&input.proxyB, (const struct b2Shape*)&m_shapeB, 0);
		input.sweepA = sweepA;
		input.sweepB = sweepB;
		input.tMax = 1.0f;

		b2TOIOutput output;

		b2TimeOfImpact(&output, &input);

		g_debugDraw.DrawString(5, m_textLine, "toi = %g", output.t);
		m_textLine += m_textIncrement;

		//extern B2_API int32 b2_toiMaxIters, b2_toiMaxRootIters;
		g_debugDraw.DrawString(5, m_textLine, "max toi iters = %d, max root iters = %d", b2_toiMaxIters, b2_toiMaxRootIters);
		m_textLine += m_textIncrement;

		b2Vec2 vertices[b2_maxPolygonVertices];

		b2Transform transformA;
		b2SweepGetTransform(&sweepA, transformA, 0.0f);
		for (int32 i = 0; i < m_shapeA.m_count; ++i)
		{
            b2TransformMulVec2(vertices[i], transformA, m_shapeA.m_vertices[i]);
		}
        b2Color color0 = { 0.9f, 0.9f, 0.9f, 1.0f };
        g_debugDraw.DrawPolygon(vertices, m_shapeA.m_count, color0);

		b2Transform transformB;
		b2SweepGetTransform(&sweepB, transformB, 0.0f);
		
		//b2Vec2 localPoint(2.0f, -0.1f);

		for (int32 i = 0; i < m_shapeB.m_count; ++i)
		{
            b2TransformMulVec2(vertices[i], transformB, m_shapeB.m_vertices[i]);
		}
        b2Color color1 = { 0.5f, 0.9f, 0.5f, 1.0f };
        g_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, color1);

		b2SweepGetTransform(&sweepB, transformB, output.t);
		for (int32 i = 0; i < m_shapeB.m_count; ++i)
		{
            b2TransformMulVec2(vertices[i], transformB, m_shapeB.m_vertices[i]);
		}
        b2Color color2 = { 0.5f, 0.7f, 0.9f, 1.0f };
        g_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, color2);

		b2SweepGetTransform(&sweepB, transformB, 1.0f);
		for (int32 i = 0; i < m_shapeB.m_count; ++i)
		{
            b2TransformMulVec2(vertices[i], transformB, m_shapeB.m_vertices[i]);
		}
        b2Color color3 = { 0.9f, 0.5f, 0.5f, 1.0f };
        g_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, color3);

#if 0
		for (float t = 0.0f; t < 1.0f; t += 0.1f)
		{
			b2SweepGetTransform(&sweepB, transformB, t);
			for (int32 i = 0; i < m_shapeB.m_count; ++i)
			{
                b2TransformMulVec2(vertices[i], transformB, m_shapeB.m_vertices[i]);
			}
            b2Color color4 = { 0.9f, 0.5f, 0.5f, 1.0f };
            g_debugDraw.DrawPolygon(vertices, m_shapeB.m_count, color4);
		}
#endif
	}

	struct b2ShapePolygon m_shapeA;
    struct b2ShapePolygon m_shapeB;
};

static int testIndex = RegisterTest("Collision", "Time of Impact", TimeOfImpact::Create);
