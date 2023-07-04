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
//#include "box2d/b2_distance.h"
#include "b2/mmB2Distance.h"

class ShapeCast : public Test
{
public:
	enum
	{
		e_vertexCount = 8
	};

	ShapeCast()
	{
#if 1
		b2Vec2Make(m_vAs[0], -0.5f, 1.0f);
        b2Vec2Make(m_vAs[1], 0.5f, 1.0f);
        b2Vec2Make(m_vAs[2], 0.0f, 0.0f);
		m_countA = 3;
		m_radiusA = b2_polygonRadius;

        b2Vec2Make(m_vBs[0], -0.5f, -0.5f);
        b2Vec2Make(m_vBs[1], 0.5f, -0.5f);
        b2Vec2Make(m_vBs[2], 0.5f, 0.5f);
        b2Vec2Make(m_vBs[3], -0.5f, 0.5f);
		m_countB = 4;
		m_radiusB = b2_polygonRadius;

		b2Vec2Make(m_transformA[0], 0.0f, 0.25f);
        b2RotMakeIdentity(m_transformA[1]);
        b2Vec2Make(m_transformB[0], -4.0f, 0.0f);
        b2RotMakeIdentity(m_transformB[1]);
        b2Vec2Make(m_translationB, 8.0f, 0.0f);
#elif 0
		m_vAs[0].Set(0.0f, 0.0f);
		m_countA = 1;
		m_radiusA = 0.5f;

		m_vBs[0].Set(0.0f, 0.0f);
		m_countB = 1;
		m_radiusB = 0.5f;

		m_transformA.p.Set(0.0f, 0.25f);
		m_transformA.q.SetIdentity();
		m_transformB.p.Set(-4.0f, 0.0f);
		m_transformB.q.SetIdentity();
		m_translationB.Set(8.0f, 0.0f);
#else
		m_vAs[0].Set(0.0f, 0.0f);
		m_vAs[1].Set(2.0f, 0.0f);
		m_countA = 2;
		m_radiusA = b2_polygonRadius;

		m_vBs[0].Set(0.0f, 0.0f);
		m_countB = 1;
		m_radiusB = 0.25f;

		// Initial overlap
		m_transformA.p.Set(0.0f, 0.0f);
		m_transformA.q.SetIdentity();
		m_transformB.p.Set(-0.244360745f, 0.05999358f);
		m_transformB.q.SetIdentity();
		m_translationB.Set(0.0f, 0.0399999991f);
#endif
	}

	static Test* Create()
	{
		return new ShapeCast;
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		struct b2ShapeCastInput input;
        b2ShapeCastInputReset(&input);
		b2DistanceProxySetVertex(&input.proxyA, m_vAs, m_countA, m_radiusA);
		b2DistanceProxySetVertex(&input.proxyB, m_vBs, m_countB, m_radiusB);
		b2TransformAssign(input.transformA, m_transformA);
		b2TransformAssign(input.transformB, m_transformB);
		b2Vec2Assign(input.translationB, m_translationB);

        struct b2ShapeCastOutput output;
		bool hit = b2ShapeCast(&output, &input);

        b2Vec2 v;
		b2Transform transformB2;
        b2RotAssign(transformB2[1], m_transformB[1]);
        b2Vec2Scale(v, input.translationB, output.lambda);
        b2Vec2Add(transformB2[0], m_transformB[0], v);

        struct b2DistanceInput distanceInput;
        b2DistanceInputReset(&distanceInput);
		b2DistanceProxySetVertex(&distanceInput.proxyA, m_vAs, m_countA, m_radiusA);
		b2DistanceProxySetVertex(&distanceInput.proxyB, m_vBs, m_countB, m_radiusB);
        b2TransformAssign(distanceInput.transformA, m_transformA);
        b2TransformAssign(distanceInput.transformB, transformB2);
		distanceInput.useRadii = false;
        struct b2SimplexCache simplexCache;
		simplexCache.count = 0;
        struct b2DistanceOutput distanceOutput;

		b2Distance(&distanceOutput, &simplexCache, &distanceInput);

		g_debugDraw.DrawString(5, m_textLine, "hit = %s, iters = %d, lambda = %g, distance = %g",
			hit ? "true" : "false", output.iterations, output.lambda, distanceOutput.distance);
		m_textLine += m_textIncrement;

		b2Vec2 vertices[b2_maxPolygonVertices];

		for (int32 i = 0; i < m_countA; ++i)
		{
            b2TransformMulVec2(vertices[i], m_transformA, m_vAs[i]);
		}

		if (m_countA == 1)
		{
            static const b2Color color = { 0.9f, 0.9f, 0.9f, 1.0f };
			g_debugDraw.DrawCircle(vertices[0], m_radiusA, color);
		}
		else
		{
            static const b2Color color = { 0.9f, 0.9f, 0.9f, 1.0f };
			g_debugDraw.DrawPolygon(vertices, m_countA, color);
		}

		for (int32 i = 0; i < m_countB; ++i)
		{
            b2TransformMulVec2(vertices[i], m_transformB, m_vBs[i]);
		}

		if (m_countB == 1)
		{
            static const b2Color color = { 0.5f, 0.9f, 0.5f, 1.0f };
			g_debugDraw.DrawCircle(vertices[0], m_radiusB, color);
		}
		else
		{
            static const b2Color color = { 0.5f, 0.9f, 0.5f, 1.0f };
			g_debugDraw.DrawPolygon(vertices, m_countB, color);
		}

		for (int32 i = 0; i < m_countB; ++i)
		{
            b2TransformMulVec2(vertices[i], transformB2, m_vBs[i]);
		}

		if (m_countB == 1)
		{
            static const b2Color color = { 0.5f, 0.7f, 0.9f, 1.0f };
			g_debugDraw.DrawCircle(vertices[0], m_radiusB, color);
		}
		else
		{
            static const b2Color color = { 0.5f, 0.7f, 0.9f, 1.0f };
			g_debugDraw.DrawPolygon(vertices, m_countB, color);
		}

		if (hit)
		{
            static const b2Color color1 = { 0.9f, 0.3f, 0.3f, 1.0f };
            static const b2Color color2 = { 0.9f, 0.3f, 0.3f, 1.0f };
            b2Vec2 p1;
            b2Vec2Assign(p1, output.point);
			g_debugDraw.DrawPoint(p1, 10.0f, color1);
            b2Vec2 p2;
            b2Vec2Add(p2, p1, output.normal);
			g_debugDraw.DrawSegment(p1, p2, color2);
		}
	}

	b2Vec2 m_vAs[b2_maxPolygonVertices];
	int32 m_countA;
	float m_radiusA;

	b2Vec2 m_vBs[b2_maxPolygonVertices];
	int32 m_countB;
	float m_radiusB;

	b2Transform m_transformA;
	b2Transform m_transformB;
	b2Vec2 m_translationB;
};

static int testIndex = RegisterTest("Collision", "Shape Cast", ShapeCast::Create);
