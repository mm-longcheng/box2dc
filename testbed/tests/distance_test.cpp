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

class DistanceTest : public Test
{
public:
	DistanceTest()
	{
        b2ShapePolygonReset(&this->m_polygonA);
        b2ShapePolygonReset(&this->m_polygonB);

		{
			b2TransformMakeIdentity(m_transformA);
			b2Vec2Make(m_transformA[0], 0.0f, -0.2f);
			b2ShapePolygonSetAsBox(&m_polygonA, 10.0f, 0.2f);
		}

		{
			b2Vec2Make(m_positionB, 12.017401f, 0.13678508f);
			m_angleB = -0.0109265f;
            b2TransformSet(m_transformB, m_positionB, m_angleB);

			b2ShapePolygonSetAsBox(&m_polygonB, 2.0f, 0.1f);
		}
	}

	static Test* Create()
	{
		return new DistanceTest;
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		struct b2DistanceInput input;
        b2DistanceInputReset(&input);
		b2DistanceProxySetShape(&input.proxyA, (const struct b2Shape*)&m_polygonA, 0);
		b2DistanceProxySetShape(&input.proxyB, (const struct b2Shape*)&m_polygonB, 0);
		b2TransformAssign(input.transformA, m_transformA);
		b2TransformAssign(input.transformB, m_transformB);
		input.useRadii = true;
        struct b2SimplexCache cache;
		cache.count = 0;
        struct b2DistanceOutput output;
		b2Distance(&output, &cache, &input);

		g_debugDraw.DrawString(5, m_textLine, "distance = %g", output.distance);
		m_textLine += m_textIncrement;

		g_debugDraw.DrawString(5, m_textLine, "iterations = %d", output.iterations);
		m_textLine += m_textIncrement;

		{
            b2Color color = { 0.9f, 0.9f, 0.9f, 1.0f };
			b2Vec2 v[b2_maxPolygonVertices];
			for (int32 i = 0; i < m_polygonA.m_count; ++i)
			{
                b2TransformMulVec2(v[i], m_transformA, m_polygonA.m_vertices[i]);
			}
            g_debugDraw.DrawPolygon(v, m_polygonA.m_count, color);

			for (int32 i = 0; i < m_polygonB.m_count; ++i)
			{
                b2TransformMulVec2(v[i], m_transformB, m_polygonB.m_vertices[i]);
			}
            g_debugDraw.DrawPolygon(v, m_polygonB.m_count, color);
		}

        b2Vec2 x1;
        b2Vec2 x2;

        b2Vec2Assign(x1, output.pointA);
        b2Vec2Assign(x2, output.pointB);

        b2Color c1 = { 1.0f, 0.0f, 0.0f, 1.0f };
        g_debugDraw.DrawPoint(x1, 4.0f, c1);

        b2Color c2 = { 1.0f, 1.0f, 0.0f, 1.0f };
        g_debugDraw.DrawPoint(x2, 4.0f, c2);
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_positionB[0] -= 0.1f;
			break;

		case GLFW_KEY_D:
			m_positionB[0] += 0.1f;
			break;

		case GLFW_KEY_S:
			m_positionB[1] -= 0.1f;
			break;

		case GLFW_KEY_W:
			m_positionB[1] += 0.1f;
			break;

		case GLFW_KEY_Q:
			m_angleB += 0.1f * b2_pi;
			break;

		case GLFW_KEY_E:
			m_angleB -= 0.1f * b2_pi;
			break;
		}

        b2TransformSet(m_transformB, m_positionB, m_angleB);
	}

	b2Vec2 m_positionB;
	float m_angleB;

	b2Transform m_transformA;
	b2Transform m_transformB;
	struct b2ShapePolygon m_polygonA;
    struct b2ShapePolygon m_polygonB;
};

static int testIndex = RegisterTest("Geometry", "Distance Test", DistanceTest::Create);
