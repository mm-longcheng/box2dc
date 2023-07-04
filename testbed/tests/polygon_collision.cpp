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

class PolygonCollision : public Test
{
public:
	PolygonCollision()
	{
        b2ShapePolygonReset(&this->m_polygonA);
        b2ShapePolygonReset(&this->m_polygonB);

		{
            b2Vec2 pos = { 0.0f, 0.0f };
			b2ShapePolygonSetAsBox(&m_polygonA, 0.2f, 0.4f);
            b2TransformSet(m_transformA, pos, 0.0f);
		}

		{
			b2ShapePolygonSetAsBox(&m_polygonB, 0.5f, 0.5f);
			b2Vec2Make(m_positionB, 19.345284f, 1.5632932f);
			m_angleB = 1.9160721f;
            b2TransformSet(m_transformB, m_positionB, m_angleB);
		}
	}

	static Test* Create()
	{
		return new PolygonCollision;
	}

	void Step(Settings& settings) override
	{
		B2_NOT_USED(settings);

		struct b2Manifold manifold;
		b2CollidePolygons(&manifold, &m_polygonA, m_transformA, &m_polygonB, m_transformB);

        struct b2WorldManifold worldManifold;
		b2WorldManifoldInitialize(&worldManifold, &manifold, m_transformA, m_polygonA.m_radius, m_transformB, m_polygonB.m_radius);

		g_debugDraw.DrawString(5, m_textLine, "point count = %d", manifold.pointCount);
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

		for (int32 i = 0; i < manifold.pointCount; ++i)
		{
            b2Color color = { 0.9f, 0.3f, 0.3f, 1.0f };
            g_debugDraw.DrawPoint(worldManifold.points[i], 4.0f, color);
		}

		Test::Step(settings);
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

	struct b2ShapePolygon m_polygonA;
    struct b2ShapePolygon m_polygonB;

	b2Transform m_transformA;
	b2Transform m_transformB;

	b2Vec2 m_positionB;
	float m_angleB;
};

static int testIndex = RegisterTest("Geometry", "Polygon Collision", PolygonCollision::Create);
