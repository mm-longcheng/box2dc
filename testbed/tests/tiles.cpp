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

/// This stress tests the dynamic tree broad-phase. This also shows that tile
/// based collision is _not_ smooth due to Box2D not knowing about adjacency.
class Tiles : public Test
{
public:
	enum
	{
		e_count = 20
	};

	Tiles()
	{
		m_fixtureCount = 0;
        struct b2Timer timer;

        b2TimerMake(&timer);

		{
			float a = 0.5f;
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.position[1] = -a;
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

#if 1
			int32 N = 200;
			int32 M = 10;
			b2Vec2 position;
			position[1] = 0.0f;
			for (int32 j = 0; j < M; ++j)
			{
				position[0] = -N * a;
				for (int32 i = 0; i < N; ++i)
				{
                    struct b2ShapePolygon shape;
                    b2ShapePolygonReset(&shape);
					b2ShapePolygonSetAsBoxDetail(&shape, a, a, position, 0.0f);
					b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
					++m_fixtureCount;
					position[0] += 2.0f * a;
				}
				position[1] -= 2.0f * a;
			}
#else
			int32 N = 200;
			int32 M = 10;
			b2Vec2 position;
			position[0] = -N * a;
			for (int32 i = 0; i < N; ++i)
			{
				position[1] = 0.0f;
				for (int32 j = 0; j < M; ++j)
				{
					struct b2ShapePolygon shape;
					b2ShapePolygonSetAsBoxDetail(&shape, a, a, position, 0.0f);
					b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
					position[1] -= 2.0f * a;
				}
				position[0] += 2.0f * a;
			}
#endif
		}

		{
			float a = 0.5f;
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, a, a);

            b2Vec2 x = { -7.0f, 0.75f };
			b2Vec2 y;
            b2Vec2 deltaX = { 0.5625f, 1.25f };
            b2Vec2 deltaY = { 1.125f, 0.0f };

			for (int32 i = 0; i < e_count; ++i)
			{
				b2Vec2Assign(y, x);

				for (int32 j = i; j < e_count; ++j)
				{
                    struct b2BodyDef bd;
                    b2BodyDefReset(&bd);
					bd.type = b2BodyTypeDynamic;
					b2Vec2Assign(bd.position, y);

					//if (i == 0 && j == 0)
					//{
					//	bd.allowSleep = false;
					//}
					//else
					//{
					//	bd.allowSleep = true;
					//}

                    struct b2Body* body = b2WorldCreateBody(m_world, &bd);
                    b2BodyCreateFixtureFromShape(body, &shape, 5.0f);
					++m_fixtureCount;
					b2Vec2Add(y, y, deltaY);
				}

                b2Vec2Add(x, x, deltaX);
			}
		}

		m_createTime = b2TimerGetMilliseconds(&timer);
	}

	void Step(Settings& settings) override
	{
		const struct b2ContactManager* cm = b2WorldGetContactManager(m_world);
		int32 height = b2BroadPhaseGetTreeHeight(&cm->m_broadPhase);
		int32 leafCount = b2BroadPhaseGetProxyCount(&cm->m_broadPhase);
		int32 minimumNodeCount = 2 * leafCount - 1;
		float minimumHeight = ceilf(logf(float(minimumNodeCount)) / logf(2.0f));
		g_debugDraw.DrawString(5, m_textLine, "dynamic tree height = %d, min = %d", height, int32(minimumHeight));
		m_textLine += m_textIncrement;

		Test::Step(settings);

		g_debugDraw.DrawString(5, m_textLine, "create time = %6.2f ms, fixture count = %d",
			m_createTime, m_fixtureCount);
		m_textLine += m_textIncrement;

		//b2DynamicTree* tree = &m_world->m_contactManager.m_broadPhase.m_tree;

		//if (m_stepCount == 400)
		//{
		//	tree->RebuildBottomUp();
		//}
	}

	static Test* Create()
	{
		return new Tiles;
	}

	int32 m_fixtureCount;
	float m_createTime;
};

static int testIndex = RegisterTest("Benchmark", "Tiles", Tiles::Create);
