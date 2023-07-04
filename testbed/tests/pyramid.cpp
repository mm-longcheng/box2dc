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

class Pyramid : public Test
{
public:
	enum
	{
		e_count = 20
	};

	Pyramid()
	{
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

            b2Vec2 a = { -40.0f, 0.0f };
            b2Vec2 b = { 40.0f, 0.0f };
            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
			b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
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
                    struct b2Body* body = b2WorldCreateBody(m_world, &bd);
					b2BodyCreateFixtureFromShape(body, &shape, 5.0f);

					b2Vec2Add(y, y, deltaY);
				}

                b2Vec2Add(x, x, deltaX);
			}
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		//b2DynamicTree* tree = &m_world->m_contactManager.m_broadPhase.m_tree;

		//if (m_stepCount == 400)
		//{
		//	tree->RebuildBottomUp();
		//}
	}

	static Test* Create()
	{
		return new Pyramid;
	}
};

static int testIndex = RegisterTest("Stacking", "Pyramid", Pyramid::Create);
