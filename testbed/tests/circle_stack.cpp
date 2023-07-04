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

class CircleStack : public Test
{
public:

	enum
	{
		e_count = 10
	};

	CircleStack()
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
            struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
			shape.m_radius = 1.0f;

			for (int32 i = 0; i < e_count; ++i)
			{
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, 0.0, 4.0f + 3.0f * i);

				m_bodies[i] = b2WorldCreateBody(m_world, &bd);

                b2BodyCreateFixtureFromShape(m_bodies[i], &shape, 1.0f);

                b2Vec2 LinearVelocity = { 0.0f, -50.0f };
				b2BodySetLinearVelocity(m_bodies[i], LinearVelocity);
			}
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		//for (int32 i = 0; i < e_count; ++i)
		//{
		//	printf("%g ", m_bodies[i]->GetWorldCenter().y);
		//}

		//for (int32 i = 0; i < e_count; ++i)
		//{
		//	printf("%g ", m_bodies[i]->GetLinearVelocity().y);
		//}

		//printf("\n");
	}

	static Test* Create()
	{
		return new CircleStack;
	}

	struct b2Body* m_bodies[e_count];
};

static int testIndex = RegisterTest("Stacking", "Circles", CircleStack::Create);
