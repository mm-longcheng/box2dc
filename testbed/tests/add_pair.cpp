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

class AddPair : public Test
{
public:

	AddPair()
	{
        b2Vec2 Gravity = { 0.0f, 0.0f };
		b2WorldSetGravity(m_world, Gravity);
		{
			struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
            b2Vec2SetZero(shape.m_p);
			shape.m_radius = 0.1f;

			float minX = -6.0f;
			float maxX = 0.0f;
			float minY = 4.0f;
			float maxY = 6.0f;
			
			for (int32 i = 0; i < 400; ++i)
			{
                b2Vec2 position = { RandomFloat(minX, maxX), RandomFloat(minY, maxY) };
				struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Assign(bd.position, position);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
                b2BodyCreateFixtureFromShape(body, &shape, 0.01f);
			}
		}
		
		{
            b2Vec2 LinearVelocity = { 10.0f, 0.0f };
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
            b2ShapePolygonSetAsBox(&shape, 1.5f, 1.5f);
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, -40.0f,5.0f);
			bd.bullet = true;
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(body, &shape, 1.0f);
			b2BodySetLinearVelocity(body, LinearVelocity);
		}
	}

	static Test* Create()
	{
		return new AddPair;
	}
};

static int testIndex = RegisterTest("Benchmark", "Add Pair", AddPair::Create);
