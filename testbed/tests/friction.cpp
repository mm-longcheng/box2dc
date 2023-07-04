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

class Friction : public Test
{
public:

	Friction()
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
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 13.0f, 0.25f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, -4.0f, 22.0f);
			bd.angle = -0.25f;

            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.25f, 1.0f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            b2Vec2Make(bd.position, 10.5f, 19.0f);

            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 13.0f, 0.25f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            b2Vec2Make(bd.position, 4.0f, 14.0f);
			bd.angle = 0.25f;

            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.25f, 1.0f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, -10.5f, 11.0f);

            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 13.0f, 0.25f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            b2Vec2Make(bd.position, -4.0f, 6.0f);
			bd.angle = -0.25f;

            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.5f, 0.5f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 25.0f;

			float friction[5] = {0.75f, 0.5f, 0.35f, 0.1f, 0.0f};

			for (int i = 0; i < 5; ++i)
			{
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, -15.0f + 4.0f * i, 28.0f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);

				fd.friction = friction[i];
				b2BodyCreateFixtureFromDef(body, &fd);
			}
		}
	}

	static Test* Create()
	{
		return new Friction;
	}
};

static int testIndex = RegisterTest("Forces", "Friction", Friction::Create);
