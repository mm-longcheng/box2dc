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

/// This is a test of typical character collision scenarios. This does not
/// show how you should implement a character in your application.
/// Instead this is used to test smooth collision on edge chains.
class CharacterCollision : public Test
{
public:
	CharacterCollision()
	{
		// Ground body
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

            b2Vec2 a = { -20.0f, 0.0f };
            b2Vec2 b = { 20.0f, 0.0f };
            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
			b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

		// Collinear edges with no adjacency information.
		// This shows the problematic case where a box shape can hit
		// an internal vertex.
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

            b2Vec2 a;
            b2Vec2 b;
            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
            b2Vec2Make(a, -8.0f, 1.0f);
            b2Vec2Make(b, -6.0f, 1.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
            b2Vec2Make(a, -6.0f, 1.0f);
            b2Vec2Make(b, -4.0f, 1.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
            b2Vec2Make(a, -4.0f, 1.0f);
            b2Vec2Make(b, -2.0f, 1.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

		// Chain shape
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.angle = 0.25f * b2_pi;
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

			b2Vec2 vs[4];
			b2Vec2Make(vs[0], 5.0f, 7.0f);
			b2Vec2Make(vs[1], 6.0f, 8.0f);
			b2Vec2Make(vs[2], 7.0f, 8.0f);
			b2Vec2Make(vs[3], 8.0f, 7.0f);
            struct b2ShapeChain shape;
            b2ShapeChainReset(&shape);
			b2ShapeChainCreateLoop(&shape, vs, 4);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
            b2ShapeChainClear(&shape);
		}

		// Square tiles. This shows that adjacency shapes may
		// have non-smooth collision. There is no solution
		// to this problem.
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

            b2Vec2 center;
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
            b2Vec2Make(center, 4.0f, 3.0f);
			b2ShapePolygonSetAsBoxDetail(&shape, 1.0f, 1.0f, center, 0.0f);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
            b2Vec2Make(center, 6.0f, 3.0f);
            b2ShapePolygonSetAsBoxDetail(&shape, 1.0f, 1.0f, center, 0.0f);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
            b2Vec2Make(center, 8.0f, 3.0f);
            b2ShapePolygonSetAsBoxDetail(&shape, 1.0f, 1.0f, center, 0.0f);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
        }

		// Square made from an edge loop. Collision should be smooth.
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

			b2Vec2 vs[4];
            b2Vec2Make(vs[0], -1.0f, 3.0f);
            b2Vec2Make(vs[1], 1.0f, 3.0f);
            b2Vec2Make(vs[2], 1.0f, 5.0f);
            b2Vec2Make(vs[3], -1.0f, 5.0f);
            struct b2ShapeChain shape;
            b2ShapeChainReset(&shape);
			b2ShapeChainCreateLoop(&shape, vs, 4);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
            b2ShapeChainClear(&shape);
		}

		// Edge loop. Collision should be smooth.
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, -10.0f, 4.0f);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

			b2Vec2 vs[10];
            b2Vec2Make(vs[0], 0.0f, 0.0f);
            b2Vec2Make(vs[1], 6.0f, 0.0f);
            b2Vec2Make(vs[2], 6.0f, 2.0f);
            b2Vec2Make(vs[3], 4.0f, 1.0f);
            b2Vec2Make(vs[4], 2.0f, 2.0f);
            b2Vec2Make(vs[5], 0.0f, 2.0f);
            b2Vec2Make(vs[6], -2.0f, 2.0f);
            b2Vec2Make(vs[7], -4.0f, 3.0f);
            b2Vec2Make(vs[8], -6.0f, 2.0f);
            b2Vec2Make(vs[9], -6.0f, 0.0f);
            struct b2ShapeChain shape;
            b2ShapeChainReset(&shape);
			b2ShapeChainCreateLoop(&shape, vs, 10);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
            b2ShapeChainClear(&shape);
		}

		// Square character 1
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            b2Vec2Make(bd.position, -3.0f, 8.0f);
			bd.type = b2BodyTypeDynamic;
			bd.fixedRotation = true;
			bd.allowSleep = false;

            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.5f, 0.5f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 20.0f;
			b2BodyCreateFixtureFromDef(body, &fd);
		}

		// Square character 2
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            b2Vec2Make(bd.position, -5.0f, 5.0f);
			bd.type = b2BodyTypeDynamic;
			bd.fixedRotation = true;
			bd.allowSleep = false;

            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.25f, 0.25f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 20.0f;
			b2BodyCreateFixtureFromDef(body, &fd);
		}

		// Hexagon character
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, -5.0f, 8.0f);
			bd.type = b2BodyTypeDynamic;
			bd.fixedRotation = true;
			bd.allowSleep = false;

            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

			float angle = 0.0f;
			float delta = b2_pi / 3.0f;
			b2Vec2 vertices[6];
			for (int32 i = 0; i < 6; ++i)
			{
				b2Vec2Make(vertices[i], 0.5f * cosf(angle), 0.5f * sinf(angle));
				angle += delta;
			}

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetPoints(&shape, vertices, 6);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 20.0f;
            b2BodyCreateFixtureFromDef(body, &fd);
		}

		// Circle character
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, 3.0f, 5.0f);
			bd.type = b2BodyTypeDynamic;
			bd.fixedRotation = true;
			bd.allowSleep = false;

            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
			shape.m_radius = 0.5f;

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 20.0f;
            b2BodyCreateFixtureFromDef(body, &fd);
		}

		// Circle character
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, -7.0f, 6.0f);
			bd.type = b2BodyTypeDynamic;
			bd.allowSleep = false;

			m_character = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
			shape.m_radius = 0.25f;

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 20.0f;
			fd.friction = 1.0f;
			b2BodyCreateFixtureFromDef(m_character, &fd);
		}
	}

	void Step(Settings& settings) override
	{
        b2Vec2ConstRef vref;
        b2Vec2 v;
        vref = b2BodyGetLinearVelocity(m_character);
        b2Vec2Assign(v, vref);
		v[0] = -5.0f;
		b2BodySetLinearVelocity(m_character, v);

		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "This tests various character collision shapes.");
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "Limitation: square and hexagon can snag on aligned boxes.");
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "Feature: edge chains have smooth collision inside and out.");
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new CharacterCollision;
	}

	struct b2Body* m_character;
};

static int testIndex = RegisterTest("Examples", "Character Collision", CharacterCollision::Create);
