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

class Dominos : public Test
{
public:

	Dominos()
	{
		struct b2Body* b1;
		{
            b2Vec2 a = { -40.0f, 0.0f };
            b2Vec2 b = { 40.0f, 0.0f };
            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
			b2ShapeEdgeSetTwoSided(&shape, a, b);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b1 = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(b1, &shape, 0.0f);
		}

		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 6.0f, 0.25f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, -1.5f, 10.0f);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.1f, 1.0f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 20.0f;
			fd.friction = 0.1f;

			for (int i = 0; i < 10; ++i)
			{
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, -6.0f + 1.0f * i, 11.25f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromDef(body, &fd);
			}
		}

		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBoxDetail(&shape, 7.0f, 0.25f, b2Vec2Zero, 0.3f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, 1.0f, 6.0f);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

        struct b2Body* b2;
		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.25f, 1.5f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, -7.0f, 4.0f);
			b2 = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(b2, &shape, 0.0f);
		}

        struct b2Body* b3;
		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 6.0f, 0.125f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, -0.9f, 1.0f);
			bd.angle = -0.15f;

			b3 = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(b3, &shape, 10.0f);
		}

        struct b2JointRevoluteDef jd;
        b2JointRevoluteDefReset(&jd);
		b2Vec2 anchor;

		b2Vec2Make(anchor, -2.0f, 1.0f);
		b2JointRevoluteDefInitialize(&jd, b1, b3, anchor);
		jd.collideConnected = true;
        b2WorldCreateJoint(m_world, &jd);

        struct b2Body* b4;
		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.25f, 0.25f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, -10.0f, 15.0f);
			b4 = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(b4, &shape, 10.0f);
		}

		b2Vec2Make(anchor, -7.0f, 15.0f);
		b2JointRevoluteDefInitialize(&jd, b2, b4, anchor);
        b2WorldCreateJoint(m_world, &jd);

        struct b2Body* b5;
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 6.5f, 3.0f);
			b5 = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);

			fd.shape = (struct b2Shape*)&shape;
			fd.density = 10.0f;
			fd.friction = 0.1f;

            b2Vec2 center;
            b2Vec2Make(center, 0.0f, -0.9f);
			b2ShapePolygonSetAsBoxDetail(&shape, 1.0f, 0.1f, center, 0.0f);
			b2BodyCreateFixtureFromDef(b5, &fd);

            b2Vec2Make(center, -0.9f, 0.0f);
			b2ShapePolygonSetAsBoxDetail(&shape, 0.1f, 1.0f, center, 0.0f);
			b2BodyCreateFixtureFromDef(b5, &fd);

            b2Vec2Make(center, 0.9f, 0.0f);
			b2ShapePolygonSetAsBoxDetail(&shape, 0.1f, 1.0f, center, 0.0f);
			b2BodyCreateFixtureFromDef(b5, &fd);
		}

		b2Vec2Make(anchor, 6.0f, 2.0f);
		b2JointRevoluteDefInitialize(&jd, b1, b5, anchor);
		b2WorldCreateJoint(m_world, &jd);

        struct b2Body* b6;
		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 1.0f, 0.1f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 6.5f, 4.1f);
			b6 = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(b6, &shape, 30.0f);
		}

		b2Vec2Make(anchor, 7.5f, 4.0f);
		b2JointRevoluteDefInitialize(&jd, b5, b6, anchor);
		b2WorldCreateJoint(m_world, &jd);

        struct b2Body* b7;
		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.1f, 1.0f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 7.4f, 1.0f);

			b7 = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(b7, &shape, 10.0f);
		}

        struct b2JointDistanceDef djd;
        b2JointDistanceDefReset(&djd);
		djd.bodyA = b3;
		djd.bodyB = b7;
		b2Vec2Make(djd.localAnchorA, 6.0f, 0.0f);
        b2Vec2Make(djd.localAnchorB, 0.0f, -1.0f);
        b2Vec2 v1, v2;
        b2Vec2 d;
        b2BodyGetWorldPoint(djd.bodyB, djd.localAnchorB, v1);
        b2BodyGetWorldPoint(djd.bodyA, djd.localAnchorA, v2);
        b2Vec2Sub(d, v1, v2);
		djd.length = b2Vec2Length(d);

		b2LinearStiffness(&djd.stiffness, &djd.damping, 1.0f, 1.0f, djd.bodyA, djd.bodyB);
		b2WorldCreateJoint(m_world, &djd);

		{
			float radius = 0.2f;

            struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
			shape.m_radius = radius;

			for (int32 i = 0; i < 4; ++i)
			{
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, 5.9f + 2.0f * radius * i, 2.4f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
                b2BodyCreateFixtureFromShape(body, &shape, 10.0f);
			}
		}
	}

	static Test* Create()
	{
		return new Dominos;
	}
};

static int testIndex = RegisterTest("Examples", "Dominos", Dominos::Create);
