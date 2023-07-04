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

// It is difficult to make a cantilever made of links completely rigid with weld joints.
// You will have to use a high number of iterations to make them stiff.
// So why not go ahead and use soft weld joints? They behave like a revolute
// joint with a rotational spring.
class Cantilever : public Test
{
public:

	enum
	{
		e_count = 8
	};

	Cantilever()
	{
		struct b2Body* ground = NULL;
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			ground = b2WorldCreateBody(m_world, &bd);

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
			b2ShapePolygonSetAsBox(&shape, 0.5f, 0.125f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 20.0f;

            struct b2JointWeldDef jd;
            b2JointWeldDefReset(&jd);

            struct b2Body* prevBody = ground;
			for (int32 i = 0; i < e_count; ++i)
			{
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, -14.5f + 1.0f * i, 5.0f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromDef(body, &fd);

                b2Vec2 anchor = { -15.0f + 1.0f * i, 5.0f };
				b2JointWeldDefInitialize(&jd, prevBody, body, anchor);
				b2WorldCreateJoint(m_world, &jd);

				prevBody = body;
			}
		}

		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 1.0f, 0.125f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 20.0f;

            struct b2JointWeldDef jd;
            b2JointWeldDefReset(&jd);
			float frequencyHz = 5.0f;
			float dampingRatio = 0.7f;

            struct b2Body* prevBody = ground;
			for (int32 i = 0; i < 3; ++i)
			{
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, -14.0f + 2.0f * i, 15.0f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromDef(body, &fd);

                b2Vec2 anchor = { -15.0f + 2.0f * i, 15.0f };
				b2JointWeldDefInitialize(&jd, prevBody, body, anchor);
				b2AngularStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
                b2WorldCreateJoint(m_world, &jd);

				prevBody = body;
			}
		}

		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.5f, 0.125f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 20.0f;

            struct b2JointWeldDef jd;
            b2JointWeldDefReset(&jd);

            struct b2Body* prevBody = ground;
			for (int32 i = 0; i < e_count; ++i)
			{
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, -4.5f + 1.0f * i, 5.0f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromDef(body, &fd);

				if (i > 0)
				{
                    b2Vec2 anchor = { -5.0f + 1.0f * i, 5.0f };
					b2JointWeldDefInitialize(&jd, prevBody, body, anchor);
                    b2WorldCreateJoint(m_world, &jd);
				}

				prevBody = body;
			}
		}

		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.5f, 0.125f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 20.0f;

            struct b2JointWeldDef jd;
            b2JointWeldDefReset(&jd);
			float frequencyHz = 8.0f;
			float dampingRatio = 0.7f;

            struct b2Body* prevBody = ground;
			for (int32 i = 0; i < e_count; ++i)
			{
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, 5.5f + 1.0f * i, 10.0f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromDef(body, &fd);

				if (i > 0)
				{
                    b2Vec2 anchor = { 5.0f + 1.0f * i, 10.0f };
					b2JointWeldDefInitialize(&jd, prevBody, body, anchor);

					b2AngularStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, prevBody, body);

					b2WorldCreateJoint(m_world, &jd);
				}

				prevBody = body;
			}
		}

		for (int32 i = 0; i < 2; ++i)
		{
			b2Vec2 vertices[3];
			b2Vec2Make(vertices[0], -0.5f, 0.0f);
			b2Vec2Make(vertices[1], 0.5f, 0.0f);
			b2Vec2Make(vertices[2], 0.0f, 1.5f);

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetPoints(&shape, vertices, 3);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 1.0f;

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, -8.0f + 8.0f * i, 12.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromDef(body, &fd);
		}

		for (int32 i = 0; i < 2; ++i)
		{
            struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
			shape.m_radius = 0.5f;

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 1.0f;

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, -6.0f + 6.0f * i, 10.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromDef(body, &fd);
		}
	}

	static Test* Create()
	{
		return new Cantilever;
	}

    struct b2Body* m_middle;
};

static int testIndex = RegisterTest("Joints", "Cantilever", Cantilever::Create);
