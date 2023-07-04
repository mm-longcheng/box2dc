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

// Test distance joints, body destruction, and joint destruction.
class Web : public Test
{
public:
	Web()
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
			b2ShapePolygonSetAsBox(&shape, 0.5f, 0.5f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;

			b2Vec2Make(bd.position, -5.0f, 5.0f);
			m_bodies[0] = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(m_bodies[0], &shape, 5.0f);

            b2Vec2Make(bd.position, 5.0f, 5.0f);
			m_bodies[1] = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(m_bodies[1], &shape, 5.0f);

            b2Vec2Make(bd.position, 5.0f, 15.0f);
			m_bodies[2] = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(m_bodies[2], &shape, 5.0f);

            b2Vec2Make(bd.position, -5.0f, 15.0f);
			m_bodies[3] = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(m_bodies[3], &shape, 5.0f);

            struct b2JointDistanceDef jd;
            b2JointDistanceDefReset(&jd);
			b2Vec2 p1, p2, d;

			float frequencyHz = 2.0f;
			float dampingRatio = 0.0f;

			jd.bodyA = ground;
			jd.bodyB = m_bodies[0];
            b2Vec2Make(jd.localAnchorA, -10.0f, 0.0f);
            b2Vec2Make(jd.localAnchorB, -0.5f, -0.5f);
			b2BodyGetWorldPoint(jd.bodyA, jd.localAnchorA, p1);
			b2BodyGetWorldPoint(jd.bodyB, jd.localAnchorB, p2);
            b2Vec2Sub(d, p2, p1);
			jd.length = b2Vec2Length(d);
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
			m_joints[0] = b2WorldCreateJoint(m_world, &jd);

			jd.bodyA = ground;
			jd.bodyB = m_bodies[1];
            b2Vec2Make(jd.localAnchorA, 10.0f, 0.0f);
            b2Vec2Make(jd.localAnchorB, 0.5f, -0.5f);
            b2BodyGetWorldPoint(jd.bodyA, jd.localAnchorA, p1);
            b2BodyGetWorldPoint(jd.bodyB, jd.localAnchorB, p2);
            b2Vec2Sub(d, p2, p1);
			jd.length = b2Vec2Length(d);
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
			m_joints[1] = b2WorldCreateJoint(m_world, &jd);

			jd.bodyA = ground;
			jd.bodyB = m_bodies[2];
            b2Vec2Make(jd.localAnchorA, 10.0f, 20.0f);
            b2Vec2Make(jd.localAnchorB, 0.5f, 0.5f);
            b2BodyGetWorldPoint(jd.bodyA, jd.localAnchorA, p1);
            b2BodyGetWorldPoint(jd.bodyB, jd.localAnchorB, p2);
            b2Vec2Sub(d, p2, p1);
			jd.length = b2Vec2Length(d);
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
			m_joints[2] = b2WorldCreateJoint(m_world, &jd);

			jd.bodyA = ground;
			jd.bodyB = m_bodies[3];
            b2Vec2Make(jd.localAnchorA, -10.0f, 20.0f);
            b2Vec2Make(jd.localAnchorB, -0.5f, 0.5f);
            b2BodyGetWorldPoint(jd.bodyA, jd.localAnchorA, p1);
            b2BodyGetWorldPoint(jd.bodyB, jd.localAnchorB, p2);
            b2Vec2Sub(d, p2, p1);
			jd.length = b2Vec2Length(d);
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
			m_joints[3] = b2WorldCreateJoint(m_world, &jd);

			jd.bodyA = m_bodies[0];
			jd.bodyB = m_bodies[1];
            b2Vec2Make(jd.localAnchorA, 0.5f, 0.0f);
            b2Vec2Make(jd.localAnchorB, -0.5f, 0.0f);
            b2BodyGetWorldPoint(jd.bodyA, jd.localAnchorA, p1);
            b2BodyGetWorldPoint(jd.bodyB, jd.localAnchorB, p2);
            b2Vec2Sub(d, p2, p1);
			jd.length = b2Vec2Length(d);
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
			m_joints[4] = b2WorldCreateJoint(m_world, &jd);

			jd.bodyA = m_bodies[1];
			jd.bodyB = m_bodies[2];
            b2Vec2Make(jd.localAnchorA, 0.0f, 0.5f);
            b2Vec2Make(jd.localAnchorB, 0.0f, -0.5f);
            b2BodyGetWorldPoint(jd.bodyA, jd.localAnchorA, p1);
            b2BodyGetWorldPoint(jd.bodyB, jd.localAnchorB, p2);
            b2Vec2Sub(d, p2, p1);
			jd.length = b2Vec2Length(d);
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
			m_joints[5] = b2WorldCreateJoint(m_world, &jd);

			jd.bodyA = m_bodies[2];
			jd.bodyB = m_bodies[3];
            b2Vec2Make(jd.localAnchorA, -0.5f, 0.0f);
            b2Vec2Make(jd.localAnchorB, 0.5f, 0.0f);
            b2BodyGetWorldPoint(jd.bodyA, jd.localAnchorA, p1);
            b2BodyGetWorldPoint(jd.bodyB, jd.localAnchorB, p2);
            b2Vec2Sub(d, p2, p1);
			jd.length = b2Vec2Length(d);
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
			m_joints[6] = b2WorldCreateJoint(m_world, &jd);

			jd.bodyA = m_bodies[3];
			jd.bodyB = m_bodies[0];
            b2Vec2Make(jd.localAnchorA, 0.0f, -0.5f);
            b2Vec2Make(jd.localAnchorB, 0.0f, 0.5f);
            b2BodyGetWorldPoint(jd.bodyA, jd.localAnchorA, p1);
            b2BodyGetWorldPoint(jd.bodyB, jd.localAnchorB, p2);
            b2Vec2Sub(d, p2, p1);
			jd.length = b2Vec2Length(d);
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
			m_joints[7] = b2WorldCreateJoint(m_world, &jd);
		}
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_B:
			for (int32 i = 0; i < 4; ++i)
			{
				if (m_bodies[i])
				{
					b2WorldDeleteBody(m_world, m_bodies[i]);
					m_bodies[i] = NULL;
					break;
				}
			}
			break;

		case GLFW_KEY_J:
			for (int32 i = 0; i < 8; ++i)
			{
				if (m_joints[i])
				{
                    b2WorldDeleteJoint(m_world, m_joints[i]);
					m_joints[i] = NULL;
					break;
				}
			}
			break;
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Press: (b) to delete a body, (j) to delete a joint");
		m_textLine += m_textIncrement;
	}

	void JointDestroyed(struct b2Joint* joint) override
	{
		for (int32 i = 0; i < 8; ++i)
		{
			if (m_joints[i] == joint)
			{
				m_joints[i] = NULL;
				break;
			}
		}
	}

	static Test* Create()
	{
		return new Web;
	}

	struct b2Body* m_bodies[4];
    struct b2Joint* m_joints[8];
};

static int testIndex = RegisterTest("Examples", "Web", Web::Create);
