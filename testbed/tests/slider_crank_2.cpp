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

#include "settings.h"
#include "test.h"

// A motor driven slider crank with joint friction.

class SliderCrank2 : public Test
{
public:
	SliderCrank2()
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
            struct b2Body* prevBody = ground;

			// Define crank.
			{
                struct b2ShapePolygon shape;
                b2ShapePolygonReset(&shape);
				b2ShapePolygonSetAsBox(&shape, 0.5f, 2.0f);

                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, 0.0f, 7.0f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromShape(body, &shape, 2.0f);

                b2Vec2 anchor = { 0.0f, 5.0f };
                struct b2JointRevoluteDef rjd;
                b2JointRevoluteDefReset(&rjd);
				b2JointRevoluteDefInitialize(&rjd, prevBody, body, anchor);
				rjd.motorSpeed = 1.0f * b2_pi;
				rjd.maxMotorTorque = 10000.0f;
				rjd.enableMotor = true;
				m_joint1 = (struct b2JointRevolute*)b2WorldCreateJoint(m_world, &rjd);

				prevBody = body;
			}

			// Define follower.
			{
                struct b2ShapePolygon shape;
                b2ShapePolygonReset(&shape);
				b2ShapePolygonSetAsBox(&shape, 0.5f, 4.0f);

                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, 0.0f, 13.0f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromShape(body, &shape, 2.0f);

                b2Vec2 anchor = { 0.0f, 9.0f };
                struct b2JointRevoluteDef rjd;
                b2JointRevoluteDefReset(&rjd);
				b2JointRevoluteDefInitialize(&rjd, prevBody, body, anchor);
				rjd.enableMotor = false;
                b2WorldCreateJoint(m_world, &rjd);

				prevBody = body;
			}

			// Define piston
			{
                struct b2ShapePolygon shape;
                b2ShapePolygonReset(&shape);
				b2ShapePolygonSetAsBox(&shape, 1.5f, 1.5f);

                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				bd.fixedRotation = true;
				b2Vec2Make(bd.position, 0.0f, 17.0f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromShape(body, &shape, 2.0f);

                b2Vec2 anchor = { 0.0f, 17.0f };
                struct b2JointRevoluteDef rjd;
                b2JointRevoluteDefReset(&rjd);
				b2JointRevoluteDefInitialize(&rjd, prevBody, body, anchor);
                b2WorldCreateJoint(m_world, &rjd);

                b2Vec2 anchor1 = { 0.0f, 17.0f };
                b2Vec2 axis1 = { 0.0f, 1.0f };
                struct b2JointPrismaticDef pjd;
                b2JointPrismaticDefReset(&pjd);
				b2JointPrismaticDefInitialize(&pjd, ground, body, anchor1, axis1);

				pjd.maxMotorForce = 1000.0f;
				pjd.enableMotor = true;

				m_joint2 = (struct b2JointPrismatic*)b2WorldCreateJoint(m_world, &pjd);
			}

			// Create a payload
			{
                struct b2ShapePolygon shape;
                b2ShapePolygonReset(&shape);
				b2ShapePolygonSetAsBox(&shape, 1.5f, 1.5f);

                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, 0.0f, 23.0f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromShape(body, &shape, 2.0f);
			}
		}
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_F:
			b2JointPrismaticEnableMotor(m_joint2, !b2JointPrismaticIsMotorEnabled(m_joint2));
			b2BodySetAwake(b2JointGetBodyB((struct b2Joint*)m_joint2), true);
			break;

		case GLFW_KEY_M:
			b2JointRevoluteEnableMotor(m_joint1, !b2JointRevoluteIsMotorEnabled(m_joint1));
			b2BodySetAwake(b2JointGetBodyB((struct b2Joint*)m_joint1), true);
			break;
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Keys: (f) toggle friction, (m) toggle motor");
		m_textLine += m_textIncrement;
		float torque = b2JointRevoluteGetMotorTorque(m_joint1, settings.m_hertz);
		g_debugDraw.DrawString(5, m_textLine, "Motor Torque = %5.0f", (float) torque);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new SliderCrank2;
	}

	struct b2JointRevolute* m_joint1;
    struct b2JointPrismatic* m_joint2;
};

static int testIndex = RegisterTest("Examples", "Slider Crank 2", SliderCrank2::Create);
