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

/// This tests bullet collision and provides an example of a gameplay scenario.
/// This also uses a loop shape.
class Pinball : public Test
{
public:
	Pinball()
	{
		// Ground body
		struct b2Body* ground = NULL;
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			ground = b2WorldCreateBody(m_world, &bd);

			b2Vec2 vs[5];
			b2Vec2Make(vs[0], -8.0f, 6.0f);
            b2Vec2Make(vs[1], -8.0f, 20.0f);
            b2Vec2Make(vs[2], 8.0f, 20.0f);
            b2Vec2Make(vs[3], 8.0f, 6.0f);
            b2Vec2Make(vs[4], 0.0f, -2.0f);

            struct b2ShapeChain loop;
            b2ShapeChainReset(&loop);
            b2ShapeChainCreateLoop(&loop, vs, 5);
            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&loop;
			fd.density = 0.0f;
			b2BodyCreateFixtureFromDef(ground, &fd);
            b2ShapeChainClear(&loop);
		}

		// Flippers
		{
            b2Vec2 p1 = { -2.0f, 0.0f };
            b2Vec2 p2 = { 2.0f, 0.0f };

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;

			b2Vec2Assign(bd.position, p1);
            struct b2Body* leftFlipper = b2WorldCreateBody(m_world, &bd);

			b2Vec2Assign(bd.position, p2);
            struct b2Body* rightFlipper = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon box;
            b2ShapePolygonReset(&box);
			b2ShapePolygonSetAsBox(&box, 1.75f, 0.1f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&box;
			fd.density = 1.0f;

			b2BodyCreateFixtureFromDef(leftFlipper, &fd);
			b2BodyCreateFixtureFromDef(rightFlipper, &fd);

            struct b2JointRevoluteDef jd;
            b2JointRevoluteDefReset(&jd);
			jd.bodyA = ground;
			b2Vec2SetZero(jd.localAnchorB);
			jd.enableMotor = true;
			jd.maxMotorTorque = 1000.0f;
			jd.enableLimit = true;

			jd.motorSpeed = 0.0f;
			b2Vec2Assign(jd.localAnchorA, p1);
			jd.bodyB = leftFlipper;
			jd.lowerAngle = -30.0f * b2_pi / 180.0f;
			jd.upperAngle = 5.0f * b2_pi / 180.0f;
			m_leftJoint = (struct b2JointRevolute*)b2WorldCreateJoint(m_world, &jd);

			jd.motorSpeed = 0.0f;
			b2Vec2Assign(jd.localAnchorA, p2);
			jd.bodyB = rightFlipper;
			jd.lowerAngle = -5.0f * b2_pi / 180.0f;
			jd.upperAngle = 30.0f * b2_pi / 180.0f;
			m_rightJoint = (struct b2JointRevolute*)b2WorldCreateJoint(m_world, &jd);
		}

		// Circle character
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, 1.0f, 15.0f);
			bd.type = b2BodyTypeDynamic;
			bd.bullet = true;

			m_ball = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
			shape.m_radius = 0.2f;

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 1.0f;
			b2BodyCreateFixtureFromDef(m_ball, &fd);
		}

		m_button = false;
	}

	void Step(Settings& settings) override
	{
		if (m_button)
		{
			b2JointRevoluteSetMotorSpeed(m_leftJoint, 20.0f);
			b2JointRevoluteSetMotorSpeed(m_rightJoint, -20.0f);
		}
		else
		{
			b2JointRevoluteSetMotorSpeed(m_leftJoint, -10.0f);
			b2JointRevoluteSetMotorSpeed(m_rightJoint, 10.0f);
		}

		Test::Step(settings);

		g_debugDraw.DrawString(5, m_textLine, "Press 'a' to control the flippers");
		m_textLine += m_textIncrement;

	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_button = true;
			break;
		}
	}

	void KeyboardUp(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_button = false;
			break;
		}
	}

	static Test* Create()
	{
		return new Pinball;
	}

	struct b2JointRevolute* m_leftJoint;
    struct b2JointRevolute* m_rightJoint;
    struct b2Body* m_ball;
	bool m_button;
};

static int testIndex = RegisterTest("Examples", "Pinball", Pinball::Create);
