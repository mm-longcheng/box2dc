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

class PulleyJoint : public Test
{
public:
	PulleyJoint()
	{
		float y = 16.0f;
		float L = 12.0f;
		float a = 1.0f;
		float b = 2.0f;

		struct b2Body* ground = NULL;
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			ground = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeCircle circle;
            b2ShapeCircleReset(&circle);
			circle.m_radius = 2.0f;

			b2Vec2Make(circle.m_p, -10.0f, y + b + L);
			b2BodyCreateFixtureFromShape(ground, &circle, 0.0f);

			b2Vec2Make(circle.m_p, 10.0f, y + b + L);
			b2BodyCreateFixtureFromShape(ground, &circle, 0.0f);
		}

		{

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, a, b);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;

			//bd.fixedRotation = true;
			b2Vec2Make(bd.position, -10.0f, y);
            struct b2Body* body1 = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(body1, &shape, 5.0f);

            b2Vec2Make(bd.position, 10.0f, y);
            struct b2Body* body2 = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(body2, &shape, 5.0f);

            struct b2JointPulleyDef pulleyDef;
            b2JointPulleyDefReset(&pulleyDef);
            b2Vec2 anchor1 = { -10.0f, y + b };
            b2Vec2 anchor2 = { 10.0f, y + b };
            b2Vec2 groundAnchor1 = { -10.0f, y + b + L };
            b2Vec2 groundAnchor2 = { 10.0f, y + b + L };
			b2JointPulleyDefInitialize(&pulleyDef, body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 1.5f);

			m_joint1 = (struct b2JointPulley*)b2WorldCreateJoint(m_world, &pulleyDef);
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		float ratio = b2JointPulleyGetRatio(m_joint1);
		float L = b2JointPulleyGetCurrentLengthA(m_joint1) + ratio * b2JointPulleyGetCurrentLengthB(m_joint1);
		g_debugDraw.DrawString(5, m_textLine, "L1 + %4.2f * L2 = %4.2f", (float) ratio, (float) L);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new PulleyJoint;
	}

	struct b2JointPulley* m_joint1;
};

static int testIndex = RegisterTest("Joints", "Pulley", PulleyJoint::Create);
