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

class GearJoint : public Test
{
public:
	GearJoint()
	{
		struct b2Body* ground = NULL;
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			ground = b2WorldCreateBody(m_world, &bd);

            b2Vec2 a = { 50.0f, 0.0f };
            b2Vec2 b = { -50.0f, 0.0f };
            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
			b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

		{
            struct b2ShapeCircle circle1;
            b2ShapeCircleReset(&circle1);
			circle1.m_radius = 1.0f;

            struct b2ShapePolygon box;
            b2ShapePolygonReset(&box);
			b2ShapePolygonSetAsBox(&box, 0.5f, 5.0f);

            struct b2ShapeCircle circle2;
            b2ShapeCircleReset(&circle2);
			circle2.m_radius = 2.0f;
			
            struct b2BodyDef bd1;
            b2BodyDefReset(&bd1);
			bd1.type = b2BodyTypeStatic;
			b2Vec2Make(bd1.position, 10.0f, 9.0f);
            struct b2Body* body1 = b2WorldCreateBody(m_world, &bd1);
			b2BodyCreateFixtureFromShape(body1, &circle1, 5.0f);

            struct b2BodyDef bd2;
            b2BodyDefReset(&bd2);
			bd2.type = b2BodyTypeDynamic;
            b2Vec2Make(bd2.position, 10.0f, 8.0f);
            struct b2Body* body2 = b2WorldCreateBody(m_world, &bd2);
			b2BodyCreateFixtureFromShape(body2, &box, 5.0f);

            struct b2BodyDef bd3;
            b2BodyDefReset(&bd3);
			bd3.type = b2BodyTypeDynamic;
			b2Vec2Make(bd3.position, 10.0f, 6.0f);
            struct b2Body* body3 = b2WorldCreateBody(m_world, &bd3);
			b2BodyCreateFixtureFromShape(body3, &circle2, 5.0f);

            struct b2JointRevoluteDef jd1;
            b2JointRevoluteDefReset(&jd1);
			b2JointRevoluteDefInitialize(&jd1, body1, body2, bd1.position);
            struct b2Joint* joint1 = b2WorldCreateJoint(m_world, &jd1);

            struct b2JointRevoluteDef jd2;
            b2JointRevoluteDefReset(&jd2);
			b2JointRevoluteDefInitialize(&jd2, body2, body3, bd3.position);
            struct b2Joint* joint2 = b2WorldCreateJoint(m_world, &jd2);

            struct b2JointGearDef jd4;
            b2JointGearDefReset(&jd4);
			jd4.bodyA = body1;
			jd4.bodyB = body3;
			jd4.joint1 = joint1;
			jd4.joint2 = joint2;
			jd4.ratio = circle2.m_radius / circle1.m_radius;
			b2WorldCreateJoint(m_world, &jd4);
		}

		{
            struct b2ShapeCircle circle1;
            b2ShapeCircleReset(&circle1);
			circle1.m_radius = 1.0f;

            struct b2ShapeCircle circle2;
            b2ShapeCircleReset(&circle2);
			circle2.m_radius = 2.0f;
			
            struct b2ShapePolygon box;
            b2ShapePolygonReset(&box);
			b2ShapePolygonSetAsBox(&box, 0.5f, 5.0f);

            struct b2BodyDef bd1;
            b2BodyDefReset(&bd1);
			bd1.type = b2BodyTypeDynamic;
			b2Vec2Make(bd1.position, -3.0f, 12.0f);
            struct b2Body* body1 = b2WorldCreateBody(m_world, &bd1);
			b2BodyCreateFixtureFromShape(body1, &circle1, 5.0f);

            struct b2JointRevoluteDef jd1;
            b2JointRevoluteDefReset(&jd1);
			jd1.bodyA = ground;
			jd1.bodyB = body1;
			b2BodyGetLocalPoint(ground, bd1.position, jd1.localAnchorA);
			b2BodyGetLocalPoint(body1, bd1.position, jd1.localAnchorB);
			jd1.referenceAngle = b2BodyGetAngle(body1) - b2BodyGetAngle(ground);
			m_joint1 = (struct b2JointRevolute*)b2WorldCreateJoint(m_world, &jd1);

            struct b2BodyDef bd2;
            b2BodyDefReset(&bd2);
			bd2.type = b2BodyTypeDynamic;
			b2Vec2Make(bd2.position, 0.0f, 12.0f);
            struct b2Body* body2 = b2WorldCreateBody(m_world, &bd2);
			b2BodyCreateFixtureFromShape(body2, &circle2, 5.0f);

            struct b2JointRevoluteDef jd2;
            b2JointRevoluteDefReset(&jd2);
			b2JointRevoluteDefInitialize(&jd2, ground, body2, bd2.position);
			m_joint2 = (struct b2JointRevolute*)b2WorldCreateJoint(m_world, &jd2);

            struct b2BodyDef bd3;
            b2BodyDefReset(&bd3);
			bd3.type = b2BodyTypeDynamic;
			b2Vec2Make(bd3.position, 2.5f, 12.0f);
            struct b2Body* body3 = b2WorldCreateBody(m_world, &bd3);
			b2BodyCreateFixtureFromShape(body3, &box, 5.0f);

            b2Vec2 axis = { 0.0f, 1.0f };
            struct b2JointPrismaticDef jd3;
            b2JointPrismaticDefReset(&jd3);
			b2JointPrismaticDefInitialize(&jd3, ground, body3, bd3.position, axis);
			jd3.lowerTranslation = -5.0f;
			jd3.upperTranslation = 5.0f;
			jd3.enableLimit = true;

			m_joint3 = (struct b2JointPrismatic*)b2WorldCreateJoint(m_world, &jd3);

            struct b2JointGearDef jd4;
            b2JointGearDefReset(&jd4);
			jd4.bodyA = body1;
			jd4.bodyB = body2;
			jd4.joint1 = (struct b2Joint*)m_joint1;
			jd4.joint2 = (struct b2Joint*)m_joint2;
			jd4.ratio = circle2.m_radius / circle1.m_radius;
			m_joint4 = (struct b2JointGear*)b2WorldCreateJoint(m_world, &jd4);

            struct b2JointGearDef jd5;
            b2JointGearDefReset(&jd5);
			jd5.bodyA = body2;
			jd5.bodyB = body3;
			jd5.joint1 = (struct b2Joint*)m_joint2;
			jd5.joint2 = (struct b2Joint*)m_joint3;
			jd5.ratio = -1.0f / circle2.m_radius;
			m_joint5 = (struct b2JointGear*)b2WorldCreateJoint(m_world, &jd5);
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		float ratio, value;
		
		ratio = b2JointGearGetRatio(m_joint4);
		value = b2JointRevoluteGetJointAngle(m_joint1) + ratio * b2JointRevoluteGetJointAngle(m_joint2);
		g_debugDraw.DrawString(5, m_textLine, "theta1 + %4.2f * theta2 = %4.2f", (float) ratio, (float) value);
		m_textLine += m_textIncrement;

		ratio = b2JointGearGetRatio(m_joint5);
		value = b2JointRevoluteGetJointAngle(m_joint2) + ratio * b2JointPrismaticGetJointTranslation(m_joint3);
		g_debugDraw.DrawString(5, m_textLine, "theta2 + %4.2f * delta = %4.2f", (float) ratio, (float) value);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new GearJoint;
	}

	struct b2JointRevolute* m_joint1;
	struct b2JointRevolute* m_joint2;
	struct b2JointPrismatic* m_joint3;
	struct b2JointGear* m_joint4;
	struct b2JointGear* m_joint5;
};

static int testIndex = RegisterTest("Joints", "Gear", GearJoint::Create);
