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
#include "imgui/imgui.h"

class RevoluteJoint : public Test
{
public:
	RevoluteJoint()
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

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			//fd.filter.categoryBits = 2;

			b2BodyCreateFixtureFromDef(ground, &fd);
		}

		m_enableLimit = true;
		m_enableMotor = false;
		m_motorSpeed = 1.0f;

		{
            b2Vec2 center = { 0.0f, 3.0f };
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
            b2ShapePolygonSetAsBoxDetail(&shape, 0.25f, 3.0f, center, 0.0f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, -10.0f, 20.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(body, &shape, 5.0f);

            b2Vec2 anchor = { -10.0f, 20.5f };
            struct b2JointRevoluteDef jd;
            b2JointRevoluteDefReset(&jd);
			b2JointRevoluteDefInitialize(&jd, ground, body, anchor);
			jd.motorSpeed = m_motorSpeed;
			jd.maxMotorTorque = 10000.0f;
			jd.enableMotor = m_enableMotor;
			jd.lowerAngle = -0.25f * b2_pi;
			jd.upperAngle = 0.5f * b2_pi;
			jd.enableLimit = m_enableLimit;

			m_joint1 = (struct b2JointRevolute*)b2WorldCreateJoint(m_world, &jd);
		}

		{
            struct b2ShapeCircle circle_shape;
            b2ShapeCircleReset(&circle_shape);
			circle_shape.m_radius = 2.0f;

            struct b2BodyDef circle_bd;
            b2BodyDefReset(&circle_bd);
			circle_bd.type = b2BodyTypeDynamic;
			b2Vec2Make(circle_bd.position, 5.0f, 30.0f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.density = 5.0f;
			fd.filter.maskBits = 1;
			fd.shape = (struct b2Shape*)&circle_shape;

			m_ball = b2WorldCreateBody(m_world, &circle_bd);
			b2BodyCreateFixtureFromDef(m_ball, &fd);

            b2Vec2 center = { -10.0f, 0.0f };
            struct b2ShapePolygon polygon_shape;
            b2ShapePolygonReset(&polygon_shape);
            b2ShapePolygonSetAsBoxDetail(&polygon_shape, 10.0f, 0.5f, center, 0.0f);

            struct b2BodyDef polygon_bd;
            b2BodyDefReset(&polygon_bd);
			b2Vec2Make(polygon_bd.position, 20.0f, 10.0f);
			polygon_bd.type = b2BodyTypeDynamic;
			polygon_bd.bullet = true;
            struct b2Body* polygon_body = b2WorldCreateBody(m_world, &polygon_bd);
			b2BodyCreateFixtureFromShape(polygon_body, &polygon_shape, 2.0f);

            b2Vec2 anchor = { 19.0f, 10.0f };
            struct b2JointRevoluteDef jd;
            b2JointRevoluteDefReset(&jd);
			b2JointRevoluteDefInitialize(&jd, ground, polygon_body, anchor);
			jd.lowerAngle = -0.25f * b2_pi;
			jd.upperAngle = 0.0f * b2_pi;
			jd.enableLimit = true;
			jd.enableMotor = true;
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10000.0f;

			m_joint2 = (struct b2JointRevolute*)b2WorldCreateJoint(m_world, &jd);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f));
		ImGui::Begin("Joint Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Checkbox("Limit", &m_enableLimit))
		{
			b2JointRevoluteEnableLimit(m_joint1, m_enableLimit);
		}

		if (ImGui::Checkbox("Motor", &m_enableMotor))
		{
            b2JointRevoluteEnableMotor(m_joint1, m_enableMotor);
		}

		if (ImGui::SliderFloat("Speed", &m_motorSpeed, -20.0f, 20.0f, "%.0f"))
		{
            b2JointRevoluteSetMotorSpeed(m_joint1, m_motorSpeed);
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);
		
		float torque1 = b2JointRevoluteGetMotorTorque(m_joint1, settings.m_hertz);
		g_debugDraw.DrawString(5, m_textLine, "Motor Torque 1= %4.0f", torque1);
		m_textLine += m_textIncrement;

		float torque2 = b2JointRevoluteGetMotorTorque(m_joint2, settings.m_hertz);
		g_debugDraw.DrawString(5, m_textLine, "Motor Torque 2= %4.0f", torque2);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new RevoluteJoint;
	}

	b2Body* m_ball;
	struct b2JointRevolute* m_joint1;
    struct b2JointRevolute* m_joint2;
	float m_motorSpeed;
	bool m_enableMotor;
	bool m_enableLimit;
};

static int testIndex = RegisterTest("Joints", "Revolute", RevoluteJoint::Create);
