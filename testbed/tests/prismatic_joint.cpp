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

// Test the prismatic joint with limits and motor options.
class PrismaticJoint : public Test
{
public:
	PrismaticJoint()
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

		m_enableLimit = true;
		m_enableMotor = false;
		m_motorSpeed = 10.0f;

		{
			struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 1.0f, 1.0f);

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 0.0f, 10.0f);
			bd.angle = 0.5f * b2_pi;
			bd.allowSleep = false;
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(body, &shape, 5.0f);

            struct b2JointPrismaticDef pjd;
            b2JointPrismaticDefReset(&pjd);

			// Horizontal
            b2Vec2 axis = { 1.0f, 0.0f };
			b2JointPrismaticDefInitialize(&pjd, ground, body, bd.position, axis);

			pjd.motorSpeed = m_motorSpeed;
			pjd.maxMotorForce = 10000.0f;
			pjd.enableMotor = m_enableMotor;
			pjd.lowerTranslation = -10.0f;
			pjd.upperTranslation = 10.0f;
			pjd.enableLimit = m_enableLimit;

			m_joint = (struct b2JointPrismatic*)b2WorldCreateJoint(m_world, &pjd);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f));
		ImGui::Begin("Joint Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Checkbox("Limit", &m_enableLimit))
		{
			b2JointPrismaticEnableLimit(m_joint, m_enableLimit);
		}

		if (ImGui::Checkbox("Motor", &m_enableMotor))
		{
			b2JointPrismaticEnableMotor(m_joint, m_enableMotor);
		}

		if (ImGui::SliderFloat("Speed", &m_motorSpeed, -100.0f, 100.0f, "%.0f"))
		{
			b2JointPrismaticSetMotorSpeed(m_joint, m_motorSpeed);
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);
		float force = b2JointPrismaticGetMotorForce(m_joint, settings.m_hertz);
		g_debugDraw.DrawString(5, m_textLine, "Motor Force = %4.0f", force);
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new PrismaticJoint;
	}

	struct b2JointPrismatic* m_joint;
	float m_motorSpeed;
	bool m_enableMotor;
	bool m_enableLimit;
};

static int testIndex = RegisterTest("Joints", "Prismatic", PrismaticJoint::Create);
