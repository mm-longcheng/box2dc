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
#include "imgui/imgui.h"

// This tests distance joints, body destruction, and joint destruction.
class DistanceJoint : public Test
{
public:
	DistanceJoint()
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
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			bd.angularDamping = 0.1f;

			b2Vec2Make(bd.position, 0.0f, 5.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.5f, 0.5f);
			b2BodyCreateFixtureFromShape(body, &shape, 5.0f);

			m_hertz = 1.0f;
			m_dampingRatio = 0.7f;

            const b2Vec2 anchorA = { 0.0f, 15.0f };
            struct b2JointDistanceDef jd;
            b2JointDistanceDefReset(&jd);
			b2JointDistanceDefInitialize(&jd, ground, body, anchorA, bd.position);
			jd.collideConnected = true;
			m_length = jd.length;
			m_minLength = m_length;
			m_maxLength = m_length;
			b2LinearStiffness(&jd.stiffness, &jd.damping, m_hertz, m_dampingRatio, jd.bodyA, jd.bodyB);
			m_joint = (struct b2JointDistance*)b2WorldCreateJoint(m_world, &jd);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(260.0f, 150.0f));
		ImGui::Begin("Joint Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::SliderFloat("Length", &m_length, 0.0f, 20.0f, "%.0f"))
		{
			m_length = b2JointDistanceSetLength(m_joint, m_length);
		}

		if (ImGui::SliderFloat("Min Length", &m_minLength, 0.0f, 20.0f, "%.0f"))
		{
			m_minLength = b2JointDistanceSetMinLength(m_joint, m_minLength);
		}

		if (ImGui::SliderFloat("Max Length", &m_maxLength, 0.0f, 20.0f, "%.0f"))
		{
			m_maxLength = b2JointDistanceSetMaxLength(m_joint, m_maxLength);
		}

		if (ImGui::SliderFloat("Hertz", &m_hertz, 0.0f, 10.0f, "%.1f"))
		{
			float stiffness;
			float damping;
			b2LinearStiffness(&stiffness, &damping, m_hertz, m_dampingRatio, b2JointGetBodyA((struct b2Joint*)m_joint), b2JointGetBodyB((struct b2Joint*)m_joint));
            b2JointDistanceSetStiffness(m_joint, stiffness);
            b2JointDistanceSetDamping(m_joint, damping);
		}

		if (ImGui::SliderFloat("Damping Ratio", &m_dampingRatio, 0.0f, 2.0f, "%.1f"))
		{
			float stiffness;
			float damping;
			b2LinearStiffness(&stiffness, &damping, m_hertz, m_dampingRatio, b2JointGetBodyA((struct b2Joint*)m_joint), b2JointGetBodyB((struct b2Joint*)m_joint));
			b2JointDistanceSetStiffness(m_joint, stiffness);
			b2JointDistanceSetDamping(m_joint, damping);
		}

		ImGui::End();
	}

	static Test* Create()
	{
		return new DistanceJoint;
	}

	struct b2JointDistance* m_joint;
	float m_length;
	float m_minLength;
	float m_maxLength;
	float m_hertz;
	float m_dampingRatio;
};

static int testIndex = RegisterTest("Joints", "Distance Joint", DistanceJoint::Create);
