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

/// This test shows how a distance joint can be used to stabilize a chain of
/// bodies with a heavy payload. Notice that the distance joint just prevents
/// excessive stretching and has no other effect.
/// By disabling the distance joint you can see that the Box2D solver has trouble
/// supporting heavy bodies with light bodies. Try playing around with the
/// densities, time step, and iterations to see how they affect stability.
/// This test also shows how to use contact filtering. Filtering is configured
/// so that the payload does not collide with the chain.
class WreckingBall : public Test
{
public:
	WreckingBall()
	{
        b2JointDistanceDefReset(&this->m_distanceJointDef);

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
			fd.friction = 0.2f;
			fd.filter.categoryBits = 0x0001;
			fd.filter.maskBits = 0xFFFF & ~0x0002;

            struct b2JointRevoluteDef jd;
            b2JointRevoluteDefReset(&jd);
			jd.collideConnected = false;

			const int32 N = 10;
			const float y = 15.0f;
			b2Vec2Make(m_distanceJointDef.localAnchorA, 0.0f, y);

            struct b2Body* prevBody = ground;
			for (int32 i = 0; i < N; ++i)
			{
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, 0.5f + 1.0f * i, y);
				if (i == N - 1)
				{
					b2Vec2Make(bd.position, 1.0f * i, y);
					bd.angularDamping = 0.4f;
				}

                struct b2Body* body = b2WorldCreateBody(m_world, &bd);

				if (i == N - 1)
				{
                    struct b2ShapeCircle circleShape;
                    b2ShapeCircleReset(&circleShape);
					circleShape.m_radius = 1.5f;
                    struct b2FixtureDef sfd;
                    b2FixtureDefReset(&sfd);
					sfd.shape = (struct b2Shape*)&circleShape;
					sfd.density = 100.0f;
					sfd.filter.categoryBits = 0x0002;
					b2BodyCreateFixtureFromDef(body, &sfd);
				}
				else
				{
					b2BodyCreateFixtureFromDef(body, &fd);
				}

                b2Vec2 anchor = { float(i), y };
				b2JointRevoluteDefInitialize(&jd, prevBody, body, anchor);
				b2WorldCreateJoint(m_world, &jd);

				prevBody = body;
			}

			b2Vec2SetZero(m_distanceJointDef.localAnchorB);

			float extraLength = 0.01f;
			m_distanceJointDef.minLength = 0.0f;
			m_distanceJointDef.maxLength = N - 1.0f + extraLength;
			m_distanceJointDef.bodyB = prevBody;
		}

		{
			m_distanceJointDef.bodyA = ground;
			m_distanceJoint = b2WorldCreateJoint(m_world, &m_distanceJointDef);
			m_stabilize = true;
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f));
		ImGui::Begin("Wrecking Ball Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Checkbox("Stabilize", &m_stabilize))
		{
			if (m_stabilize == true && m_distanceJoint == nullptr)
			{
				m_distanceJoint = b2WorldCreateJoint(m_world, &m_distanceJointDef);
			}
			else if (m_stabilize == false && m_distanceJoint != nullptr)
			{
				b2WorldDeleteJoint(m_world, m_distanceJoint);
				m_distanceJoint = nullptr;
			}
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		if (m_distanceJoint)
		{
			g_debugDraw.DrawString(5, m_textLine, "Distance Joint ON");
		}
		else
		{
			g_debugDraw.DrawString(5, m_textLine, "Distance Joint OFF");
		}
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new WreckingBall;
	}

	struct b2JointDistanceDef m_distanceJointDef;
	struct b2Joint* m_distanceJoint;
	bool m_stabilize;
};

static int testIndex = RegisterTest("Examples", "Wrecking Ball", WreckingBall::Create);
