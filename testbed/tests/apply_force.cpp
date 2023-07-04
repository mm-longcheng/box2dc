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

// This test shows how to apply forces and torques to a body.
// It also shows how to use the friction joint that can be useful
// for overhead games.
class ApplyForce : public Test
{
public:
	ApplyForce()
	{
        b2Vec2 Gravity = { 0.0f, 0.0f };
        b2WorldSetGravity(m_world, Gravity);

		const float k_restitution = 0.4f;

		struct b2Body* ground;
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, 0.0f, 20.0f);
			ground = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);

            struct b2FixtureDef sd;
            b2FixtureDefReset(&sd);
			sd.shape = (struct b2Shape*)&shape;
			sd.density = 0.0f;
			sd.restitution = k_restitution;

            b2Vec2 a;
            b2Vec2 b;

			// Left vertical
            b2Vec2Make(a, -20.0f, -20.0f);
            b2Vec2Make(b, -20.0f, 20.0f);
			b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromDef(ground, &sd);

			// Right vertical
            b2Vec2Make(a, 20.0f, -20.0f);
            b2Vec2Make(b, 20.0f, 20.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
            b2BodyCreateFixtureFromDef(ground, &sd);

			// Top horizontal
            b2Vec2Make(a, -20.0f, 20.0f);
            b2Vec2Make(b, 20.0f, 20.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
            b2BodyCreateFixtureFromDef(ground, &sd);

			// Bottom horizontal
            b2Vec2Make(a, -20.0f, -20.0f);
            b2Vec2Make(b, 20.0f, -20.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
            b2BodyCreateFixtureFromDef(ground, &sd);
		}

        {
            b2Transform xf1;
            b2RotFromAngle(xf1[1], 0.3524f * b2_pi);
            b2RotGetXAxis(xf1[1], xf1[0]);

            b2Vec2 v;
            b2Vec2 vertices[3];

            b2Vec2Make(v, -1.0f, 0.0f);
            b2TransformMulVec2(vertices[0], xf1, v);
            b2Vec2Make(v, 1.0f, 0.0f);
            b2TransformMulVec2(vertices[1], xf1, v);
            b2Vec2Make(v, 0.0f, 0.5f);
            b2TransformMulVec2(vertices[2], xf1, v);

            struct b2ShapePolygon poly1;
            b2ShapePolygonReset(&poly1);
            b2ShapePolygonSetPoints(&poly1, vertices, 3);

            struct b2FixtureDef sd1;
            b2FixtureDefReset(&sd1);
            sd1.shape = (struct b2Shape*)&poly1;
            sd1.density = 2.0f;

            b2Transform xf2;
            b2RotFromAngle(xf2[1], -0.3524f * b2_pi);
            b2RotGetXAxis(xf2[1], xf2[0]);
            b2Vec2Negate(xf2[0], xf2[0]);

            b2Vec2Make(v, -1.0f, 0.0f);
            b2TransformMulVec2(vertices[0], xf2, v);
            b2Vec2Make(v, 1.0f, 0.0f);
            b2TransformMulVec2(vertices[1], xf2, v);
            b2Vec2Make(v, 0.0f, 0.5f);
            b2TransformMulVec2(vertices[2], xf2, v);

            struct b2ShapePolygon poly2;
            b2ShapePolygonReset(&poly2);
            b2ShapePolygonSetPoints(&poly2, vertices, 3);

            struct b2FixtureDef sd2;
            b2FixtureDefReset(&sd2);
            sd2.shape = (struct b2Shape*)&poly2;
            sd2.density = 2.0f;

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            bd.type = b2BodyTypeDynamic;

            b2Vec2Make(bd.position, 0.0f, 3.0);
            bd.angle = b2_pi;
            bd.allowSleep = false;
            m_body = b2WorldCreateBody(m_world, &bd);
            b2BodyCreateFixtureFromDef(m_body, &sd1);
            b2BodyCreateFixtureFromDef(m_body, &sd2);

            float gravity = 10.0f;
            float I = b2BodyGetInertia(m_body);
            float mass = b2BodyGetMass(m_body);

            // Compute an effective radius that can be used to
            // set the max torque for a friction joint
            // For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
            float radius = b2Sqrt(2.0f * I / mass);

            struct b2JointFrictionDef jd;
            b2JointFrictionDefReset(&jd);
            jd.bodyA = ground;
            jd.bodyB = m_body;
            b2Vec2SetZero(jd.localAnchorA);
            b2Vec2Assign(jd.localAnchorB, b2BodyGetLocalCenter(m_body));
            jd.collideConnected = true;
            jd.maxForce = 0.5f * mass * gravity;
            jd.maxTorque = 0.2f * mass * radius * gravity;

            b2WorldCreateJoint(m_world, &jd);
        }

		{
			struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
            b2ShapePolygonSetAsBox(&shape, 0.5f, 0.5f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 1.0f;
			fd.friction = 0.3f;

			for (int i = 0; i < 10; ++i)
			{
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;

				b2Vec2Make(bd.position, 0.0f, 7.0f + 1.54f * i);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);

				b2BodyCreateFixtureFromDef(body, &fd);

				float gravity = 10.0f;
				float I = b2BodyGetInertia(body);
				float mass = b2BodyGetMass(body);

				// For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
				float radius = b2Sqrt(2.0f * I / mass);

				struct b2JointFrictionDef jd;
                b2JointFrictionDefReset(&jd);
				b2Vec2SetZero(jd.localAnchorA);
                b2Vec2SetZero(jd.localAnchorB);
				jd.bodyA = ground;
				jd.bodyB = body;
				jd.collideConnected = true;
				jd.maxForce = mass * gravity;
				jd.maxTorque = 0.1f * mass * radius * gravity;

				b2WorldCreateJoint(m_world, &jd);
			}
        }
	}

	void Step(Settings& settings) override
	{
		g_debugDraw.DrawString(5, m_textLine, "Forward (W), Turn (A) and (D)");
		m_textLine += m_textIncrement;

		if (glfwGetKey(g_mainWindow, GLFW_KEY_W) == GLFW_PRESS)
		{
            b2Vec2 v;

            b2Vec2 f;
            b2Vec2 p;

            b2Vec2Make(v, 0.0f, -50.0f);
			b2BodyGetWorldVector(m_body, v, f);
            b2Vec2Make(v, 0.0f, 3.0f);
			b2BodyGetWorldPoint(m_body, v, p);
			b2BodyApplyForce(m_body, f, p, true);
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_A) == GLFW_PRESS)
		{
			b2BodyApplyTorque(m_body, 10.0f, true);
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_D) == GLFW_PRESS)
		{
			b2BodyApplyTorque(m_body, -10.0f, true);
		}

		Test::Step(settings);
	}

	static Test* Create()
	{
		return new ApplyForce;
	}

    struct b2Body* m_body;
};

static int testIndex = RegisterTest("Forces", "Apply Force", ApplyForce::Create);
