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

// This is a fun demo that shows off the wheel joint
class Car : public Test
{
public:
	Car()
	{		
		m_speed = 50.0f;

		struct b2Body* ground = NULL;
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			ground = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 0.0f;
			fd.friction = 0.6f;

            b2Vec2 a;
            b2Vec2 b;
            b2Vec2Make(a, -20.0f, 0.0f);
            b2Vec2Make(b, 20.0f, 0.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromDef(ground, &fd);

			float hs[10] = {0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};

			float x = 20.0f, y1 = 0.0f, dx = 5.0f;

			for (int32 i = 0; i < 10; ++i)
			{
                float y2 = hs[i];
                b2Vec2Make(a, x, y1);
                b2Vec2Make(b, x + dx, y2);
                b2ShapeEdgeSetTwoSided(&shape, a, b);
				b2BodyCreateFixtureFromDef(ground, &fd);
				y1 = y2;
				x += dx;
			}

			for (int32 i = 0; i < 10; ++i)
			{
				float y2 = hs[i];
                b2Vec2Make(a, x, y1);
                b2Vec2Make(b, x + dx, y2);
				b2ShapeEdgeSetTwoSided(&shape, a, b);
				b2BodyCreateFixtureFromDef(ground, &fd);
				y1 = y2;
				x += dx;
			}

            b2Vec2Make(a, x, 0.0f);
            b2Vec2Make(b, x + 40.0f, 0.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
            b2BodyCreateFixtureFromDef(ground, &fd);

			x += 80.0f;
            b2Vec2Make(a, x, 0.0f);
            b2Vec2Make(b, x + 40.0f, 0.0f);
			b2ShapeEdgeSetTwoSided(&shape, a, b);
            b2BodyCreateFixtureFromDef(ground, &fd);

			x += 40.0f;
            b2Vec2Make(a, x, 0.0f);
            b2Vec2Make(b, x + 10.0f, 5.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
            b2BodyCreateFixtureFromDef(ground, &fd);

			x += 20.0f;
            b2Vec2Make(a, x, 0.0f);
            b2Vec2Make(b, x + 40.0f, 0.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
            b2BodyCreateFixtureFromDef(ground, &fd);

			x += 40.0f;
            b2Vec2Make(a, x, 0.0f);
            b2Vec2Make(b, x, 20.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
            b2BodyCreateFixtureFromDef(ground, &fd);
        }

		// Teeter
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, 140.0f, 1.0f);
			bd.type = b2BodyTypeDynamic;
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon box;
            b2ShapePolygonReset(&box);
			b2ShapePolygonSetAsBox(&box, 10.0f, 0.25f);
			b2BodyCreateFixtureFromShape(body, &box, 1.0f);

            struct b2JointRevoluteDef jd;
            b2JointRevoluteDefReset(&jd);
			b2JointRevoluteDefInitialize(&jd, ground, body, b2BodyGetPosition(body));
			jd.lowerAngle = -8.0f * b2_pi / 180.0f;
			jd.upperAngle = 8.0f * b2_pi / 180.0f;
			jd.enableLimit = true;
			b2WorldCreateJoint(m_world, &jd);

			b2BodyApplyAngularImpulse(body, 100.0f, true);
		}

		// Bridge
		{
			int32 N = 20;
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 1.0f, 0.125f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 1.0f;
			fd.friction = 0.6f;

            struct b2JointRevoluteDef jd;
            b2JointRevoluteDefReset(&jd);

            struct b2Body* prevBody = ground;
			for (int32 i = 0; i < N; ++i)
			{
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, 161.0f + 2.0f * i, -0.125f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromDef(body, &fd);

                b2Vec2 anchor = { 160.0f + 2.0f * i, -0.125f };
				b2JointRevoluteDefInitialize(&jd, prevBody, body, anchor);
                b2WorldCreateJoint(m_world, &jd);

				prevBody = body;
			}

            b2Vec2 anchor = { 160.0f + 2.0f * N, -0.125f };
			b2JointRevoluteDefInitialize(&jd, prevBody, ground, anchor);
            b2WorldCreateJoint(m_world, &jd);
		}

		// Boxes
		{
            struct b2ShapePolygon box;
            b2ShapePolygonReset(&box);
			b2ShapePolygonSetAsBox(&box, 0.5f, 0.5f);

            struct b2Body* body = NULL;
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;

			b2Vec2Make(bd.position, 230.0f, 0.5f);
			body = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(body, &box, 0.5f);

            b2Vec2Make(bd.position, 230.0f, 1.5f);
            body = b2WorldCreateBody(m_world, &bd);
            b2BodyCreateFixtureFromShape(body, &box, 0.5f);

			b2Vec2Make(bd.position, 230.0f, 2.5f);
            body = b2WorldCreateBody(m_world, &bd);
            b2BodyCreateFixtureFromShape(body, &box, 0.5f);

			b2Vec2Make(bd.position, 230.0f, 3.5f);
            body = b2WorldCreateBody(m_world, &bd);
            b2BodyCreateFixtureFromShape(body, &box, 0.5f);

			b2Vec2Make(bd.position, 230.0f, 4.5f);
            body = b2WorldCreateBody(m_world, &bd);
            b2BodyCreateFixtureFromShape(body, &box, 0.5f);
		}

		// Car
		{
            struct b2ShapePolygon chassis;
            b2ShapePolygonReset(&chassis);
			b2Vec2 vertices[8];
			b2Vec2Make(vertices[0], -1.5f, -0.5f);
			b2Vec2Make(vertices[1], 1.5f, -0.5f);
			b2Vec2Make(vertices[2], 1.5f, 0.0f);
			b2Vec2Make(vertices[3], 0.0f, 0.9f);
			b2Vec2Make(vertices[4], -1.15f, 0.9f);
			b2Vec2Make(vertices[5], -1.5f, 0.2f);
			b2ShapePolygonSetPoints(&chassis, vertices, 6);

            struct b2ShapeCircle circle;
            b2ShapeCircleReset(&circle);
			circle.m_radius = 0.4f;

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 0.0f, 1.0f);
			m_car = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(m_car, &chassis, 1.0f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&circle;
			fd.density = 1.0f;
			fd.friction = 0.9f;

            b2Vec2Make(bd.position, -1.0f, 0.35f);
			m_wheel1 = b2WorldCreateBody(m_world, &bd);
            b2BodyCreateFixtureFromDef(m_wheel1, &fd);

            b2Vec2Make(bd.position, 1.0f, 0.4f);
			m_wheel2 = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromDef(m_wheel2, &fd);

            struct b2JointWheelDef jd;
            b2JointWheelDefReset(&jd);
            b2Vec2 axis = { 0.0f, 1.0f };

			float mass1 = b2BodyGetMass(m_wheel1);
			float mass2 = b2BodyGetMass(m_wheel2);

			float hertz = 4.0f;
			float dampingRatio = 0.7f;
			float omega = 2.0f * b2_pi * hertz;

			b2JointWheelDefInitialize(&jd, m_car, m_wheel1, b2BodyGetPosition(m_wheel1), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.stiffness = mass1 * omega * omega;
			jd.damping = 2.0f * mass1 * dampingRatio * omega;
			jd.lowerTranslation = -0.25f;
			jd.upperTranslation = 0.25f;
			jd.enableLimit = true;
			m_spring1 = (struct b2JointWheel*)b2WorldCreateJoint(m_world, &jd);

			b2JointWheelDefInitialize(&jd, m_car, m_wheel2, b2BodyGetPosition(m_wheel2), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = false;
			jd.stiffness = mass2 * omega * omega;
			jd.damping = 2.0f * mass2 * dampingRatio * omega;
			jd.lowerTranslation = -0.25f;
			jd.upperTranslation = 0.25f;
			jd.enableLimit = true;
			m_spring2 = (struct b2JointWheel*)b2WorldCreateJoint(m_world, &jd);
		}
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			b2JointWheelSetMotorSpeed(m_spring1, m_speed);
			break;

		case GLFW_KEY_S:
			b2JointWheelSetMotorSpeed(m_spring1, 0.0f);
			break;

		case GLFW_KEY_D:
			b2JointWheelSetMotorSpeed(m_spring1, -m_speed);
			break;
		}
	}

	void Step(Settings& settings) override
	{
		g_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e");
		m_textLine += m_textIncrement;

        b2Vec2ConstRef pos = b2BodyGetPosition(m_car);
		g_camera.m_center[0] = pos[0];
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Car;
	}

	struct b2Body* m_car;
	struct b2Body* m_wheel1;
	struct b2Body* m_wheel2;

	float m_speed;
	struct b2JointWheel* m_spring1;
	struct b2JointWheel* m_spring2;
};

static int testIndex = RegisterTest("Examples", "Car", Car::Create);
