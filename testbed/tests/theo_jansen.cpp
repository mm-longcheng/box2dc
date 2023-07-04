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

// Inspired by a contribution from roman_m
// Dimensions scooped from APE (http://www.cove.org/ape/index.htm)

#include "test.h"

class TheoJansen : public Test
{
public:

	void CreateLeg(float s, const b2Vec2& wheelAnchor)
	{
        b2Vec2 p1 = { 5.4f * s, -6.1f };
        b2Vec2 p2 = { 7.2f * s, -1.2f };
		b2Vec2 p3 = { 4.3f * s, -1.9f };
		b2Vec2 p4 = { 3.1f * s, 0.8f };
		b2Vec2 p5 = { 6.0f * s, 1.5f };
		b2Vec2 p6 = { 2.5f * s, 3.7f };

		struct b2FixtureDef fd1, fd2;
        b2FixtureDefReset(&fd1);
        b2FixtureDefReset(&fd2);
        fd1.filter.groupIndex = -1;
		fd2.filter.groupIndex = -1;
		fd1.density = 1.0f;
		fd2.density = 1.0f;

        struct b2ShapePolygon poly1, poly2;
        b2ShapePolygonReset(&poly1);
        b2ShapePolygonReset(&poly2);

		if (s > 0.0f)
		{
			b2Vec2 vertices[3];

			b2Vec2Assign(vertices[0], p1);
			b2Vec2Assign(vertices[1], p2);
			b2Vec2Assign(vertices[2], p3);
			b2ShapePolygonSetPoints(&poly1, vertices, 3);

			b2Vec2Assign(vertices[0], b2Vec2Zero);
			b2Vec2Sub(vertices[1], p5, p4);
            b2Vec2Sub(vertices[2], p6, p4);
			b2ShapePolygonSetPoints(&poly2, vertices, 3);
		}
		else
		{
			b2Vec2 vertices[3];

            b2Vec2Assign(vertices[0], p1);
            b2Vec2Assign(vertices[1], p3);
            b2Vec2Assign(vertices[2], p2);
			b2ShapePolygonSetPoints(&poly1, vertices, 3);

            b2Vec2Assign(vertices[0], b2Vec2Zero);
            b2Vec2Sub(vertices[1], p6, p4);
            b2Vec2Sub(vertices[2], p5, p4);
            b2ShapePolygonSetPoints(&poly2, vertices, 3);
		}

		fd1.shape = (struct b2Shape*)&poly1;
		fd2.shape = (struct b2Shape*)&poly2;

        struct b2BodyDef bd1, bd2;
        b2BodyDefReset(&bd1);
        b2BodyDefReset(&bd2);
        bd1.type = b2BodyTypeDynamic;
		bd2.type = b2BodyTypeDynamic;
		b2Vec2Assign(bd1.position, m_offset);
		b2Vec2Add(bd2.position, p4, m_offset);

		bd1.angularDamping = 10.0f;
		bd2.angularDamping = 10.0f;

        struct b2Body* body1 = b2WorldCreateBody(m_world, &bd1);
        struct b2Body* body2 = b2WorldCreateBody(m_world, &bd2);

		b2BodyCreateFixtureFromDef(body1, &fd1);
		b2BodyCreateFixtureFromDef(body2, &fd2);

		{
            struct b2JointDistanceDef jd;
            b2JointDistanceDefReset(&jd);

			// Using a soft distance constraint can reduce some jitter.
			// It also makes the structure seem a bit more fluid by
			// acting like a suspension system.
			float dampingRatio = 0.5f;
			float frequencyHz = 10.0f;

            b2Vec2 v1, v2;

            b2Vec2Add(v1, p2, m_offset);
            b2Vec2Add(v2, p5, m_offset);
            b2JointDistanceDefInitialize(&jd, body1, body2, v1, v2);
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            b2WorldCreateJoint(m_world, &jd);

            b2Vec2Add(v1, p3, m_offset);
            b2Vec2Add(v2, p4, m_offset);
			b2JointDistanceDefInitialize(&jd, body1, body2, v1, v2);
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            b2WorldCreateJoint(m_world, &jd);

            b2Vec2Add(v1, p3, m_offset);
            b2Vec2Add(v2, wheelAnchor, m_offset);
			b2JointDistanceDefInitialize(&jd, body1, m_wheel, v1, v2);
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            b2WorldCreateJoint(m_world, &jd);

            b2Vec2Add(v1, p6, m_offset);
            b2Vec2Add(v2, wheelAnchor, m_offset);
			b2JointDistanceDefInitialize(&jd, body2, m_wheel, v1, v2);
			b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);
            b2WorldCreateJoint(m_world, &jd);
        }

		{
            b2Vec2 v1;
            struct b2JointRevoluteDef jd;
            b2JointRevoluteDefReset(&jd);
            b2Vec2Add(v1, p4, m_offset);
			b2JointRevoluteDefInitialize(&jd, body2, m_chassis, v1);
            b2WorldCreateJoint(m_world, &jd);
		}
	}

	TheoJansen()
	{
		b2Vec2Make(m_offset, 0.0f, 8.0f);
		m_motorSpeed = 2.0f;
		m_motorOn = true;
        b2Vec2 pivot = { 0.0f, 0.8f };

		// Ground
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

            b2Vec2 a;
            b2Vec2 b;
            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
            b2Vec2Make(a, -50.0f, 0.0f);
            b2Vec2Make(b, 50.0f, 0.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

            b2Vec2Make(a, -50.0f, 0.0f);
            b2Vec2Make(b, -50.0f, 10.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

            b2Vec2Make(a, 50.0f, 0.0f);
            b2Vec2Make(b, 50.0f, 10.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
        }

		// Balls
		for (int32 i = 0; i < 40; ++i)
		{
            struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
			shape.m_radius = 0.25f;

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
            b2Vec2Make(bd.position, -40.0f + 2.0f * i, 0.5f);

            struct b2Body* body = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromShape(body, &shape, 1.0f);
		}

		// Chassis
		{
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 2.5f, 1.0f);

            struct b2FixtureDef sd;
            b2FixtureDefReset(&sd);
			sd.density = 1.0f;
			sd.shape = (struct b2Shape*)&shape;
			sd.filter.groupIndex = -1;
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
            b2Vec2Add(bd.position, pivot, m_offset);
			m_chassis = b2WorldCreateBody(m_world, &bd);
            b2BodyCreateFixtureFromDef(m_chassis, &sd);
		}

		{
            struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
			shape.m_radius = 1.6f;

            struct b2FixtureDef sd;
            b2FixtureDefReset(&sd);
			sd.density = 1.0f;
			sd.shape = (struct b2Shape*)&shape;
			sd.filter.groupIndex = -1;
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Add(bd.position, pivot, m_offset);
			m_wheel = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromDef(m_wheel, &sd);
		}

		{
            b2Vec2 v;
            struct b2JointRevoluteDef jd;
            b2JointRevoluteDefReset(&jd);
            b2Vec2Add(v, pivot, m_offset);
			b2JointRevoluteDefInitialize(&jd, m_wheel, m_chassis, v);
			jd.collideConnected = false;
			jd.motorSpeed = m_motorSpeed;
			jd.maxMotorTorque = 400.0f;
			jd.enableMotor = m_motorOn;
			m_motorJoint = (struct b2JointRevolute*)b2WorldCreateJoint(m_world, &jd);
		}

        b2Vec2 v = { 0.0f, -0.8f };
        b2Vec2 wheelAnchor;

        b2Vec2Add(wheelAnchor, pivot, v);

		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);

		b2BodySetTransform(m_wheel, b2BodyGetPosition(m_wheel), 120.0f * b2_pi / 180.0f);
		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);

		b2BodySetTransform(m_wheel, b2BodyGetPosition(m_wheel), -120.0f * b2_pi / 180.0f);
		CreateLeg(-1.0f, wheelAnchor);
		CreateLeg(1.0f, wheelAnchor);
	}

	void Step(Settings& settings) override
	{
		g_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d, toggle motor = m");
		m_textLine += m_textIncrement;

		Test::Step(settings);
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
            b2JointRevoluteSetMotorSpeed(m_motorJoint, -m_motorSpeed);
            //m_motorJoint->(structb2RevoluteJoint*)SetMotorSpeed(-m_motorSpeed);
			break;

		case GLFW_KEY_S:
            b2JointRevoluteSetMotorSpeed(m_motorJoint, 0.0f);
			break;

		case GLFW_KEY_D:
            b2JointRevoluteSetMotorSpeed(m_motorJoint, m_motorSpeed);
			break;

		case GLFW_KEY_M:
            b2JointRevoluteEnableMotor(m_motorJoint, !b2JointRevoluteIsMotorEnabled(m_motorJoint));
            break;
		}
	}

	static Test* Create()
	{
		return new TheoJansen;
	}

	b2Vec2 m_offset;
    struct b2Body* m_chassis;
    struct b2Body* m_wheel;
	struct b2JointRevolute* m_motorJoint;
	bool m_motorOn;
	float m_motorSpeed;
};

static int testIndex = RegisterTest("Examples", "Theo Jansen", TheoJansen::Create);
