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

class BodyTypes : public Test
{
public:
	BodyTypes()
	{
        struct b2Body* ground = NULL;
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			ground = b2WorldCreateBody(m_world, &bd);

            b2Vec2 a;
            b2Vec2 b;
            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
            b2Vec2Make(a, -20.0f, 0.0f);
            b2Vec2Make(b, 20.0f, 0.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;

			b2BodyCreateFixtureFromDef(ground, &fd);
		}

		// Define attachment
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 0.0f, 3.0f);
			m_attachment = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.5f, 2.0f);
			b2BodyCreateFixtureFromShape(m_attachment, &shape, 2.0f);
		}

		// Define platform
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
            b2Vec2Make(bd.position, -4.0f, 5.0f);
			m_platform = b2WorldCreateBody(m_world, &bd);

            b2Vec2 center = { 4.0f, 0.0f };
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
            b2ShapePolygonSetAsBoxDetail(&shape, 0.5f, 4.0f, center, 0.5f * b2_pi);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.friction = 0.6f;
			fd.density = 2.0f;
			b2BodyCreateFixtureFromDef(m_platform, &fd);

            b2Vec2 anchor1 = { 0.0f, 5.0f };
            struct b2JointRevoluteDef rjd;
            b2JointRevoluteDefReset(&rjd);
			b2JointRevoluteDefInitialize(&rjd, m_attachment, m_platform, anchor1);
			rjd.maxMotorTorque = 50.0f;
			rjd.enableMotor = true;
			b2WorldCreateJoint(m_world, &rjd);

            const b2Vec2 anchor = { 0.0f, 5.0f };
            const b2Vec2 axis = { 1.0f, 0.0f };
            struct b2JointPrismaticDef pjd;
            b2JointPrismaticDefReset(&pjd);
			b2JointPrismaticDefInitialize(&pjd, ground, m_platform, anchor, axis);

			pjd.maxMotorForce = 1000.0f;
			pjd.enableMotor = true;
			pjd.lowerTranslation = -10.0f;
			pjd.upperTranslation = 10.0f;
			pjd.enableLimit = true;

            b2WorldCreateJoint(m_world, &pjd);

			m_speed = 3.0f;
		}

		// Create a payload
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position ,0.0f, 8.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.75f, 0.75f);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.friction = 0.6f;
			fd.density = 2.0f;

			b2BodyCreateFixtureFromDef(body, &fd);
		}
	}

	void Keyboard(int key) override
	{
        b2Vec2 LinearVelocity = { -m_speed, 0.0f };
		switch (key)
		{
		case GLFW_KEY_D:
			b2BodySetType(m_platform, b2BodyTypeDynamic);
			break;

		case GLFW_KEY_S:
			b2BodySetType(m_platform, b2BodyTypeStatic);
			break;

		case GLFW_KEY_K:
			b2BodySetType(m_platform, b2BodyTypeKinematic);
			b2BodySetLinearVelocity(m_platform, LinearVelocity);
			b2BodySetAngularVelocity(m_platform, 0.0f);
			break;
		}
	}

	void Step(Settings& settings) override
	{
		// Drive the kinematic body.
		if (b2BodyGetType(m_platform) == b2BodyTypeKinematic)
		{
            b2TransformConstRef tf = b2BodyGetTransform(m_platform);
			b2Vec2ConstRef p = tf[0];
			b2Vec2ConstRef vr = b2BodyGetLinearVelocity(m_platform);
            b2Vec2 v;
            b2Vec2Assign(v, vr);

			if ((p[0] < -10.0f && v[0] < 0.0f) ||
				(p[0] > 10.0f && v[0] > 0.0f))
			{
				v[0] = -v[0];
				b2BodySetLinearVelocity(m_platform, v);
			}
		}

		Test::Step(settings);

		g_debugDraw.DrawString(5, m_textLine, "Keys: (d) dynamic, (s) static, (k) kinematic");
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new BodyTypes;
	}

    struct b2Body* m_attachment;
    struct b2Body* m_platform;
	float m_speed;
};

static int testIndex = RegisterTest("Examples", "Body Types", BodyTypes::Create);
