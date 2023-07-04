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

class Tumbler : public Test
{
public:

	enum
	{
		e_count = 800
	};

	Tumbler()
	{
		struct b2Body* ground = NULL;
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			ground = b2WorldCreateBody(m_world, &bd);
		}

		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			bd.allowSleep = false;
			b2Vec2Make(bd.position, 0.0f, 10.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            b2Vec2 center;
            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
            b2Vec2Make(center, 10.0f, 0.0f);
			b2ShapePolygonSetAsBoxDetail(&shape, 0.5f, 10.0f, center, 0.0);
			b2BodyCreateFixtureFromShape(body, &shape, 5.0f);
            b2Vec2Make(center, -10.0f, 0.0f);
			b2ShapePolygonSetAsBoxDetail(&shape, 0.5f, 10.0f, center, 0.0);
			b2BodyCreateFixtureFromShape(body, &shape, 5.0f);
            b2Vec2Make(center, 0.0f, 10.0f);
			b2ShapePolygonSetAsBoxDetail(&shape, 10.0f, 0.5f, center, 0.0);
			b2BodyCreateFixtureFromShape(body, &shape, 5.0f);
            b2Vec2Make(center, 0.0f, -10.0f);
			b2ShapePolygonSetAsBoxDetail(&shape, 10.0f, 0.5f, center, 0.0);
			b2BodyCreateFixtureFromShape(body, &shape, 5.0f);

            struct b2JointRevoluteDef jd;
            b2JointRevoluteDefReset(&jd);
			jd.bodyA = ground;
			jd.bodyB = body;
			b2Vec2Make(jd.localAnchorA, 0.0f, 10.0f);
            b2Vec2Make(jd.localAnchorB, 0.0f, 0.0f);
			jd.referenceAngle = 0.0f;
			jd.motorSpeed = 0.05f * b2_pi;
			jd.maxMotorTorque = 1e8f;
			jd.enableMotor = true;
			m_joint = (struct b2JointRevolute*)b2WorldCreateJoint(m_world, &jd);
		}

		m_count = 0;
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		if (m_count < e_count)
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 0.0f, 10.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.125f, 0.125f);
			b2BodyCreateFixtureFromShape(body, &shape, 1.0f);

			++m_count;
		}
	}

	static Test* Create()
	{
		return new Tumbler;
	}

	struct b2JointRevolute* m_joint;
	int32 m_count;
};

static int testIndex = RegisterTest("Benchmark", "Tumbler", Tumbler::Create);
