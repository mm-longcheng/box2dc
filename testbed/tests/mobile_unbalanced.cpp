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

class MobileUnbalanced : public Test
{
public:

	enum
	{
		e_depth = 4
	};

	MobileUnbalanced()
	{
		struct b2Body* ground;

		// Create ground body.
		{
            struct b2BodyDef bodyDef;
            b2BodyDefReset(&bodyDef);
			b2Vec2Make(bodyDef.position, 0.0f, 20.0f);
			ground = b2WorldCreateBody(m_world, &bodyDef);
		}

		float a = 0.5f;
        b2Vec2 h = { 0.0f, a };

        struct b2Body* root = AddNode(ground, b2Vec2Zero, 0, 3.0f, a);

        struct b2JointRevoluteDef jointDef;
        b2JointRevoluteDefReset(&jointDef);
		jointDef.bodyA = ground;
		jointDef.bodyB = root;
		b2Vec2SetZero(jointDef.localAnchorA);
		b2Vec2Assign(jointDef.localAnchorB, h);
		b2WorldCreateJoint(m_world, &jointDef);
	}

    struct b2Body* AddNode(struct b2Body* parent, const b2Vec2 localAnchor, int32 depth, float offset, float a)
	{
		float density = 20.0f;
        b2Vec2 h = { 0.0f, a };

        b2Vec2 p;
        b2Vec2Add(p, b2BodyGetPosition(parent), localAnchor);
        b2Vec2Sub(p, p, h);

        struct b2BodyDef bodyDef;
        b2BodyDefReset(&bodyDef);
		bodyDef.type = b2BodyTypeDynamic;
		b2Vec2Assign(bodyDef.position, p);
        struct b2Body* body = b2WorldCreateBody(m_world, &bodyDef);

        struct b2ShapePolygon shape;
        b2ShapePolygonReset(&shape);
		b2ShapePolygonSetAsBox(&shape, 0.25f * a, a);
		b2BodyCreateFixtureFromShape(body, &shape, density);

		if (depth == e_depth)
		{
			return body;
		}

        b2Vec2 a1 = { offset, -a };
        b2Vec2 a2 = { -offset, -a };
        struct b2Body* body1 = AddNode(body, a1, depth + 1, 0.5f * offset, a);
        struct b2Body* body2 = AddNode(body, a2, depth + 1, 0.5f * offset, a);

        struct b2JointRevoluteDef jointDef;
        b2JointRevoluteDefReset(&jointDef);
		jointDef.bodyA = body;
		b2Vec2Assign(jointDef.localAnchorB, h);

		b2Vec2Assign(jointDef.localAnchorA, a1);
		jointDef.bodyB = body1;
        b2WorldCreateJoint(m_world, &jointDef);

		b2Vec2Assign(jointDef.localAnchorA, a2);
		jointDef.bodyB = body2;
		b2WorldCreateJoint(m_world, &jointDef);

		return body;
	}

	static Test* Create()
	{
		return new MobileUnbalanced;
	}
};

static int testIndex = RegisterTest("Solver", "Mobile Unbalanced", MobileUnbalanced::Create);
