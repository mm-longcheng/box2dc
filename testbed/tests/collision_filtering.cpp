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

// This is a test of collision filtering.
// There is a triangle, a box, and a circle.
// There are 6 shapes. 3 large and 3 small.
// The 3 small ones always collide.
// The 3 large ones never collide.
// The boxes don't collide with triangles (except if both are small).
const int16	k_smallGroup = 1;
const int16 k_largeGroup = -1;

const uint16 k_triangleCategory = 0x0002;
const uint16 k_boxCategory = 0x0004;
const uint16 k_circleCategory = 0x0008;

const uint16 k_triangleMask = 0xFFFF;
const uint16 k_boxMask = 0xFFFF ^ k_triangleCategory;
const uint16 k_circleMask = 0xFFFF;

class CollisionFiltering : public Test
{
public:
	CollisionFiltering()
	{
		// Ground body
		{
			struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
            b2Vec2 a = { -40.0f, 0.0f };
            b2Vec2 b = { 40.0f, 0.0f };
			b2ShapeEdgeSetTwoSided(&shape, a, b);

            struct b2FixtureDef sd;
            b2FixtureDefReset(&sd);
			sd.shape = (struct b2Shape*)&shape;
			sd.friction = 0.3f;

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromDef(ground, &sd);
		}

		// Small triangle
		b2Vec2 vertices[3];
		b2Vec2Make(vertices[0], -1.0f, 0.0f);
		b2Vec2Make(vertices[1], 1.0f, 0.0f);
		b2Vec2Make(vertices[2], 0.0f, 2.0f);
        struct b2ShapePolygon polygon;
        b2ShapePolygonReset(&polygon);
		b2ShapePolygonSetPoints(&polygon, vertices, 3);

        struct b2FixtureDef triangleShapeDef;
        b2FixtureDefReset(&triangleShapeDef);
		triangleShapeDef.shape = (struct b2Shape*)&polygon;
		triangleShapeDef.density = 1.0f;

		triangleShapeDef.filter.groupIndex = k_smallGroup;
		triangleShapeDef.filter.categoryBits = k_triangleCategory;
		triangleShapeDef.filter.maskBits = k_triangleMask;

        struct b2BodyDef triangleBodyDef;
        b2BodyDefReset(&triangleBodyDef);
		triangleBodyDef.type = b2BodyTypeDynamic;
		b2Vec2Make(triangleBodyDef.position, -5.0f, 2.0f);

        struct b2Body* body1 = b2WorldCreateBody(m_world, &triangleBodyDef);
		b2BodyCreateFixtureFromDef(body1, &triangleShapeDef);

		// Large triangle (recycle definitions)
		b2Vec2Scale(vertices[0], vertices[0], 2.0f);
        b2Vec2Scale(vertices[1], vertices[1], 2.0f);
        b2Vec2Scale(vertices[2], vertices[2], 2.0f);
		b2ShapePolygonSetPoints(&polygon, vertices, 3);
		triangleShapeDef.filter.groupIndex = k_largeGroup;
		b2Vec2Make(triangleBodyDef.position, -5.0f, 6.0f);
		triangleBodyDef.fixedRotation = true; // look at me!

        struct b2Body* body2 = b2WorldCreateBody(m_world, &triangleBodyDef);
		b2BodyCreateFixtureFromDef(body2, &triangleShapeDef);

		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, -5.0f, 10.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon p;
            b2ShapePolygonReset(&p);
			b2ShapePolygonSetAsBox(&p, 0.5f, 1.0f);
            b2BodyCreateFixtureFromShape(body, &p, 1.0f);

            struct b2JointPrismaticDef jd;
            b2JointPrismaticDefReset(&jd);
			jd.bodyA = body2;
			jd.bodyB = body;
			jd.enableLimit = true;
			b2Vec2Make(jd.localAnchorA, 0.0f, 4.0f);
			b2Vec2SetZero(jd.localAnchorB);
            b2Vec2Make(jd.localAxisA, 0.0f, 1.0f);
			jd.lowerTranslation = -1.0f;
			jd.upperTranslation = 1.0f;

			b2WorldCreateJoint(m_world, &jd);
		}

		// Small box
		b2ShapePolygonSetAsBox(&polygon, 1.0f, 0.5f);
        struct b2FixtureDef boxShapeDef;
        b2FixtureDefReset(&boxShapeDef);
		boxShapeDef.shape = (struct b2Shape*)&polygon;
		boxShapeDef.density = 1.0f;
		boxShapeDef.restitution = 0.1f;

		boxShapeDef.filter.groupIndex = k_smallGroup;
		boxShapeDef.filter.categoryBits = k_boxCategory;
		boxShapeDef.filter.maskBits = k_boxMask;

        struct b2BodyDef boxBodyDef;
        b2BodyDefReset(&boxBodyDef);
		boxBodyDef.type = b2BodyTypeDynamic;
		b2Vec2Make(boxBodyDef.position, 0.0f, 2.0f);

        struct b2Body* body3 = b2WorldCreateBody(m_world, &boxBodyDef);
		b2BodyCreateFixtureFromDef(body3, &boxShapeDef);

		// Large box (recycle definitions)
		b2ShapePolygonSetAsBox(&polygon, 2.0f, 1.0f);
		boxShapeDef.filter.groupIndex = k_largeGroup;
		b2Vec2Make(boxBodyDef.position, 0.0f, 6.0f);

        struct b2Body* body4 = b2WorldCreateBody(m_world, &boxBodyDef);
		b2BodyCreateFixtureFromDef(body4, &boxShapeDef);

		// Small circle
        struct b2ShapeCircle circle;
        b2ShapeCircleReset(&circle);
		circle.m_radius = 1.0f;

        struct b2FixtureDef circleShapeDef;
        b2FixtureDefReset(&circleShapeDef);
		circleShapeDef.shape = (struct b2Shape*)&circle;
		circleShapeDef.density = 1.0f;

		circleShapeDef.filter.groupIndex = k_smallGroup;
		circleShapeDef.filter.categoryBits = k_circleCategory;
		circleShapeDef.filter.maskBits = k_circleMask;

        struct b2BodyDef circleBodyDef;
        b2BodyDefReset(&circleBodyDef);
		circleBodyDef.type = b2BodyTypeDynamic;
		b2Vec2Make(circleBodyDef.position, 5.0f, 2.0f);
		
        struct b2Body* body5 = b2WorldCreateBody(m_world, &circleBodyDef);
		b2BodyCreateFixtureFromDef(body5, &circleShapeDef);

		// Large circle
		circle.m_radius *= 2.0f;
		circleShapeDef.filter.groupIndex = k_largeGroup;
		b2Vec2Make(circleBodyDef.position, 5.0f, 6.0f);

        struct b2Body* body6 = b2WorldCreateBody(m_world, &circleBodyDef);
		b2BodyCreateFixtureFromDef(body6, &circleShapeDef);
	}

	static Test* Create()
	{
		return new CollisionFiltering;
	}
};

static int testIndex = RegisterTest("Examples", "Collision Filtering", CollisionFiltering::Create);
