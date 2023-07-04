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

#include <algorithm>

// This test shows collision processing and tests
// deferred body destruction.
class CollisionProcessing : public Test
{
public:
	CollisionProcessing()
	{
		// Ground body
		{
            b2Vec2 a = { -50.0f, 0.0f };
            b2Vec2 b = { 50.0f, 0.0f };
			struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
			b2ShapeEdgeSetTwoSided(&shape, a, b);

            struct b2FixtureDef sd;
            b2FixtureDefReset(&sd);
			sd.shape = (struct b2Shape*)&shape;

            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromDef(ground, &sd);
		}

		float xLo = -5.0f, xHi = 5.0f;
		float yLo = 2.0f, yHi = 35.0f;

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

        struct b2BodyDef triangleBodyDef;
        b2BodyDefReset(&triangleBodyDef);
		triangleBodyDef.type = b2BodyTypeDynamic;
		b2Vec2Make(triangleBodyDef.position, RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

        struct b2Body* body1 = b2WorldCreateBody(m_world, &triangleBodyDef);
		b2BodyCreateFixtureFromDef(body1, &triangleShapeDef);

		// Large triangle (recycle definitions)
        b2Vec2Scale(vertices[0], vertices[0], 2.0f);
        b2Vec2Scale(vertices[1], vertices[1], 2.0f);
        b2Vec2Scale(vertices[2], vertices[2], 2.0f);
		b2ShapePolygonSetPoints(&polygon, vertices, 3);

		b2Vec2Make(triangleBodyDef.position, RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

        struct b2Body* body2 = b2WorldCreateBody(m_world, &triangleBodyDef);
		b2BodyCreateFixtureFromDef(body2, &triangleShapeDef);
		
		// Small box
		b2ShapePolygonSetAsBox(&polygon, 1.0f, 0.5f);

        struct b2FixtureDef boxShapeDef;
        b2FixtureDefReset(&boxShapeDef);
		boxShapeDef.shape = (struct b2Shape*)&polygon;
		boxShapeDef.density = 1.0f;

        struct b2BodyDef boxBodyDef;
        b2BodyDefReset(&boxBodyDef);
		boxBodyDef.type = b2BodyTypeDynamic;
		b2Vec2Make(boxBodyDef.position, RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

        struct b2Body* body3 = b2WorldCreateBody(m_world, &boxBodyDef);
		b2BodyCreateFixtureFromDef(body3, &boxShapeDef);

		// Large box (recycle definitions)
		b2ShapePolygonSetAsBox(&polygon, 2.0f, 1.0f);
		b2Vec2Make(boxBodyDef.position, RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));
		
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

        struct b2BodyDef circleBodyDef;
        b2BodyDefReset(&circleBodyDef);
		circleBodyDef.type = b2BodyTypeDynamic;
		b2Vec2Make(circleBodyDef.position, RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

        struct b2Body* body5 = b2WorldCreateBody(m_world, &circleBodyDef);
		b2BodyCreateFixtureFromDef(body5, &circleShapeDef);

		// Large circle
		circle.m_radius *= 2.0f;
		b2Vec2Make(circleBodyDef.position, RandomFloat(xLo, xHi), RandomFloat(yLo, yHi));

        struct b2Body* body6 = b2WorldCreateBody(m_world, &circleBodyDef);
		b2BodyCreateFixtureFromDef(body6, &circleShapeDef);
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		// We are going to destroy some bodies according to contact
		// points. We must buffer the bodies that should be destroyed
		// because they may belong to multiple contact points.
		const int32 k_maxNuke = 6;
        struct b2Body* nuke[k_maxNuke];
		int32 nukeCount = 0;

		// Traverse the contact results. Destroy bodies that
		// are touching heavier bodies.
		for (int32 i = 0; i < m_pointCount; ++i)
		{
			ContactPoint* point = m_points + i;

			struct b2Body* body1 = b2FixtureGetBodyRef(point->fixtureA);
            struct b2Body* body2 = b2FixtureGetBodyRef(point->fixtureB);
			float mass1 = b2BodyGetMass(body1);
			float mass2 = b2BodyGetMass(body2);

			if (mass1 > 0.0f && mass2 > 0.0f)
			{
				if (mass2 > mass1)
				{
					nuke[nukeCount++] = body1;
				}
				else
				{
					nuke[nukeCount++] = body2;
				}

				if (nukeCount == k_maxNuke)
				{
					break;
				}
			}
		}

		// Sort the nuke array to group duplicates.
		std::sort(nuke, nuke + nukeCount);

		// Destroy the bodies, skipping duplicates.
		int32 i = 0;
		while (i < nukeCount)
		{
			b2Body* b = nuke[i++];
			while (i < nukeCount && nuke[i] == b)
			{
				++i;
			}

			if (b != m_bomb)
			{
				b2WorldDeleteBody(m_world, b);
			}
		}
	}

	static Test* Create()
	{
		return new CollisionProcessing;
	}
};

static int testIndex = RegisterTest("Examples", "Collision Processing", CollisionProcessing::Create);
