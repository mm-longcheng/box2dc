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

#include <string.h>

/// This tests stacking. It also shows how to use b2World::Query
/// and b2TestOverlap.

extern const struct b2QueryCallbackMeta b2QueryCallbackMetaPolygonShapes;

/// This callback is called by b2World::QueryAABB. We find all the fixtures
/// that overlap an AABB. Of those, we use b2TestOverlap to determine which fixtures
/// overlap a circle. Up to 4 overlapped fixtures will be highlighted with a yellow border.
class PolygonShapesCallback : public b2QueryCallback
{
public:
	
	enum
	{
		e_maxCount = 4
	};

	PolygonShapesCallback()
	{
        this->m = &b2QueryCallbackMetaPolygonShapes;
        this->o = this;

        b2ShapeCircleReset(&this->m_circle);

		m_count = 0;
	}

	struct b2ShapeCircle m_circle;
	b2Transform m_transform;
    struct b2Draw* g_debugDraw;
	int32 m_count;
};

/// Called for each fixture found in the query AABB.
/// @return false to terminate the query.
int
PolygonShapesReportFixture(
    PolygonShapesCallback* p,
    struct b2Fixture* fixture)
{
    if (p->m_count == PolygonShapesCallback::e_maxCount)
    {
        return false;
    }

    struct b2Body* body = b2FixtureGetBodyRef(fixture);
    struct b2Shape* shape = b2FixtureGetShapeRef(fixture);

    bool overlap = b2TestOverlap(shape, 0, (const struct b2Shape*)&p->m_circle, 0, b2BodyGetTransform(body), p->m_transform);

    if (overlap)
    {
        b2Color color = { 0.95f, 0.95f, 0.6f, 1.0f };
        b2Vec2ConstRef center = b2BodyGetWorldCenter(body);
        b2DrawPoint(p->g_debugDraw, center, 5.0f, color);
        ++p->m_count;
    }

    return true;
}

const struct b2QueryCallbackMeta b2QueryCallbackMetaPolygonShapes = 
{
    &PolygonShapesReportFixture,
};

class PolygonShapes : public Test
{
public:

	enum
	{
		e_maxBodies = 256
	};

	PolygonShapes()
	{
        b2ShapePolygonReset(&this->m_polygons[0]);
        b2ShapePolygonReset(&this->m_polygons[1]);
        b2ShapePolygonReset(&this->m_polygons[2]);
        b2ShapePolygonReset(&this->m_polygons[3]);

        b2ShapeCircleReset(&this->m_circle);

		// Ground body
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

            b2Vec2 a = { -40.0f, 0.0f };
            b2Vec2 b = { 40.0f, 0.0f };
            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
			b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

		{
			b2Vec2 vertices[3];
			b2Vec2Make(vertices[0], -0.5f, 0.0f);
			b2Vec2Make(vertices[1], 0.5f, 0.0f);
			b2Vec2Make(vertices[2], 0.0f, 1.5f);
			b2ShapePolygonSetPoints(&m_polygons[0], vertices, 3);
		}
		
		{
			b2Vec2 vertices[3];
			b2Vec2Make(vertices[0], -0.1f, 0.0f);
			b2Vec2Make(vertices[1], 0.1f, 0.0f);
			b2Vec2Make(vertices[2], 0.0f, 1.5f);
			b2ShapePolygonSetPoints(&m_polygons[1], vertices, 3);
		}

		{
			float w = 1.0f;
			float b = w / (2.0f + b2Sqrt(2.0f));
			float s = b2Sqrt(2.0f) * b;

			b2Vec2 vertices[8];
			b2Vec2Make(vertices[0], 0.5f * s, 0.0f);
			b2Vec2Make(vertices[1], 0.5f * w, b);
			b2Vec2Make(vertices[2], 0.5f * w, b + s);
			b2Vec2Make(vertices[3], 0.5f * s, w);
			b2Vec2Make(vertices[4], -0.5f * s, w);
			b2Vec2Make(vertices[5], -0.5f * w, b + s);
			b2Vec2Make(vertices[6], -0.5f * w, b);
			b2Vec2Make(vertices[7], -0.5f * s, 0.0f);

			b2ShapePolygonSetPoints(&m_polygons[2], vertices, 8);
		}

		{
			b2ShapePolygonSetAsBox(&m_polygons[3], 0.5f, 0.5f);
		}

		{
			m_circle.m_radius = 0.5f;
		}

		m_bodyIndex = 0;
		memset(m_bodies, 0, sizeof(m_bodies));
	}

	void Create(int32 index)
	{
		if (m_bodies[m_bodyIndex] != NULL)
		{
			b2WorldDeleteBody(m_world, m_bodies[m_bodyIndex]);
			m_bodies[m_bodyIndex] = NULL;
		}

		struct b2BodyDef bd;
        b2BodyDefReset(&bd);
		bd.type = b2BodyTypeDynamic;

		float x = RandomFloat(-2.0f, 2.0f);
		b2Vec2Make(bd.position, x, 10.0f);
		bd.angle = RandomFloat(-b2_pi, b2_pi);

		if (index == 4)
		{
			bd.angularDamping = 0.02f;
		}

		m_bodies[m_bodyIndex] = b2WorldCreateBody(m_world, &bd);

		if (index < 4)
		{
			struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)(m_polygons + index);
			fd.density = 1.0f;
			fd.friction = 0.3f;
			b2BodyCreateFixtureFromDef(m_bodies[m_bodyIndex], &fd);
		}
		else
		{
            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)(&m_circle);
			fd.density = 1.0f;
			fd.friction = 0.3f;

			b2BodyCreateFixtureFromDef(m_bodies[m_bodyIndex], &fd);
		}

		m_bodyIndex = (m_bodyIndex + 1) % e_maxBodies;
	}

	void DestroyBody()
	{
		for (int32 i = 0; i < e_maxBodies; ++i)
		{
			if (m_bodies[i] != NULL)
			{
				b2WorldDeleteBody(m_world, m_bodies[i]);
				m_bodies[i] = NULL;
				return;
			}
		}
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_1:
		case GLFW_KEY_2:
		case GLFW_KEY_3:
		case GLFW_KEY_4:
		case GLFW_KEY_5:
			Create(key - GLFW_KEY_1);
			break;

		case GLFW_KEY_A:
			for (int32 i = 0; i < e_maxBodies; i += 2)
			{
				if (m_bodies[i])
				{
					bool enabled = b2BodyIsEnabled(m_bodies[i]);
					b2BodySetEnabled(m_bodies[i], !enabled);
				}
			}
			break;

		case GLFW_KEY_D:
			DestroyBody();
			break;
		}
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		PolygonShapesCallback callback;
		callback.m_circle.m_radius = 2.0f;
		b2Vec2Make(callback.m_circle.m_p, 0.0f, 1.1f);
		b2TransformMakeIdentity(callback.m_transform);
		callback.g_debugDraw = &g_debugDraw;

		struct b2AABB aabb;
		b2ShapeCircleComputeAABB(&callback.m_circle, &aabb, callback.m_transform, 0);

		b2WorldQueryAABB(m_world, &callback, &aabb);

        b2Color color = { 0.4f, 0.7f, 0.8f, 1.0f };
		g_debugDraw.DrawCircle(callback.m_circle.m_p, callback.m_circle.m_radius, color);

		g_debugDraw.DrawString(5, m_textLine, "Press 1-5 to drop stuff, maximum of %d overlaps detected", PolygonShapesCallback::e_maxCount);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "Press 'a' to enable/disable some bodies");
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "Press 'd' to destroy a body");
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new PolygonShapes;
	}

	int32 m_bodyIndex;
	struct b2Body* m_bodies[e_maxBodies];
    struct b2ShapePolygon m_polygons[4];
    struct b2ShapeCircle m_circle;
};

static int testIndex = RegisterTest("Geometry", "Polygon Shapes", PolygonShapes::Create);
