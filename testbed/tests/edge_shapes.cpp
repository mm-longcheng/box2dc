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

#include "settings.h"
#include "test.h"

#include <string.h>

extern const struct b2RayCastCallbackMeta b2RayCastCallbackMetaEdgeShapes;

class EdgeShapesCallback : public b2RayCastCallback
{
public:
	EdgeShapesCallback()
	{
        this->m = &b2RayCastCallbackMetaEdgeShapes;
        this->o = this;

		m_fixture = NULL;
	}

	b2Fixture* m_fixture;
	b2Vec2 m_point;
	b2Vec2 m_normal;
};

float
EdgeShapesCallbackReportFixture(
    EdgeShapesCallback* p,
    struct b2Fixture* fixture,
    const b2Vec2 point,
    const b2Vec2 normal,
    float fraction)
{
    p->m_fixture = fixture;
    b2Vec2Assign(p->m_point, point);
    b2Vec2Assign(p->m_normal, normal);

    return fraction;
}

const struct b2RayCastCallbackMeta b2RayCastCallbackMetaEdgeShapes =
{
    &EdgeShapesCallbackReportFixture,
};

class EdgeShapes : public Test
{
public:

	enum
	{
		e_maxBodies = 256
	};

	EdgeShapes()
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

			float x1 = -20.0f;
			float y1 = 2.0f * cosf(x1 / 10.0f * b2_pi);
			for (int32 i = 0; i < 80; ++i)
			{
				float x2 = x1 + 0.5f;
				float y2 = 2.0f * cosf(x2 / 10.0f * b2_pi);

                b2Vec2 a = { x1, y1 };
                b2Vec2 b = { x2, y2 };
                struct b2ShapeEdge shape;
                b2ShapeEdgeReset(&shape);
				b2ShapeEdgeSetTwoSided(&shape, a, b);
				b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

				x1 = x2;
				y1 = y2;
			}
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

		m_angle = 0.0f;
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
		float x = RandomFloat(-10.0f, 10.0f);
		float y = RandomFloat(10.0f, 20.0f);
		b2Vec2Make(bd.position, x, y);
		bd.angle = RandomFloat(-b2_pi, b2_pi);
		bd.type = b2BodyTypeDynamic;

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
			fd.friction = 0.3f;
			fd.density = 20.0f;
			b2BodyCreateFixtureFromDef(m_bodies[m_bodyIndex], &fd);
		}
		else
		{
			struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&m_circle;
			fd.friction = 0.3f;
			fd.density = 20.0f;
            b2BodyCreateFixtureFromDef(m_bodies[m_bodyIndex] , &fd);
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

		case GLFW_KEY_D:
			DestroyBody();
			break;
		}
	}

	void Step(Settings& settings) override
	{
		bool advanceRay = settings.m_pause == 0 || settings.m_singleStep;

		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Press 1-5 to drop stuff");
		m_textLine += m_textIncrement;

		float L = 25.0f;
        b2Vec2 point1 = { 0.0f, 10.0f };
        b2Vec2 d = { L * cosf(m_angle), -L * b2AbsFloat(sinf(m_angle)) };
        b2Vec2 point2;
        b2Vec2Add(point2, point1, d);

		EdgeShapesCallback callback;

		b2WorldRayCast(m_world, &callback, point1, point2);

		if (callback.m_fixture)
		{
            static const b2Color color1 = { 0.4f, 0.9f, 0.4f, 1.0f };
			g_debugDraw.DrawPoint(callback.m_point, 5.0f, color1);

            static const b2Color color2 = { 0.8f, 0.8f, 0.8f, 1.0f };
			g_debugDraw.DrawSegment(point1, callback.m_point, color2);

            static const b2Color color3 = { 0.9f, 0.9f, 0.4f, 1.0f };
            b2Vec2 v;
            b2Vec2 head;
            b2Vec2Scale(v, callback.m_normal, 0.5f);
            b2Vec2Add(head, callback.m_point, v);
			g_debugDraw.DrawSegment(callback.m_point, head, color3);
		}
		else
		{
            static const b2Color color4 = { 0.8f, 0.8f, 0.8f, 1.0f };
			g_debugDraw.DrawSegment(point1, point2, color4);
		}

		if (advanceRay)
		{
			m_angle += 0.25f * b2_pi / 180.0f;
		}
	}

	static Test* Create()
	{
		return new EdgeShapes;
	}

	int32 m_bodyIndex;
    struct b2Body* m_bodies[e_maxBodies];
	struct b2ShapePolygon m_polygons[4];
    struct b2ShapeCircle m_circle;

	float m_angle;
};

static int testIndex = RegisterTest("Geometry", "Edge Shapes", EdgeShapes::Create);
