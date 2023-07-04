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
#include "imgui/imgui.h"

enum
{
	e_maxBodies = 256
};

extern struct b2RayCastCallbackMeta b2RayCastCallbackMetaRayCastClosest;

// This test demonstrates how to use the world ray-cast feature.
// NOTE: we are intentionally filtering one of the polygons, therefore
// the ray will always miss one type of polygon.

// This callback finds the closest hit. Polygon 0 is filtered.
class RayCastClosestCallback : public b2RayCastCallback
{
public:
	RayCastClosestCallback()
	{
        this->m = &b2RayCastCallbackMetaRayCastClosest;
        this->o = this;

		m_hit = false;
	}
	
	bool m_hit;
	b2Vec2 m_point;
	b2Vec2 m_normal;
};

float 
RayCastClosestReportFixture(
    RayCastClosestCallback* p,
    struct b2Fixture* fixture,
    const b2Vec2 point,
    const b2Vec2 normal,
    float fraction)
{
    uintptr_t index = b2FixtureGetUserData(fixture);
    if (index == 1)
    {
        // By returning -1, we instruct the calling code to ignore this fixture and
        // continue the ray-cast to the next fixture.
        return -1.0f;
    }

    p->m_hit = true;
    b2Vec2Assign(p->m_point, point);
    b2Vec2Assign(p->m_normal, normal);

    // By returning the current fraction, we instruct the calling code to clip the ray and
    // continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
    // are reported in order. However, by clipping, we can always get the closest fixture.
    return fraction;
}

struct b2RayCastCallbackMeta b2RayCastCallbackMetaRayCastClosest =
{
    &RayCastClosestReportFixture,
};

extern struct b2RayCastCallbackMeta b2RayCastCallbackMetaRayCastAny;

// This callback finds any hit. Polygon 0 is filtered. For this type of query we are usually
// just checking for obstruction, so the actual fixture and hit point are irrelevant. 
class RayCastAnyCallback : public b2RayCastCallback
{
public:
	RayCastAnyCallback()
	{
        this->m = &b2RayCastCallbackMetaRayCastAny;
        this->o = this;

		m_hit = false;
	}

	bool m_hit;
	b2Vec2 m_point;
	b2Vec2 m_normal;
};

float 
RayCastAnyReportFixture(
    RayCastAnyCallback* p,
    struct b2Fixture* fixture,
    const b2Vec2 point,
    const b2Vec2 normal,
    float fraction)
{
    uintptr_t index = b2FixtureGetUserData(fixture);
    if (index == 1)
    {
        // By returning -1, we instruct the calling code to ignore this fixture and
        // continue the ray-cast to the next fixture.
        return -1.0f;
    }

    p->m_hit = true;
    b2Vec2Assign(p->m_point, point);
    b2Vec2Assign(p->m_normal, normal);

    // At this point we have a hit, so we know the ray is obstructed.
    // By returning 0, we instruct the calling code to terminate the ray-cast.
    return 0.0f;
}

struct b2RayCastCallbackMeta b2RayCastCallbackMetaRayCastAny =
{
    &RayCastAnyReportFixture,
};

extern struct b2RayCastCallbackMeta b2RayCastCallbackMetaRayCastMultiple;

// This ray cast collects multiple hits along the ray. Polygon 0 is filtered.
// The fixtures are not necessary reported in order, so we might not capture
// the closest fixture.
class RayCastMultipleCallback : public b2RayCastCallback
{
public:
	enum
	{
		e_maxCount = 3
	};

	RayCastMultipleCallback()
	{
        this->m = &b2RayCastCallbackMetaRayCastMultiple;
        this->o = this;

		m_count = 0;
	}

	b2Vec2 m_points[e_maxCount];
	b2Vec2 m_normals[e_maxCount];
	int32 m_count;
};

float
RayCastMultipleReportFixture(
    RayCastMultipleCallback* p,
    struct b2Fixture* fixture,
    const b2Vec2 point,
    const b2Vec2 normal,
    float fraction)
{
    uintptr_t index = b2FixtureGetUserData(fixture);
    if (index == 1)
    {
        // By returning -1, we instruct the calling code to ignore this fixture and
        // continue the ray-cast to the next fixture.
        return -1.0f;
    }

    b2Assert(p->m_count < RayCastMultipleCallback::e_maxCount);

    b2Vec2Assign(p->m_points[p->m_count], point);
    b2Vec2Assign(p->m_normals[p->m_count], normal);
    ++p->m_count;

    if (p->m_count == RayCastMultipleCallback::e_maxCount)
    {
        // At this point the buffer is full.
        // By returning 0, we instruct the calling code to terminate the ray-cast.
        return 0.0f;
    }

    // By returning 1, we instruct the caller to continue without clipping the ray.
    return 1.0f;
}

struct b2RayCastCallbackMeta b2RayCastCallbackMetaRayCastMultiple =
{
    &RayCastMultipleReportFixture,
};

class RayCast : public Test
{
public:

	enum Mode
	{
		e_any = 0,
		e_closest = 1,
		e_multiple = 2
	};

	RayCast()
	{
        b2ShapePolygonReset(&this->m_polygons[0]);
        b2ShapePolygonReset(&this->m_polygons[1]);
        b2ShapePolygonReset(&this->m_polygons[2]);
        b2ShapePolygonReset(&this->m_polygons[3]);

        b2ShapeCircleReset(&this->m_circle);

        b2ShapeEdgeReset(&this->m_edge);

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
			b2Vec2Make(vertices[0] ,-0.5f, 0.0f);
			b2Vec2Make(vertices[1] ,0.5f, 0.0f);
			b2Vec2Make(vertices[2] ,0.0f, 1.5f);
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

		{
            b2Vec2 a = { -1.0f, 0.0f };
            b2Vec2 b = { 1.0f, 0.0f };
			b2ShapeEdgeSetTwoSided(&m_edge, a, b);
		}

		m_bodyIndex = 0;
		memset(m_bodies, 0, sizeof(m_bodies));

		m_degrees = 0.0f;

		m_mode = e_closest;
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
		float y = RandomFloat(0.0f, 20.0f);
		b2Vec2Make(bd.position, x, y);
		bd.angle = RandomFloat(-b2_pi, b2_pi);

		if (index == 4)
		{
			bd.angularDamping = 0.02f;
		}

		m_bodies[m_bodyIndex] = b2WorldCreateBody(m_world, &bd);

		if (index < 4)
		{
			b2FixtureDef fd;
			fd.shape = (struct b2Shape*)(m_polygons + index);
			fd.friction = 0.3f;
			fd.userData = index + 1;
			b2BodyCreateFixtureFromDef(m_bodies[m_bodyIndex], &fd);
		}
		else if (index < 5)
		{
			b2FixtureDef fd;
			fd.shape = (struct b2Shape*)&m_circle;
			fd.friction = 0.3f;
			fd.userData = index + 1;
			b2BodyCreateFixtureFromDef(m_bodies[m_bodyIndex], &fd);
		}
		else
		{
			b2FixtureDef fd;
			fd.shape = (struct b2Shape*)&m_edge;
			fd.friction = 0.3f;
			fd.userData = index + 1;

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

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(210.0f, 285.0f));
		ImGui::Begin("Ray-cast Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Button("Shape 1"))
		{
			Create(0);
		}

		if (ImGui::Button("Shape 2"))
		{
			Create(1);
		}

		if (ImGui::Button("Shape 3"))
		{
			Create(2);
		}

		if (ImGui::Button("Shape 4"))
		{
			Create(3);
		}

		if (ImGui::Button("Shape 5"))
		{
			Create(4);
		}

		if (ImGui::Button("Shape 6"))
		{
			Create(5);
		}

		if (ImGui::Button("Destroy Shape"))
		{
			DestroyBody();
		}

		ImGui::RadioButton("Any", &m_mode, e_any);
		ImGui::RadioButton("Closest", &m_mode, e_closest);
		ImGui::RadioButton("Multiple", &m_mode, e_multiple);

		ImGui::SliderFloat("Angle", &m_degrees, 0.0f, 360.0f, "%.0f");

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		g_debugDraw.DrawString(5, m_textLine, "Shape 1 is intentionally ignored by the ray");
		m_textLine += m_textIncrement;
		switch (m_mode)
		{
		case e_closest:
			g_debugDraw.DrawString(5, m_textLine, "Ray-cast mode: closest - find closest fixture along the ray");
			break;
		
		case e_any:
			g_debugDraw.DrawString(5, m_textLine, "Ray-cast mode: any - check for obstruction");
			break;

		case e_multiple:
			g_debugDraw.DrawString(5, m_textLine, "Ray-cast mode: multiple - gather multiple fixtures");
			break;
		}

		m_textLine += m_textIncrement;

		float angle = b2_pi * m_degrees / 180.0f;
		float L = 11.0f;
        b2Vec2 point1 = { 0.0f, 10.0f };
        b2Vec2 d = { L * cosf(angle), L * sinf(angle) };
        b2Vec2 point2;
        b2Vec2Add(point2, point1, d);

		if (m_mode == e_closest)
		{
			RayCastClosestCallback callback;
			b2WorldRayCast(m_world, &callback, point1, point2);

			if (callback.m_hit)
			{
                b2Color color1 = { 0.4f, 0.9f, 0.4f, 1.0f };
                b2Color color2 = { 0.8f, 0.8f, 0.8f, 1.0f };
                b2Color color3 = { 0.9f, 0.9f, 0.4f, 1.0f };
                g_debugDraw.DrawPoint(callback.m_point, 5.0f, color1);
				g_debugDraw.DrawSegment(point1, callback.m_point, color2);
                b2Vec2 head;
                b2Vec2Scale(head, callback.m_normal, 0.5f);
                b2Vec2Add(head, callback.m_point, head);
				g_debugDraw.DrawSegment(callback.m_point, head, color3);
			}
			else
			{
                b2Color color4 = { 0.8f, 0.8f, 0.8f, 1.0f };
				g_debugDraw.DrawSegment(point1, point2, color4);
			}
		}
		else if (m_mode == e_any)
		{
			RayCastAnyCallback callback;
			b2WorldRayCast(m_world, &callback, point1, point2);

			if (callback.m_hit)
			{
                b2Color color1 = { 0.4f, 0.9f, 0.4f, 1.0f };
                b2Color color2 = { 0.8f, 0.8f, 0.8f, 1.0f };
                b2Color color3 = { 0.9f, 0.9f, 0.4f, 1.0f };
                g_debugDraw.DrawPoint(callback.m_point, 5.0f, color1);
				g_debugDraw.DrawSegment(point1, callback.m_point, color2);
                b2Vec2 head;
                b2Vec2Scale(head, callback.m_normal, 0.5f);
                b2Vec2Add(head, callback.m_point, head);
				g_debugDraw.DrawSegment(callback.m_point, head, color3);
			}
			else
			{
                b2Color color4 = { 0.8f, 0.8f, 0.8f, 1.0f };
				g_debugDraw.DrawSegment(point1, point2, color4);
			}
		}
		else if (m_mode == e_multiple)
		{
            b2Color color0 = { 0.8f, 0.8f, 0.8f, 1.0f };
            RayCastMultipleCallback callback;
			b2WorldRayCast(m_world, &callback, point1, point2);
			g_debugDraw.DrawSegment(point1, point2, color0);

			for (int32 i = 0; i < callback.m_count; ++i)
			{
                b2Color color1 = { 0.4f, 0.9f, 0.4f, 1.0f };
                b2Color color2 = { 0.8f, 0.8f, 0.8f, 1.0f };
                b2Color color3 = { 0.9f, 0.9f, 0.4f, 1.0f };
                b2Vec2 p;
                b2Vec2 n;
                b2Vec2Assign(p, callback.m_points[i]);
                b2Vec2Assign(n, callback.m_normals[i]);
				g_debugDraw.DrawPoint(p, 5.0f, color1);
				g_debugDraw.DrawSegment(point1, p, color2);
                b2Vec2 head;
                b2Vec2Scale(head, n, 0.5f);
                b2Vec2Add(head, p, head);
				g_debugDraw.DrawSegment(p, head, color3);
			}
		}

#if 0
		// This case was failing.
		{
			b2Vec2 vertices[4];
			//b2Vec2Make(vertices[0], -22.875f, -3.0f);
			//b2Vec2Make(vertices[1], 22.875f, -3.0f);
			//b2Vec2Make(vertices[2], 22.875f, 3.0f);
			//b2Vec2Make(vertices[3], -22.875f, 3.0f);

			struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			//b2ShapePolygonSetPoints(&shape, vertices, 4);
			b2ShapePolygonSetAsBox(&shape, 22.875f, 3.0f);

            struct b2RayCastInput input;
            b2Vec2Make(input.p1, 10.2725f,1.71372f);
            b2Vec2Make(input.p2, 10.2353f,2.21807f);
			//input.maxFraction = 0.567623f;
			input.maxFraction = 0.56762173f;

			b2Transform xf;
            b2TransformMakeIdentity(xf);
            b2Vec2Make(xf[0], 23.0f, 5.0f);

            struct b2RayCastOutput output;
			bool hit;
			hit = b2ShapePolygonRayCast(&shape, &output, &input, xf, 0);
			hit = false;

            b2Color color = { 1.0f, 1.0f, 1.0f, 1.0f };
			b2Vec2 vs[4];
			for (int32 i = 0; i < 4; ++i)
			{
                b2TransformMulVec2(vs[i], xf, shape.m_vertices[i]);
			}

			g_debugDraw.DrawPolygon(vs, 4, color);
			g_debugDraw.DrawSegment(input.p1, input.p2, color);
		}
#endif
	}

	static Test* Create()
	{
		return new RayCast;
	}

	int32 m_bodyIndex;
	b2Body* m_bodies[e_maxBodies];
	struct b2ShapePolygon m_polygons[4];
	struct b2ShapeCircle m_circle;
	struct b2ShapeEdge m_edge;
	float m_degrees;
	int32 m_mode;
};

static int testIndex = RegisterTest("Collision", "Ray Cast", RayCast::Create);
