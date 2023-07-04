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
#include "imgui/imgui.h"

// This shows how to use sensor shapes. Sensors don't have collision, but report overlap events.
class Sensors : public Test
{
public:

	enum
	{
		e_count = 7
	};

	Sensors()
	{
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

			{
                b2Vec2 a = { -40.0f, 0.0f };
                b2Vec2 b = { 40.0f, 0.0f };
                struct b2ShapeEdge shape;
                b2ShapeEdgeReset(&shape);
				b2ShapeEdgeSetTwoSided(&shape, a, b);
				b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
			}

#if 0
			{
                b2Vec2 center = { 0.0f, 20.0f };
                struct b2FixtureDef sd;
                b2FixtureDefReset(&sd);
				b2FixtureDefSetAsBox(sd, 10.0f, 2.0f, center, 0.0f);
				sd.isSensor = true;
				m_sensor = b2BodyCreateFixtureFromDef(ground, &sd);
			}
#else
			{
                struct b2ShapeCircle shape;
                b2ShapeCircleReset(&shape);
				shape.m_radius = 5.0f;
				b2Vec2Make(shape.m_p, 0.0f, 10.0f);

                struct b2FixtureDef fd;
                b2FixtureDefReset(&fd);
				fd.shape = (struct b2Shape*)&shape;
				fd.isSensor = true;
				m_sensor = b2BodyCreateFixtureFromDef(ground, &fd);
			}
#endif
		}

		{
            struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
			shape.m_radius = 1.0f;

			for (int32 i = 0; i < e_count; ++i)
			{
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, -10.0f + 3.0f * i, 20.0f);
				bd.userData = i;

				m_touching[i] = false;
				m_bodies[i] = b2WorldCreateBody(m_world, &bd);

				b2BodyCreateFixtureFromShape(m_bodies[i], &shape, 1.0f);
			}
		}

		m_force = 100.0f;
	}

	// Implement contact listener.
	void BeginContact(struct b2Contact* contact) override
	{
        struct b2Fixture* fixtureA = b2ContactGetFixtureARef(contact);
        struct b2Fixture* fixtureB = b2ContactGetFixtureBRef(contact);

		if (fixtureA == m_sensor)
		{
			uintptr_t index = b2BodyGetUserData(b2FixtureGetBody(fixtureB));
			if (index < e_count)
			{
				m_touching[index] = true;
			}
		}

		if (fixtureB == m_sensor)
		{
			uintptr_t index = b2BodyGetUserData(b2FixtureGetBody(fixtureA));
			if (index < e_count)
			{
				m_touching[index] = true;
			}
		}
	}

	// Implement contact listener.
	void EndContact(struct b2Contact* contact) override
	{
        struct b2Fixture* fixtureA = b2ContactGetFixtureARef(contact);
        struct b2Fixture* fixtureB = b2ContactGetFixtureBRef(contact);

		if (fixtureA == m_sensor)
		{
			uintptr_t index = b2BodyGetUserData(b2FixtureGetBody(fixtureB));
			if (index < e_count)
			{
				m_touching[index] = false;
			}
		}

		if (fixtureB == m_sensor)
		{
			uintptr_t index = b2BodyGetUserData(b2FixtureGetBody(fixtureA));
			if (index < e_count)
			{
				m_touching[index] = false;
			}
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 60.0f));
		ImGui::Begin("Sensor Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		ImGui::SliderFloat("Force", &m_force, 0.0f, 2000.0f, "%.0f");

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		Test::Step(settings);

		// Traverse the contact results. Apply a force on shapes
		// that overlap the sensor.
		for (int32 i = 0; i < e_count; ++i)
		{
			if (m_touching[i] == false)
			{
				continue;
			}

			struct b2Body* body = m_bodies[i];
            struct b2Body* ground = b2FixtureGetBodyRef(m_sensor);

            struct b2ShapeCircle* circle = (struct b2ShapeCircle*)b2FixtureGetShape(m_sensor);
            b2Vec2 center;
            b2BodyGetWorldPoint(ground, circle->m_p, center);

			b2Vec2ConstRef position = b2BodyGetPosition(body);
            b2Vec2 d;
            b2Vec2Sub(d, center, position);
			if (b2Vec2SquaredLength(d) < FLT_EPSILON * FLT_EPSILON)
			{
				continue;
			}

            b2Vec2Normalize(d, d);
            b2Vec2 F;
            b2Vec2Scale(F, d, m_force);
			b2BodyApplyForce(body, F, position, false);
		}
	}

	static Test* Create()
	{
		return new Sensors;
	}

	struct b2Fixture* m_sensor;
    struct b2Body* m_bodies[e_count];
	float m_force;
	bool m_touching[e_count];
};

static int testIndex = RegisterTest("Collision", "Sensors", Sensors::Create);
