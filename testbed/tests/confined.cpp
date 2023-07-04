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

class Confined : public Test
{
public:

	enum
	{
		e_columnCount = 0,
		e_rowCount = 0
	};

	Confined()
	{
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);

            b2Vec2 a;
            b2Vec2 b;

			// Floor
            b2Vec2Make(a, -10.0f, 0.0f);
            b2Vec2Make(b, 10.0f, 0.0f);
            b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			// Left wall
            b2Vec2Make(a, -10.0f, 0.0f);
            b2Vec2Make(b, -10.0f, 20.0f);
			b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			// Right wall
            b2Vec2Make(a, 10.0f, 0.0f);
            b2Vec2Make(b, 10.0f, 20.0f);
			b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			// Roof
            b2Vec2Make(a, -10.0f, 20.0f);
            b2Vec2Make(b, 10.0f, 20.0f);
			b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}

		float radius = 0.5f;
        struct b2ShapeCircle shape;
        b2ShapeCircleReset(&shape);
		b2Vec2SetZero(shape.m_p);
		shape.m_radius = radius;

        struct b2FixtureDef fd;
        b2FixtureDefReset(&fd);
		fd.shape = (struct b2Shape*)&shape;
		fd.density = 1.0f;
		fd.friction = 0.1f;

		for (int32 j = 0; j < e_columnCount; ++j)
		{
			for (int i = 0; i < e_rowCount; ++i)
			{
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, -10.0f + (2.1f * j + 1.0f + 0.01f * i) * radius, (2.0f * i + 1.0f) * radius);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);

				b2BodyCreateFixtureFromDef(body, &fd);
			}
		}

        b2Vec2 Gravity = { 0.0f, 0.0f };
		b2WorldSetGravity(m_world, Gravity);
	}

	void CreateCircle()
	{
		float radius = 2.0f;
        struct b2ShapeCircle shape;
        b2ShapeCircleReset(&shape);
		b2Vec2SetZero(shape.m_p);
		shape.m_radius = radius;

        struct b2FixtureDef fd;
        b2FixtureDefReset(&fd);
		fd.shape = (struct b2Shape*)&shape;
		fd.density = 1.0f;
		fd.friction = 0.0f;

        b2Vec2 p = { RandomFloat(), 3.0f + RandomFloat() };
        struct b2BodyDef bd;
        b2BodyDefReset(&bd);
		bd.type = b2BodyTypeDynamic;
		b2Vec2Assign(bd.position, p);
		//bd.allowSleep = false;
        struct b2Body* body = b2WorldCreateBody(m_world, &bd);

		b2BodyCreateFixtureFromDef(body, &fd);
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_C:
			CreateCircle();
			break;
		}
	}

	void Step(Settings& settings) override
	{
		bool sleeping = true;
        struct b2Body* b;
		for (b = b2WorldGetBodyListRef(m_world); b; b = b2BodyGetNextRef(b))
		{
			if (b2BodyGetType(b) != b2BodyTypeDynamic)
			{
				continue;
			}

			if (b2BodyIsAwake(b))
			{
				sleeping = false;
			}
		}

		if (m_stepCount == 180)
		{
			m_stepCount += 0;
		}

		//if (sleeping)
		//{
		//	CreateCircle();
		//}

		Test::Step(settings);

		for (b = b2WorldGetBodyListRef(m_world); b; b = b2BodyGetNextRef(b))
		{
			if (b2BodyGetType(b) != b2BodyTypeDynamic)
			{
				continue;
			}

            b2Vec2 p;
            b2Vec2ConstRef pref = b2BodyGetPosition(b);
            b2Vec2Assign(p, pref);
			if (p[0] <= -10.0f || 10.0f <= p[0] || p[1] <= 0.0f || 20.0f <= p[1])
			{
				p[0] += 0.0f;
			}
		}

		g_debugDraw.DrawString(5, m_textLine, "Press 'c' to create a circle.");
		m_textLine += m_textIncrement;
	}

	static Test* Create()
	{
		return new Confined;
	}
};

static int testIndex = RegisterTest("Solver", "Confined", Confined::Create);
