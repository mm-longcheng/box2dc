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

class Boxes : public Test
{
public:
	Boxes()
	{
		float groundSize = 25.0f;

		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon box;
            b2ShapePolygonReset(&box);
            b2ShapePolygonSetAsBox(&box, groundSize, 1.2f);
            struct b2FixtureDef sd;
            b2FixtureDefReset(&sd);
			sd.shape = (struct b2Shape*)&box;
			b2BodyCreateFixtureFromDef(ground, &sd);

			bd.angle = 0.5f * b2_pi;
			b2Vec2Make(bd.position, groundSize, 2.0f * groundSize);
			ground = b2WorldCreateBody(m_world, &bd);

			b2ShapePolygonSetAsBox(&box, 2.0f * groundSize, 1.2f);
            b2BodyCreateFixtureFromDef(ground, &sd);

            b2Vec2Make(bd.position, -groundSize, 2.0f * groundSize);
			ground = b2WorldCreateBody(m_world, &bd);
			b2BodyCreateFixtureFromDef(ground, &sd);
		}

		int32_t num = 26;
		float rad = 0.5f;

		float shift = rad * 2.0f;
		float centerx = shift * num / 2.0f;
		float centery = shift / 2.0f;

		struct b2BodyDef bd;
        b2BodyDefReset(&bd);
		bd.type = b2BodyTypeDynamic;

        struct b2FixtureDef sd;
        b2FixtureDefReset(&sd);
		sd.density = 1.0f;
		sd.friction = 0.5f;

        struct b2ShapePolygon cuboid;
        b2ShapePolygonReset(&cuboid);
        b2ShapePolygonSetAsBox(&cuboid, 0.5f, 0.5f);
		sd.shape = (struct b2Shape*)&cuboid;

#ifdef _DEBUG
		int32_t numj = 5;
#else
		int32_t numj = 5 * num;
#endif

		for (int32_t i = 0; i < num; ++i)
		{
			float x = i * shift - centerx;

			for (int32_t j = 0; j < numj; ++j)
			{
				float y = j * shift + centery + 2.0f;

				b2Vec2Make(bd.position, x, y);

				struct b2Body* rigidBody = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromDef(rigidBody, &sd);
			}
		}
    }

	static Test* Create()
	{
		return new Boxes;
	}
};

static int testIndex = RegisterTest("Benchmark", "Boxes", Boxes::Create);
