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

// This test holds worlds dumped using b2World::Dump.
class DumpLoader : public Test
{
public:

	DumpLoader()
	{
		struct b2ShapeChain chainShape;
        b2ShapeChainReset(&chainShape);
        b2Vec2 vertices[] = { {-5,0}, {5,0}, {5,5}, {4,1}, {-4,1}, {-5,5} };
		b2ShapeChainCreateLoop(&chainShape, vertices, 6);

        struct b2FixtureDef groundFixtureDef;
        b2FixtureDefReset(&groundFixtureDef);
		groundFixtureDef.density = 0;
		groundFixtureDef.shape = (struct b2Shape*)&chainShape;

        struct b2BodyDef groundBodyDef;
        b2BodyDefReset(&groundBodyDef);
		groundBodyDef.type = b2BodyTypeStatic;

        struct b2Body *groundBody = b2WorldCreateBody(m_world, &groundBodyDef);
        struct b2Fixture *groundBodyFixture = b2BodyCreateFixtureFromDef(groundBody, &groundFixtureDef);

        struct b2ShapeCircle ballShape;
        b2ShapeCircleReset(&ballShape);
		ballShape.m_radius = 1;

        struct b2FixtureDef ballFixtureDef;
        b2FixtureDefReset(&ballFixtureDef);
		ballFixtureDef.restitution = 0.75f;
		ballFixtureDef.density = 1;
		ballFixtureDef.shape = (struct b2Shape*)&ballShape;

        struct b2BodyDef ballBodyDef;
        b2BodyDefReset(&ballBodyDef);
		ballBodyDef.type = b2BodyTypeDynamic;
		b2Vec2Make(ballBodyDef.position, 0, 10);
		// ballBodyDef.angularDamping = 0.2f;

		m_ball = b2WorldCreateBody(m_world, &ballBodyDef);
        struct b2Fixture *ballFixture = b2BodyCreateFixtureFromDef(m_ball, &ballFixtureDef);
        b2Vec2 force = { -1000, -400 };
		b2BodyApplyForceToCenter(m_ball, force, true);

        b2ShapeChainClear(&chainShape);
	}

	void Step(Settings& settings) override
	{
        b2Vec2ConstRef v = b2BodyGetLinearVelocity(m_ball);
		float omega = b2BodyGetAngularVelocity(m_ball);

        struct b2MassData massData;
        b2BodyGetMassData(m_ball, &massData);

		float ke = 0.5f * massData.mass * b2Vec2DotProduct(v, v) + 0.5f * massData.I * omega * omega;

		g_debugDraw.DrawString(5, m_textLine, "kinetic energy = %.6f", ke);
		m_textLine += m_textIncrement;

		Test::Step(settings);
	}

	static Test* Create()
	{
		return new DumpLoader;
	}

	struct b2Body* m_ball;
};

static int testIndex = RegisterTest("Bugs", "Dump Loader", DumpLoader::Create);
