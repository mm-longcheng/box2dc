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

#include <assert.h>

class DynamicTree : public Test
{
public:

	enum
	{
		e_actorCount = 128
	};

	DynamicTree()
	{
        b2DynamicTreeInit(&this->m_tree);

		m_worldExtent = 15.0f;
		m_proxyExtent = 0.5f;

		srand(888);

		for (int32 i = 0; i < e_actorCount; ++i)
		{
			Actor* actor = m_actors + i;
			GetRandomAABB(&actor->aabb);
			actor->proxyId = b2DynamicTreeCreateProxy(&m_tree, &actor->aabb, actor);
		}

		m_stepCount = 0;

		float h = m_worldExtent;
		b2Vec2Make(m_queryAABB.lowerBound, -3.0f, -4.0f + h);
		b2Vec2Make(m_queryAABB.upperBound, 5.0f, 6.0f + h);

        b2Vec2Make(m_rayCastInput.p1, -5.0, 5.0f + h);
        b2Vec2Make(m_rayCastInput.p2, 7.0f, -4.0f + h);
		//b2Vec2Make(m_rayCastInput.p1, 0.0f, 2.0f + h);
		//b2Vec2Make(m_rayCastInput.p2, 0.0f, -2.0f + h);
		m_rayCastInput.maxFraction = 1.0f;

		m_automated = false;
	}

    ~DynamicTree()
    {
        b2DynamicTreeDestroy(&this->m_tree);
    }

	static Test* Create()
	{
		return new DynamicTree;
	}

	void Step(Settings& settings) override
	{
		B2_NOT_USED(settings);

		m_rayActor = NULL;
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			m_actors[i].fraction = 1.0f;
			m_actors[i].overlap = false;
		}

		if (m_automated == true)
		{
			int32 actionCount = b2MaxInt32(1, e_actorCount >> 2);

			for (int32 i = 0; i < actionCount; ++i)
			{
				Action();
			}
		}

		Query();
		RayCast();

		for (int32 i = 0; i < e_actorCount; ++i)
		{
			Actor* actor = m_actors + i;
			if (actor->proxyId == b2_nullNode)
				continue;

            b2Color c = { 0.9f, 0.9f, 0.9f, 1.0f };
			if (actor == m_rayActor && actor->overlap)
			{
                b2ColorMakeRGB(c, 0.9f, 0.6f, 0.6f);
			}
			else if (actor == m_rayActor)
			{
                b2ColorMakeRGB(c, 0.6f, 0.9f, 0.6f);
			}
			else if (actor->overlap)
			{
                b2ColorMakeRGB(c, 0.6f, 0.6f, 0.9f);
			}

			g_debugDraw.DrawAABB(&actor->aabb, c);
		}

        b2Color c = { 0.7f, 0.7f, 0.7f, 1.0f };
		g_debugDraw.DrawAABB(&m_queryAABB, c);

		g_debugDraw.DrawSegment(m_rayCastInput.p1, m_rayCastInput.p2, c);

        b2Color c1 = { 0.2f, 0.9f, 0.2f, 1.0f };
        b2Color c2 = { 0.9f, 0.2f, 0.2f, 1.0f };
		g_debugDraw.DrawPoint(m_rayCastInput.p1, 6.0f, c1);
		g_debugDraw.DrawPoint(m_rayCastInput.p2, 6.0f, c2);

		if (m_rayActor)
		{
            b2Color cr = { 0.2f, 0.2f, 0.9f, 1.0f };
            b2Vec2 v;
            b2Vec2 p;
            b2Vec2Sub(v, m_rayCastInput.p2, m_rayCastInput.p1);
            b2Vec2Scale(v, v, m_rayActor->fraction);
            b2Vec2Add(p, m_rayCastInput.p1, v);
			g_debugDraw.DrawPoint(p, 6.0f, cr);
		}

		{
			int32 height = b2DynamicTreeGetHeight(&m_tree);
			g_debugDraw.DrawString(5, m_textLine, "dynamic tree height = %d", height);
			m_textLine += m_textIncrement;
		}

		++m_stepCount;
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_automated = !m_automated;
			break;

		case GLFW_KEY_C:
			CreateProxy();
			break;

		case GLFW_KEY_D:
			DestroyProxy();
			break;

		case GLFW_KEY_M:
			MoveProxy();
			break;
		}
	}

	static int QueryCallback(DynamicTree* p, int32 proxyId)
	{
		Actor* actor = (Actor*)b2DynamicTreeGetUserData(&p->m_tree, proxyId);
		actor->overlap = b2AABBTestOverlap(&p->m_queryAABB, &actor->aabb);
		return true;
	}

    static float RayCastCallback(DynamicTree* p, const b2RayCastInput& input, int32 proxyId)
	{
		Actor* actor = (Actor*)b2DynamicTreeGetUserData(&p->m_tree, proxyId);

		b2RayCastOutput output;
		bool hit = b2AABBRayCast(&actor->aabb, &output, &input);

		if (hit)
		{
            p->m_rayCastOutput = output;
            p->m_rayActor = actor;
            p->m_rayActor->fraction = output.fraction;
			return output.fraction;
		}

		return input.maxFraction;
	}

private:

	struct Actor
	{
		b2AABB aabb;
		float fraction;
		bool overlap;
		int32 proxyId;
	};

	void GetRandomAABB(b2AABB* aabb)
	{
		b2Vec2 w; 
        b2Vec2Make(w, 2.0f * m_proxyExtent, 2.0f * m_proxyExtent);
		//aabb->lowerBound[0] = -m_proxyExtent;
		//aabb->lowerBound[1] = -m_proxyExtent + m_worldExtent;
		aabb->lowerBound[0] = RandomFloat(-m_worldExtent, m_worldExtent);
		aabb->lowerBound[1] = RandomFloat(0.0f, 2.0f * m_worldExtent);
        b2Vec2Add(aabb->upperBound, aabb->lowerBound, w);
	}

	void MoveAABB(b2AABB* aabb)
	{
		b2Vec2 d;
		d[0] = RandomFloat(-0.5f, 0.5f);
		d[1] = RandomFloat(-0.5f, 0.5f);
		//d.x = 2.0f;
		//d.y = 0.0f;
		b2Vec2Add(aabb->lowerBound, aabb->lowerBound, d);
        b2Vec2Add(aabb->upperBound, aabb->upperBound, d);

        b2Vec2 c0;
        b2Vec2 min;
        b2Vec2 max;
        b2Vec2 c;

        b2Vec2Add(c0, aabb->lowerBound, aabb->upperBound);
        b2Vec2Scale(c0, c0, 0.5f);
		b2Vec2Make(min, -m_worldExtent, 0.0f);
        b2Vec2Make(max, m_worldExtent, 2.0f * m_worldExtent);
        b2Vec2Clamp(c, c0, min, max);

        b2Vec2 v;
        b2Vec2Sub(v, c, c0);
        b2Vec2Add(aabb->lowerBound, aabb->lowerBound, v);
        b2Vec2Add(aabb->upperBound, aabb->upperBound, v);
	}

	void CreateProxy()
	{
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			int32 j = rand() % e_actorCount;
			Actor* actor = m_actors + j;
			if (actor->proxyId == b2_nullNode)
			{
				GetRandomAABB(&actor->aabb);
				actor->proxyId = b2DynamicTreeCreateProxy(&m_tree, &actor->aabb, actor);
				return;
			}
		}
	}

	void DestroyProxy()
	{
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			int32 j = rand() % e_actorCount;
			Actor* actor = m_actors + j;
			if (actor->proxyId != b2_nullNode)
			{
				b2DynamicTreeDeleteProxy(&m_tree, actor->proxyId);
				actor->proxyId = b2_nullNode;
				return;
			}
		}
	}

	void MoveProxy()
	{
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			int32 j = rand() % e_actorCount;
			Actor* actor = m_actors + j;
			if (actor->proxyId == b2_nullNode)
			{
				continue;
			}

			struct b2AABB aabb0 = actor->aabb;
			MoveAABB(&actor->aabb);
            b2Vec2 v1, v2;
            b2Vec2 displacement;
            b2AABBGetCenter(&actor->aabb, v1);
            b2AABBGetCenter(&aabb0, v2);
            b2Vec2Add(displacement, v1, v2);
			b2DynamicTreeMoveProxy(&m_tree, actor->proxyId, &actor->aabb, displacement);
			return;
		}
	}

	void Action()
	{
		int32 choice = rand() % 20;

		switch (choice)
		{
		case 0:
			CreateProxy();
			break;

		case 1:
			DestroyProxy();
			break;

		default:
			MoveProxy();
		}
	}

	void Query()
	{
		b2DynamicTreeQuery(&m_tree, &m_queryAABB, this, &DynamicTree::QueryCallback);

		for (int32 i = 0; i < e_actorCount; ++i)
		{
			if (m_actors[i].proxyId == b2_nullNode)
			{
				continue;
			}

			bool overlap = b2AABBTestOverlap(&m_queryAABB, &m_actors[i].aabb);
			B2_NOT_USED(overlap);
			b2Assert(overlap == m_actors[i].overlap);
		}
	}

	void RayCast()
	{
		m_rayActor = NULL;

		b2RayCastInput input = m_rayCastInput;

		// Ray cast against the dynamic tree.
		b2DynamicTreeRayCast(&m_tree, &input, this, &DynamicTree::RayCastCallback);

		// Brute force ray cast.
		Actor* bruteActor = NULL;
		b2RayCastOutput bruteOutput;
		for (int32 i = 0; i < e_actorCount; ++i)
		{
			if (m_actors[i].proxyId == b2_nullNode)
			{
				continue;
			}

			b2RayCastOutput output;
			bool hit = b2AABBRayCast(&m_actors[i].aabb, &output, &input);
			if (hit)
			{
				bruteActor = m_actors + i;
				bruteOutput = output;
				input.maxFraction = output.fraction;
			}
		}

		if (bruteActor != NULL)
		{
			b2Assert(bruteOutput.fraction == m_rayCastOutput.fraction);
		}
	}

	float m_worldExtent;
	float m_proxyExtent;

	struct b2DynamicTree m_tree;
    struct b2AABB m_queryAABB;
    struct b2RayCastInput m_rayCastInput;
    struct b2RayCastOutput m_rayCastOutput;
	Actor* m_rayActor;
	Actor m_actors[e_actorCount];
	int32 m_stepCount;
	bool m_automated;
};

static int testIndex = RegisterTest("Collision", "Dynamic Tree", DynamicTree::Create);
