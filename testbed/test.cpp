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
#include "settings.h"
#include <stdio.h>
#include <string.h>

void
DestructionListenerSayGoodbyeJoint(
    DestructionListener* p,
    struct b2Joint* joint)
{
    if ((struct b2Joint*)p->test->m_mouseJoint == joint)
    {
        p->test->m_mouseJoint = NULL;
    }
    else
    {
        p->test->JointDestroyed(joint);
    }
}

void
DestructionListenerSayGoodbyeFixture(
    DestructionListener* p,
    struct b2Fixture* fixture)
{
    B2_NOT_USED(p);
    B2_NOT_USED(fixture);
}

const struct b2DestructionListenerMeta b2DestructionListenerMetaMy = 
{
    &DestructionListenerSayGoodbyeJoint,
    &DestructionListenerSayGoodbyeFixture,
};

DestructionListener::DestructionListener(void)
{
    this->m = &b2DestructionListenerMetaMy;
    this->o = this;
}


void
TestBeginContact(
    Test* p,
    struct b2Contact* contact)
{
    //B2_NOT_USED(p);
    //B2_NOT_USED(contact);
    p->BeginContact(contact);
}

void
TestEndContact(
    Test* p,
    struct b2Contact* contact)
{
    //B2_NOT_USED(p);
    //B2_NOT_USED(contact);
    p->EndContact(contact);
}

void
TestPreSolve(
    Test* p,
    struct b2Contact* contact,
    const struct b2Manifold* oldManifold)
{
    p->PreSolve(contact, oldManifold);
}

void
TestPostSolve(
    Test* p,
    struct b2Contact* contact,
    const struct b2ContactImpulse* impulse)
{
    //B2_NOT_USED(p);
    //B2_NOT_USED(contact);
    //B2_NOT_USED(impulse);
    p->PostSolve(contact, impulse);
}

const struct b2ContactListenerMeta b2ContactListenerMetaTest = 
{
    &TestBeginContact,
    &TestEndContact,
    &TestPreSolve,
    &TestPostSolve,
};

Test::Test()
{
    this->m = &b2ContactListenerMetaTest;
    this->o = this;

	b2Vec2 gravity;
	b2Vec2Make(gravity, 0.0f, -10.0f);
	m_world = new b2World();
    b2WorldInit(m_world);
    b2WorldSetGravity(this->m_world, gravity);
	m_bomb = NULL;
	m_textLine = 30;
	m_textIncrement = 18;
	m_mouseJoint = NULL;
	m_pointCount = 0;

	m_destructionListener.test = this;
	b2WorldSetDestructionListener(this->m_world, &m_destructionListener);
	b2WorldSetContactListener(this->m_world, this);
	b2WorldSetDebugDraw(this->m_world, &g_debugDraw);
	
	m_bombSpawning = false;

	m_stepCount = 0;

    struct b2BodyDef bodyDef;
    b2BodyDefReset(&bodyDef);
	m_groundBody = b2WorldCreateBody(this->m_world, &bodyDef);

	memset(&m_maxProfile, 0, sizeof(struct b2Profile));
	memset(&m_totalProfile, 0, sizeof(struct b2Profile));
}

Test::~Test()
{
    b2WorldDeleteBody(this->m_world, m_groundBody);

	// By deleting the world, we delete the bomb, mouse joint, etc.
    b2WorldDestroy(m_world);
    delete m_world;
	m_world = NULL;
}

void Test::DrawTitle(const char *string)
{
    g_debugDraw.DrawString(5, 5, string);
    m_textLine = int32(26.0f);
}

extern const struct b2QueryCallbackMeta b2QueryCallbackMetaMy;

class QueryCallback : public b2QueryCallback
{
public:
    QueryCallback(const b2Vec2 point)
    {
        this->m = &b2QueryCallbackMetaMy;
        this->o = this;

        b2Vec2Assign(m_point, point);
        m_fixture = NULL;
    }

    b2Vec2 m_point;
    b2Fixture* m_fixture;
};

int 
QueryCallbackReportFixture(
    QueryCallback* p,
    struct b2Fixture* fixture)
{
    struct b2Body* body = b2FixtureGetBodyRef(fixture);
    if (b2BodyGetType(body) == b2BodyTypeDynamic)
    {
        bool inside = b2FixtureTestPoint(fixture, p->m_point);
        if (inside)
        {
            p->m_fixture = fixture;

            // We are done, terminate the query.
            return false;
        }
    }

    // Continue the query.
    return true;
}

const struct b2QueryCallbackMeta b2QueryCallbackMetaMy =
{
    &QueryCallbackReportFixture,
};

void Test::MouseDown(const b2Vec2 p)
{
	b2Vec2Assign(m_mouseWorld, p);
	
	if (m_mouseJoint != NULL)
	{
		return;
	}

	// Make a small box.
	struct b2AABB aabb;
	b2Vec2 d;
    b2Vec2Make(d, 0.001f, 0.001f);
    b2Vec2Sub(aabb.lowerBound, p, d);
    b2Vec2Add(aabb.upperBound, p, d);

	// Query the world for overlapping shapes.
	QueryCallback callback(p);
	b2WorldQueryAABB(m_world, &callback, &aabb);

	if (callback.m_fixture)
	{
		float frequencyHz = 5.0f;
		float dampingRatio = 0.7f;

        struct b2Body* body = b2FixtureGetBodyRef(callback.m_fixture);
		struct b2JointMouseDef jd;
        b2JointMouseDefReset(&jd);
		jd.bodyA = m_groundBody;
		jd.bodyB = body;
		b2Vec2Assign(jd.target, p);
		jd.maxForce = 1000.0f * b2BodyGetMass(body);
		b2LinearStiffness(&jd.stiffness, &jd.damping, frequencyHz, dampingRatio, jd.bodyA, jd.bodyB);

		m_mouseJoint = (struct b2JointMouse*)b2WorldCreateJoint(m_world, &jd);
		b2BodySetAwake(body, true);
	}
}

void Test::SpawnBomb(const b2Vec2 worldPt)
{
	b2Vec2Assign(m_bombSpawnPoint, worldPt);
	m_bombSpawning = true;
}
    
void Test::CompleteBombSpawn(const b2Vec2 p)
{
	if (m_bombSpawning == false)
	{
		return;
	}

	const float multiplier = 30.0f;
    b2Vec2 vel;
    b2Vec2Sub(vel, m_bombSpawnPoint, p);
    b2Vec2Scale(vel, vel, multiplier);
	LaunchBomb(m_bombSpawnPoint,vel);
	m_bombSpawning = false;
}

void Test::PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
{
    const struct b2Manifold* manifold = b2ContactGetManifold(contact);

    if (manifold->pointCount == 0)
    {
        return;
    }

    struct b2Fixture* fixtureA = b2ContactGetFixtureARef(contact);
    struct b2Fixture* fixtureB = b2ContactGetFixtureBRef(contact);

    enum b2PointState state1[b2_maxManifoldPoints], state2[b2_maxManifoldPoints];
    b2GetPointStates(state1, state2, oldManifold, manifold);

    struct b2WorldManifold worldManifold;
    b2ContactGetWorldManifold(contact, &worldManifold);

    for (int32 i = 0; i < manifold->pointCount && this->m_pointCount < k_maxContactPoints; ++i)
    {
        ContactPoint* cp = this->m_points + this->m_pointCount;
        cp->fixtureA = fixtureA;
        cp->fixtureB = fixtureB;
        b2Vec2Assign(cp->position, worldManifold.points[i]);
        b2Vec2Assign(cp->normal, worldManifold.normal);
        cp->state = state2[i];
        cp->normalImpulse = manifold->points[i].normalImpulse;
        cp->tangentImpulse = manifold->points[i].tangentImpulse;
        cp->separation = worldManifold.separations[i];
        ++this->m_pointCount;
    }
}

void Test::ShiftMouseDown(const b2Vec2 p)
{
	b2Vec2Assign(m_mouseWorld, p);
	
	if (m_mouseJoint != NULL)
	{
		return;
	}

	SpawnBomb(p);
}

void Test::MouseUp(const b2Vec2 p)
{
	if (m_mouseJoint)
	{
		b2WorldDeleteJoint(m_world, (struct b2Joint*)m_mouseJoint);
		m_mouseJoint = NULL;
	}
	
	if (m_bombSpawning)
	{
		CompleteBombSpawn(p);
	}
}

void Test::MouseMove(const b2Vec2 p)
{
	b2Vec2Assign(m_mouseWorld, p);
	
	if (m_mouseJoint)
	{
		b2JointMouseSetTarget(m_mouseJoint, p);
	}
}

void Test::LaunchBomb()
{
    b2Vec2 p = { RandomFloat(-15.0f, 15.0f), 30.0f };
    b2Vec2 v;
    b2Vec2Scale(v, p, -5.0f);
	LaunchBomb(p, v);
}

void Test::LaunchBomb(const b2Vec2 position, const b2Vec2 velocity)
{
	if (m_bomb)
	{
		b2WorldDeleteBody(m_world, m_bomb);
		m_bomb = NULL;
	}

	struct b2BodyDef bd;
    b2BodyDefReset(&bd);
	bd.type = b2BodyTypeDynamic;
	b2Vec2Assign(bd.position, position);
	bd.bullet = true;
	m_bomb = b2WorldCreateBody(m_world, &bd);
	b2BodySetLinearVelocity(m_bomb, velocity);
	
    struct b2ShapeCircle circle;
    b2ShapeCircleReset(&circle);
    circle.m_radius = 0.3f;

    struct b2FixtureDef fd;
    b2FixtureDefReset(&fd);
	fd.shape = (struct b2Shape*)&circle;
	fd.density = 20.0f;
	fd.restitution = 0.0f;
	
    b2Vec2 pos1 = { 0.3f, 0.3f };
    b2Vec2 pos2 = { 0.3f, 0.3f };

    b2Vec2 minV;
    b2Vec2 maxV;

    b2Vec2Sub(minV, position, pos1);
    b2Vec2Add(maxV, position, pos2);
	
	struct b2AABB aabb;
    b2Vec2Assign(aabb.lowerBound, minV);
    b2Vec2Assign(aabb.upperBound, maxV);

    b2BodyCreateFixtureFromDef(m_bomb, &fd);
}

void Test::Step(Settings& settings)
{
	float timeStep = settings.m_hertz > 0.0f ? 1.0f / settings.m_hertz : float(0.0f);

	if (settings.m_pause)
	{
		if (settings.m_singleStep)
		{
			settings.m_singleStep = 0;
		}
		else
		{
			timeStep = 0.0f;
		}

		g_debugDraw.DrawString(5, m_textLine, "****PAUSED****");
		m_textLine += m_textIncrement;
	}

	uint32 flags = 0;
	flags += settings.m_drawShapes * b2DrawBitShape;
	flags += settings.m_drawJoints * b2DrawBitJoint;
	flags += settings.m_drawAABBs * b2DrawBitAabb;
	flags += settings.m_drawCOMs * b2DrawBitCenterOfMass;
	b2DrawSetFlags(&g_debugDraw, flags);

	b2WorldSetAllowSleeping(m_world, settings.m_enableSleep);
	b2WorldSetWarmStarting(m_world, settings.m_enableWarmStarting);
	b2WorldSetContinuousPhysics(m_world, settings.m_enableContinuous);
	b2WorldSetSubStepping(m_world, settings.m_enableSubStepping);

	m_pointCount = 0;

	b2WorldStep(m_world, timeStep, settings.m_velocityIterations, settings.m_positionIterations);

	b2WorldDebugDraw(m_world);
    g_debugDraw.Flush();

	if (timeStep > 0.0f)
	{
		++m_stepCount;
	}

	if (settings.m_drawStats)
	{
		int32 bodyCount = b2WorldGetBodyCount(m_world);
		int32 contactCount = b2WorldGetContactCount(m_world);
		int32 jointCount = b2WorldGetJointCount(m_world);
		g_debugDraw.DrawString(5, m_textLine, "bodies/contacts/joints = %d/%d/%d", bodyCount, contactCount, jointCount);
		m_textLine += m_textIncrement;

		int32 proxyCount = b2WorldGetProxyCount(m_world);
		int32 height = b2WorldGetTreeHeight(m_world);
		int32 balance = b2WorldGetTreeBalance(m_world);
		float quality = b2WorldGetTreeQuality(m_world);
		g_debugDraw.DrawString(5, m_textLine, "proxies/height/balance/quality = %d/%d/%d/%g", proxyCount, height, balance, quality);
		m_textLine += m_textIncrement;
	}

	// Track maximum profile times
	{
		const struct b2Profile* p = b2WorldGetProfile(m_world);
		m_maxProfile.step = b2MaxFloat(m_maxProfile.step, p->step);
		m_maxProfile.collide = b2MaxFloat(m_maxProfile.collide, p->collide);
		m_maxProfile.solve = b2MaxFloat(m_maxProfile.solve, p->solve);
		m_maxProfile.solveInit = b2MaxFloat(m_maxProfile.solveInit, p->solveInit);
		m_maxProfile.solveVelocity = b2MaxFloat(m_maxProfile.solveVelocity, p->solveVelocity);
		m_maxProfile.solvePosition = b2MaxFloat(m_maxProfile.solvePosition, p->solvePosition);
		m_maxProfile.solveTOI = b2MaxFloat(m_maxProfile.solveTOI, p->solveTOI);
		m_maxProfile.broadphase = b2MaxFloat(m_maxProfile.broadphase, p->broadphase);

		m_totalProfile.step += p->step;
		m_totalProfile.collide += p->collide;
		m_totalProfile.solve += p->solve;
		m_totalProfile.solveInit += p->solveInit;
		m_totalProfile.solveVelocity += p->solveVelocity;
		m_totalProfile.solvePosition += p->solvePosition;
		m_totalProfile.solveTOI += p->solveTOI;
		m_totalProfile.broadphase += p->broadphase;
	}

	if (settings.m_drawProfile)
	{
		const struct b2Profile* p = b2WorldGetProfile(m_world);

        struct b2Profile aveProfile;
		memset(&aveProfile, 0, sizeof(struct b2Profile));
		if (m_stepCount > 0)
		{
			float scale = 1.0f / m_stepCount;
			aveProfile.step = scale * m_totalProfile.step;
			aveProfile.collide = scale * m_totalProfile.collide;
			aveProfile.solve = scale * m_totalProfile.solve;
			aveProfile.solveInit = scale * m_totalProfile.solveInit;
			aveProfile.solveVelocity = scale * m_totalProfile.solveVelocity;
			aveProfile.solvePosition = scale * m_totalProfile.solvePosition;
			aveProfile.solveTOI = scale * m_totalProfile.solveTOI;
			aveProfile.broadphase = scale * m_totalProfile.broadphase;
		}

		g_debugDraw.DrawString(5, m_textLine, "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p->step, aveProfile.step, m_maxProfile.step);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p->collide, aveProfile.collide, m_maxProfile.collide);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p->solve, aveProfile.solve, m_maxProfile.solve);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)", p->solveInit, aveProfile.solveInit, m_maxProfile.solveInit);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)", p->solveVelocity, aveProfile.solveVelocity, m_maxProfile.solveVelocity);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)", p->solvePosition, aveProfile.solvePosition, m_maxProfile.solvePosition);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)", p->solveTOI, aveProfile.solveTOI, m_maxProfile.solveTOI);
		m_textLine += m_textIncrement;
		g_debugDraw.DrawString(5, m_textLine, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p->broadphase, aveProfile.broadphase, m_maxProfile.broadphase);
		m_textLine += m_textIncrement;
	}

	if (m_bombSpawning)
	{
		b2Color c;
        b2ColorMakeRGB(c, 0.0f, 0.0f, 1.0f);
        g_debugDraw.DrawPoint(m_bombSpawnPoint, 4.0f, c);

        b2ColorMakeRGB(c, 0.8f, 0.8f, 0.8f);
        g_debugDraw.DrawSegment(m_mouseWorld, m_bombSpawnPoint, c);
	}

	if (settings.m_drawContactPoints)
	{
		const float k_impulseScale = 0.1f;
		const float k_axisScale = 0.3f;

		for (int32 i = 0; i < m_pointCount; ++i)
		{
			ContactPoint* point = m_points + i;

			if (point->state == b2StateAdd)
			{
				// Add
                static const b2Color c = { 0.3f, 0.95f, 0.3f, 1.0f };
                g_debugDraw.DrawPoint(point->position, 10.0f, c);
			}
			else if (point->state == b2StatePersist)
			{
				// Persist
                static const b2Color c = { 0.3f, 0.3f, 0.95f, 1.0f };
                g_debugDraw.DrawPoint(point->position, 5.0f, c);
			}

			if (settings.m_drawContactNormals == 1)
			{
                static const b2Color c = { 0.9f, 0.9f, 0.9f, 1.0f };
                b2Vec2 p1;
                b2Vec2 p2;
                b2Vec2Assign(p1, point->position);
                b2Vec2Scale(p2, point->normal, k_axisScale);
                b2Vec2Add(p2, p1, p2);
                g_debugDraw.DrawSegment(p1, p2, c);
			}
			else if (settings.m_drawContactImpulse == 1)
			{
                static const b2Color c = { 0.9f, 0.9f, 0.3f, 1.0f };
                b2Vec2 p1;
                b2Vec2 p2;
                b2Vec2Assign(p1, point->position);
                b2Vec2Scale(p2, point->normal, k_impulseScale * point->normalImpulse);
                b2Vec2Add(p2, p1, p2);
                g_debugDraw.DrawSegment(p1, p2, c);
			}

			if (settings.m_drawFrictionImpulse == 1)
			{
                static const b2Color c = { 0.9f, 0.9f, 0.3f, 1.0f };
                b2Vec2 tangent;
                b2Vec2 p1;
                b2Vec2 p2;
                b2Vec2CrossProductKR(tangent, point->normal, 1.0f);
                b2Vec2Assign(p1, point->position);
                b2Vec2Scale(p2, tangent, k_impulseScale * point->tangentImpulse);
                b2Vec2Add(p2, p1, p2);
                g_debugDraw.DrawSegment(p1, p2, c);
			}
		}
	}
}

void Test::ShiftOrigin(const b2Vec2 newOrigin)
{
	b2WorldShiftOrigin(m_world, newOrigin);
}

TestEntry g_testEntries[MAX_TESTS] = { {nullptr} };
int g_testCount = 0;

int RegisterTest(const char* category, const char* name, TestCreateFcn* fcn)
{
	int index = g_testCount;
	if (index < MAX_TESTS)
	{
		g_testEntries[index] = { category, name, fcn };
		++g_testCount;
		return index;
	}

	return -1;
}
