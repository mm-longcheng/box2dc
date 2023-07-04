/*
-----------------------------------------------------------------------------
MIT License

Copyright (c) 2019 Erin Catto
Copyright (c) 2023-2023 mm_longcheng@icloud.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
*/

#include "mmB2Fixture.h"
#include "mmB2Settings.h"
#include "mmB2Shape.h"
#include "mmB2BlockAllocator.h"
#include "mmB2BroadPhase.h"
#include "mmB2Contact.h"
#include "mmB2World.h"

#include "mmB2ShapeCircle.h"
#include "mmB2ShapeEdge.h"
#include "mmB2ShapePolygon.h"
#include "mmB2ShapeChain.h"

#include <assert.h>

B2_API
void
b2FilterReset(
    struct b2Filter* p)
{
    p->categoryBits = 0x0001;
    p->maskBits = 0xFFFF;
    p->groupIndex = 0;
}

B2_API
void
b2FixtureDefReset(
    struct b2FixtureDef* p)
{
    p->shape = NULL;
    p->userData = 0;
    p->friction = 0.2f;
    p->restitution = 0.0f;
    p->restitutionThreshold = 1.0f * b2_lengthUnitsPerMeter;
    p->density = 0.0f;
    p->isSensor = b2False;

    b2FilterReset(&p->filter);
}

B2_API
void
b2FixtureInit(
    struct b2Fixture* p)
{
    p->m_body = NULL;
    p->m_next = NULL;
    p->m_proxies = NULL;
    p->m_proxyCount = 0;
    p->m_shape = NULL;
    p->m_density = 0.0f;
    p->m_userData = 0;

    b2FilterReset(&p->m_filter);
}

B2_API
void
b2FixtureDestroy(
    struct b2Fixture* p)
{
    b2Assert(NULL == p->m_shape);
    b2Assert(NULL == p->m_proxies);

    p->m_userData = 0;
    p->m_density = 0.0f;
    p->m_shape = NULL;
    p->m_proxyCount = 0;
    p->m_proxies = NULL;
    p->m_next = NULL;
    p->m_body = NULL;
}

B2_API
enum b2ShapeType
b2FixtureGetType(
    const struct b2Fixture* p)
{
    return b2ShapeGetType(p->m_shape);
}

B2_API
void
b2FixtureSetSensor(
    struct b2Fixture* p,
    int sensor)
{
    if (sensor != p->m_isSensor)
    {
        b2BodySetAwake(p->m_body, b2True);
        p->m_isSensor = sensor;
    }
}

B2_API
void
b2FixtureSetFilterData(
    struct b2Fixture* p,
    const struct b2Filter* filter)
{
    p->m_filter = *filter;

    b2FixtureRefilter(p);
}

B2_API
void
b2FixtureRefilter(
    struct b2Fixture* p)
{
    int32 i;

    struct b2ContactEdge* edge;
    struct b2World* world;
    struct b2BroadPhase* broadPhase;

    if (p->m_body == NULL)
    {
        return;
    }

    // Flag associated contacts for filtering.
    edge = b2BodyGetContactListRef(p->m_body);
    while (edge)
    {
        struct b2Contact* contact;
        const struct b2Fixture* fixtureA;
        const struct b2Fixture* fixtureB;

        contact = edge->contact;
        fixtureA = b2ContactGetFixtureA(contact);
        fixtureB = b2ContactGetFixtureB(contact);
        if (fixtureA == p || fixtureB == p)
        {
            b2ContactFlagForFiltering(contact);
        }

        edge = edge->next;
    }

    world = b2BodyGetWorldRef(p->m_body);

    if (world == NULL)
    {
        return;
    }

    // Touch each proxy so that new pairs may be created
    broadPhase = &world->m_contactManager.m_broadPhase;
    for (i = 0; i < p->m_proxyCount; ++i)
    {
        b2BroadPhaseTouchProxy(broadPhase, p->m_proxies[i].proxyId);
    }
}

B2_API
int
b2FixtureTestPoint(
    const struct b2Fixture* p,
    const b2Vec2 point)
{
    return b2ShapeTestPoint(p->m_shape, b2BodyGetTransform(p->m_body), point);
}

B2_API
int
b2FixtureRayCast(
    const struct b2Fixture* p,
    struct b2RayCastOutput* output,
    const struct b2RayCastInput* input,
    int32 childIndex)
{
    return b2ShapeRayCast(p->m_shape, output, input, b2BodyGetTransform(p->m_body), childIndex);
}

B2_API
void
b2FixtureGetMassData(
    const struct b2Fixture* p,
    struct b2MassData* massData)
{
    b2ShapeComputeMass(p->m_shape, massData, p->m_density);
}

B2_API
void
b2FixtureSetDensity(
    struct b2Fixture* p,
    float density)
{
    b2Assert(b2IsValid(density) && density >= 0.0f);
    p->m_density = density;
}

B2_API
const struct b2AABB*
b2FixtureGetAABB(
    const struct b2Fixture* p,
    int32 childIndex)
{
    b2Assert(0 <= childIndex && childIndex < p->m_proxyCount);
    return &p->m_proxies[childIndex].aabb;
}

B2_API
void
b2FixtureDump(
    const struct b2Fixture* p,
    int32 bodyIndex)
{
    b2Dump("    b2FixtureDef fd;\n");
    b2Dump("    fd.friction = %.9g;\n", p->m_friction);
    b2Dump("    fd.restitution = %.9g;\n", p->m_restitution);
    b2Dump("    fd.restitutionThreshold = %.9g;\n", p->m_restitutionThreshold);
    b2Dump("    fd.density = %.9g;\n", p->m_density);
    b2Dump("    fd.isSensor = bool(%d);\n", p->m_isSensor);
    b2Dump("    fd.filter.categoryBits = uint16(%d);\n", p->m_filter.categoryBits);
    b2Dump("    fd.filter.maskBits = uint16(%d);\n", p->m_filter.maskBits);
    b2Dump("    fd.filter.groupIndex = int16(%d);\n", p->m_filter.groupIndex);

    switch (p->m_shape->m_type)
    {
    case b2ShapeTypeCircle:
    {
        struct b2ShapeCircle* s = (struct b2ShapeCircle*)p->m_shape;
        b2Dump("    b2CircleShape shape;\n");
        b2Dump("    shape.m_radius = %.9g;\n", s->m_radius);
        b2Dump("    shape.m_p.Set(%.9g, %.9g);\n", s->m_p[0], s->m_p[1]);
    }
    break;

    case b2ShapeTypeEdge:
    {
        struct b2ShapeEdge* s = (struct b2ShapeEdge*)p->m_shape;
        b2Dump("    b2EdgeShape shape;\n");
        b2Dump("    shape.m_radius = %.9g;\n", s->m_radius);
        b2Dump("    shape.m_vertex0.Set(%.9g, %.9g);\n", s->m_vertex0[0], s->m_vertex0[1]);
        b2Dump("    shape.m_vertex1.Set(%.9g, %.9g);\n", s->m_vertex1[0], s->m_vertex1[1]);
        b2Dump("    shape.m_vertex2.Set(%.9g, %.9g);\n", s->m_vertex2[0], s->m_vertex2[1]);
        b2Dump("    shape.m_vertex3.Set(%.9g, %.9g);\n", s->m_vertex3[0], s->m_vertex3[1]);
        b2Dump("    shape.m_oneSided = bool(%d);\n", s->m_oneSided);
    }
    break;

    case b2ShapeTypePolygon:
    {
        int32 i;
        struct b2ShapePolygon* s = (struct b2ShapePolygon*)p->m_shape;
        b2Dump("    b2PolygonShape shape;\n");
        b2Dump("    b2Vec2 vs[%d];\n", b2_maxPolygonVertices);
        for (i = 0; i < s->m_count; ++i)
        {
            b2Dump("    vs[%d].Set(%.9g, %.9g);\n", i, s->m_vertices[i][0], s->m_vertices[i][1]);
        }
        b2Dump("    shape.Set(vs, %d);\n", s->m_count);
    }
    break;

    case b2ShapeTypeChain:
    {
        int32 i;
        struct b2ShapeChain* s = (struct b2ShapeChain*)p->m_shape;
        b2Dump("    b2ChainShape shape;\n");
        b2Dump("    b2Vec2 vs[%d];\n", s->m_count);
        for (i = 0; i < s->m_count; ++i)
        {
            b2Dump("    vs[%d].Set(%.9g, %.9g);\n", i, s->m_vertices[i][0], s->m_vertices[i][1]);
        }
        b2Dump("    shape.CreateChain(vs, %d);\n", s->m_count);
        b2Dump("    shape.m_prevVertex.Set(%.9g, %.9g);\n", s->m_prevVertex[0], s->m_prevVertex[1]);
        b2Dump("    shape.m_nextVertex.Set(%.9g, %.9g);\n", s->m_nextVertex[0], s->m_nextVertex[1]);
    }
    break;

    default:
        return;
    }

    b2Dump("\n");
    b2Dump("    fd.shape = &shape;\n");
    b2Dump("\n");
    b2Dump("    bodies[%d]->CreateFixture(&fd);\n", bodyIndex);
}

B2_API
void
b2FixtureCreate(
    struct b2Fixture* p,
    struct b2BlockAllocator* allocator,
    struct b2Body* body,
    const struct b2FixtureDef* def)
{
    int32 childCount;
    int32 i;

    p->m_userData = def->userData;
    p->m_friction = def->friction;
    p->m_restitution = def->restitution;
    p->m_restitutionThreshold = def->restitutionThreshold;

    p->m_body = body;
    p->m_next = NULL;

    p->m_filter = def->filter;

    p->m_isSensor = def->isSensor;

    p->m_shape = b2ShapeClone(def->shape, allocator);

    // Reserve proxy space
    childCount = b2ShapeGetChildCount(p->m_shape);
    p->m_proxies = (struct b2FixtureProxy*)b2BlockAllocatorAllocate(allocator, childCount * sizeof(struct b2FixtureProxy));
    for (i = 0; i < childCount; ++i)
    {
        p->m_proxies[i].fixture = NULL;
        p->m_proxies[i].proxyId = b2BroadPhaseNullProxy;
    }
    p->m_proxyCount = 0;

    p->m_density = def->density;
}

B2_API
void
b2FixtureDelete(
    struct b2Fixture* p,
    struct b2BlockAllocator* allocator)
{
    int32 childCount;

    // The proxies must be destroyed before calling this.
    b2Assert(p->m_proxyCount == 0);

    // Free the proxy array.
    childCount = b2ShapeGetChildCount(p->m_shape);
    b2BlockAllocatorFree(allocator, p->m_proxies, childCount * sizeof(struct b2FixtureProxy));
    p->m_proxies = NULL;

    // Free the child shape.
    switch (p->m_shape->m_type)
    {
    case b2ShapeTypeCircle:
    {
        struct b2ShapeCircle* s = (struct b2ShapeCircle*)p->m_shape;
        b2ShapeCircleDestroy(s);
        b2BlockAllocatorFree(allocator, s, sizeof(struct b2ShapeCircle));
    }
    break;

    case b2ShapeTypeEdge:
    {
        struct b2ShapeEdge* s = (struct b2ShapeEdge*)p->m_shape;
        b2ShapeEdgeDestroy(s);
        b2BlockAllocatorFree(allocator, s, sizeof(struct b2ShapeEdge));
    }
    break;

    case b2ShapeTypePolygon:
    {
        struct b2ShapePolygon* s = (struct b2ShapePolygon*)p->m_shape;
        b2ShapePolygonDestroy(s);
        b2BlockAllocatorFree(allocator, s, sizeof(struct b2ShapePolygon));
    }
    break;

    case b2ShapeTypeChain:
    {
        struct b2ShapeChain* s = (struct b2ShapeChain*)p->m_shape;
        b2ShapeChainDestroy(s);
        b2BlockAllocatorFree(allocator, s, sizeof(struct b2ShapeChain));
    }
    break;

    default:
        b2Assert(b2False);
        break;
    }

    p->m_shape = NULL;
}

B2_API
void
b2FixtureCreateProxies(
    struct b2Fixture* p,
    struct b2BroadPhase* broadPhase,
    const b2Transform xf)
{
    int32 i;

    b2Assert(p->m_proxyCount == 0);

    // Create proxies in the broad-phase.
    p->m_proxyCount = b2ShapeGetChildCount(p->m_shape);

    for (i = 0; i < p->m_proxyCount; ++i)
    {
        struct b2FixtureProxy* proxy;
        proxy = p->m_proxies + i;
        b2ShapeComputeAABB(p->m_shape, &proxy->aabb, xf, i);
        proxy->proxyId = b2BroadPhaseCreateProxy(broadPhase, &proxy->aabb, proxy);
        proxy->fixture = p;
        proxy->childIndex = i;
    }
}

B2_API
void
b2FixtureDeleteProxies(
    struct b2Fixture* p,
    struct b2BroadPhase* broadPhase)
{
    int32 i;

    // Destroy proxies in the broad-phase.
    for (i = 0; i < p->m_proxyCount; ++i)
    {
        struct b2FixtureProxy* proxy;
        proxy = p->m_proxies + i;
        b2BroadPhaseDeleteProxy(broadPhase, proxy->proxyId);
        proxy->proxyId = b2BroadPhaseNullProxy;
    }

    p->m_proxyCount = 0;
}

B2_API
void
b2FixtureSynchronize(
    struct b2Fixture* p,
    struct b2BroadPhase* broadPhase,
    const b2Transform transform1,
    const b2Transform transform2)
{
    int32 i;

    if (p->m_proxyCount == 0)
    {
        return;
    }

    for (i = 0; i < p->m_proxyCount; ++i)
    {
        b2Vec2 v1, v2;
        b2Vec2 displacement;
        struct b2AABB aabb1, aabb2;
        struct b2FixtureProxy* proxy;

        proxy = p->m_proxies + i;

        // Compute an AABB that covers the swept shape (may miss some rotation effect).
        b2ShapeComputeAABB(p->m_shape, &aabb1, transform1, proxy->childIndex);
        b2ShapeComputeAABB(p->m_shape, &aabb2, transform2, proxy->childIndex);

        b2AABBCombine2(&proxy->aabb, &aabb1, &aabb2);

        b2AABBGetCenter(&aabb2, v2); 
        b2AABBGetCenter(&aabb1, v1);
        b2Vec2Sub(displacement, v2, v1);

        b2BroadPhaseMoveProxy(broadPhase, proxy->proxyId, &proxy->aabb, displacement);
    }
}
