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

#include "mmB2ContactChainPolygon.h"
#include "mmB2Contact.h"
#include "mmB2Fixture.h"
#include "mmB2BlockAllocator.h"
#include "mmB2Shape.h"
#include "mmB2ShapeChain.h"
#include "mmB2ShapePolygon.h"
#include "mmB2ShapeEdge.h"

#include <assert.h>

B2_API const struct b2MetaAllocator b2MetaAllocatorContactChainAndPolygon =
{
    "b2ContactChainAndPolygon",
    sizeof(struct b2Contact),
    &b2ContactChainAndPolygonPrepare,
    &b2ContactChainAndPolygonDiscard,
};

B2_API const struct b2ContactMeta b2ContactMetaChainAndPolygon =
{
    &b2ContactChainAndPolygonEvaluate,
};

B2_API
void
b2ContactChainAndPolygonPrepare(
    struct b2Contact* p,
    struct b2Fixture* fixtureA, int32 indexA,
    struct b2Fixture* fixtureB, int32 indexB)
{
    b2ContactPrepare(p, fixtureA, indexA, fixtureB, indexB);
    p->Meta = &b2ContactMetaChainAndPolygon;
    b2Assert(b2FixtureGetType(p->m_fixtureA) == b2ShapeTypeChain);
    b2Assert(b2FixtureGetType(p->m_fixtureB) == b2ShapeTypePolygon);
}

B2_API
void
b2ContactChainAndPolygonDiscard(
    struct b2Contact* p)
{
    p->Meta = &b2ContactMetaChainAndPolygon;
    b2ContactDiscard(p);
}

B2_API
void
b2ContactChainAndPolygonEvaluate(
    struct b2Contact* p,
    struct b2Manifold* manifold,
    const b2Transform xfA,
    const b2Transform xfB)
{
    struct b2ShapeChain* chain = (struct b2ShapeChain*)b2FixtureGetShape(p->m_fixtureA);
    struct b2ShapePolygon* polygon = (struct b2ShapePolygon*)b2FixtureGetShape(p->m_fixtureB);
    struct b2ShapeEdge edge;
    b2ShapeChainGetChildEdge(chain, &edge, p->m_indexA);
    b2CollideEdgeAndPolygon(manifold, &edge, xfA, polygon, xfB);
}
