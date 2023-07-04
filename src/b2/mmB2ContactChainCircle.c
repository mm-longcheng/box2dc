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

#include "mmB2ContactChainCircle.h"
#include "mmB2Contact.h"
#include "mmB2Fixture.h"
#include "mmB2BlockAllocator.h"
#include "mmB2Shape.h"
#include "mmB2ShapeChain.h"
#include "mmB2ShapeCircle.h"
#include "mmB2ShapeEdge.h"

#include <assert.h>

B2_API const struct b2MetaAllocator b2MetaAllocatorContactChainAndCircle =
{
    "b2ContactChainAndCircle",
    sizeof(struct b2Contact),
    &b2ContactChainAndCirclePrepare,
    &b2ContactChainAndCircleDiscard,
};

B2_API const struct b2ContactMeta b2ContactMetaChainAndCircle =
{
    &b2ContactChainAndCircleEvaluate,
};

B2_API
void
b2ContactChainAndCirclePrepare(
    struct b2Contact* p,
    struct b2Fixture* fixtureA,int32 indexA,
    struct b2Fixture* fixtureB,int32 indexB)
{
    b2ContactPrepare(p, fixtureA, indexA, fixtureB, indexB);
    p->Meta = &b2ContactMetaChainAndCircle;
    b2Assert(b2FixtureGetType(p->m_fixtureA) == b2ShapeTypeChain);
    b2Assert(b2FixtureGetType(p->m_fixtureB) == b2ShapeTypeCircle);
}

B2_API
void
b2ContactChainAndCircleDiscard(
    struct b2Contact* p)
{
    p->Meta = &b2ContactMetaChainAndCircle;
    b2ContactDiscard(p);
}

B2_API
void
b2ContactChainAndCircleEvaluate(
    struct b2Contact* p,
    struct b2Manifold* manifold,
    const b2Transform xfA,
    const b2Transform xfB)
{
    struct b2ShapeChain* chain;
    struct b2ShapeCircle* circle;
    struct b2ShapeEdge edge;

    chain = (struct b2ShapeChain*)b2FixtureGetShape(p->m_fixtureA);
    circle = (struct b2ShapeCircle*)b2FixtureGetShape(p->m_fixtureB);
    b2ShapeChainGetChildEdge(chain, &edge, p->m_indexA);
    b2CollideEdgeAndCircle(manifold, &edge, xfA, circle, xfB);
}
