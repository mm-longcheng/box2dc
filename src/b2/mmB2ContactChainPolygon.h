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

#ifndef __mmB2ContactChainPolygon_h__
#define __mmB2ContactChainPolygon_h__

#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

struct b2Contact;
struct b2Fixture;
struct b2Manifold;

B2_API extern const struct b2MetaAllocator b2MetaAllocatorContactChainAndPolygon;
B2_API extern const struct b2ContactMeta b2ContactMetaChainAndPolygon;

B2_API
void
b2ContactChainAndPolygonPrepare(
    struct b2Contact* p,
    struct b2Fixture* fixtureA, int32 indexA,
    struct b2Fixture* fixtureB, int32 indexB);

B2_API
void
b2ContactChainAndPolygonDiscard(
    struct b2Contact* p);

B2_API
void
b2ContactChainAndPolygonEvaluate(
    struct b2Contact* p,
    struct b2Manifold* manifold,
    const b2Transform xfA,
    const b2Transform xfB);

#include "b2/mmB2Suffix.h"

#endif//__mmB2ContactChainPolygon_h__
