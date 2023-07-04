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

#include "mmB2BroadPhase.h"

#include <string.h>

static
void 
b2BroadPhaseBufferMove(
    struct b2BroadPhase* p,
    int32 proxyId)
{
    if (p->m_moveCount == p->m_moveCapacity)
    {
        int32* oldBuffer = p->m_moveBuffer;
        p->m_moveCapacity *= 2;
        p->m_moveBuffer = (int32*)b2Alloc(p->m_moveCapacity * sizeof(int32));
        memcpy(p->m_moveBuffer, oldBuffer, p->m_moveCount * sizeof(int32));
        b2Free(oldBuffer);
    }

    p->m_moveBuffer[p->m_moveCount] = proxyId;
    ++p->m_moveCount;
}

static
void 
b2BroadPhaseUnBufferMove(
    struct b2BroadPhase* p,
    int32 proxyId)
{
    int32 i;
    for (i = 0; i < p->m_moveCount; ++i)
    {
        if (p->m_moveBuffer[i] == proxyId)
        {
            p->m_moveBuffer[i] = b2BroadPhaseNullProxy;
        }
    }
}

static
int 
b2BroadPhaseQueryCallback(
    struct b2BroadPhase* p,
    int32 proxyId)
{
    int moved;

    // A proxy cannot form a pair with itself.
    if (proxyId == p->m_queryProxyId)
    {
        return b2True;
    }

    moved = b2DynamicTreeWasMoved(&p->m_tree, proxyId);
    if (moved && proxyId > p->m_queryProxyId)
    {
        // Both proxies are moving. Avoid duplicate pairs.
        return b2True;
    }

    // Grow the pair buffer as needed.
    if (p->m_pairCount == p->m_pairCapacity)
    {
        struct b2Pair* oldBuffer = p->m_pairBuffer;
        p->m_pairCapacity = p->m_pairCapacity + (p->m_pairCapacity >> 1);
        p->m_pairBuffer = (struct b2Pair*)b2Alloc(p->m_pairCapacity * sizeof(struct b2Pair));
        memcpy(p->m_pairBuffer, oldBuffer, p->m_pairCount * sizeof(struct b2Pair));
        b2Free(oldBuffer);
    }

    p->m_pairBuffer[p->m_pairCount].proxyIdA = b2MinInt32(proxyId, p->m_queryProxyId);
    p->m_pairBuffer[p->m_pairCount].proxyIdB = b2MaxInt32(proxyId, p->m_queryProxyId);
    ++p->m_pairCount;

    return b2True;
}

B2_API
void
b2BroadPhaseInit(
    struct b2BroadPhase* p)
{
    b2DynamicTreeInit(&p->m_tree);

    p->m_proxyCount = 0;

    p->m_pairCapacity = 16;
    p->m_pairCount = 0;
    p->m_pairBuffer = (struct b2Pair*)b2Alloc(p->m_pairCapacity * sizeof(struct b2Pair));

    p->m_moveCapacity = 16;
    p->m_moveCount = 0;
    p->m_moveBuffer = (int32*)b2Alloc(p->m_moveCapacity * sizeof(int32));
}

B2_API
void
b2BroadPhaseDestroy(
    struct b2BroadPhase* p)
{
    b2Free(p->m_moveBuffer);
    b2Free(p->m_pairBuffer);

    b2DynamicTreeDestroy(&p->m_tree);
}

B2_API
int32
b2BroadPhaseCreateProxy(
    struct b2BroadPhase* p,
    const struct b2AABB* aabb,
    void* userData)
{
    int32 proxyId = b2DynamicTreeCreateProxy(&p->m_tree, aabb, userData);
    ++p->m_proxyCount;
    b2BroadPhaseBufferMove(p, proxyId);
    return proxyId;
}

B2_API
void
b2BroadPhaseDeleteProxy(
    struct b2BroadPhase* p,
    int32 proxyId)
{
    b2BroadPhaseUnBufferMove(p, proxyId);
    --p->m_proxyCount;
    b2DynamicTreeDeleteProxy(&p->m_tree, proxyId);
}

B2_API
void
b2BroadPhaseMoveProxy(
    struct b2BroadPhase* p,
    int32 proxyId,
    const struct b2AABB* aabb,
    const b2Vec2 displacement)
{
    int buffer = b2DynamicTreeMoveProxy(&p->m_tree, proxyId, aabb, displacement);
    if (buffer)
    {
        b2BroadPhaseBufferMove(p, proxyId);
    }
}

B2_API
void
b2BroadPhaseTouchProxy(
    struct b2BroadPhase* p,
    int32 proxyId)
{
    b2BroadPhaseBufferMove(p, proxyId);
}

B2_API
const struct b2AABB*
b2BroadPhaseGetFatAABB(
    const struct b2BroadPhase* p,
    int32 proxyId)
{
    return b2DynamicTreeGetFatAABB(&p->m_tree, proxyId);
}

B2_API
void*
b2BroadPhaseGetUserData(
    const struct b2BroadPhase* p,
    int32 proxyId)
{
    return b2DynamicTreeGetUserData(&p->m_tree, proxyId);
}

B2_API
int
b2BroadPhaseTestOverlap(
    const struct b2BroadPhase* p,
    int32 proxyIdA,
    int32 proxyIdB)
{
    const struct b2AABB* aabbA = b2DynamicTreeGetFatAABB(&p->m_tree, proxyIdA);
    const struct b2AABB* aabbB = b2DynamicTreeGetFatAABB(&p->m_tree, proxyIdB);
    return b2AABBTestOverlap(aabbA, aabbB);
}

B2_API
int32
b2BroadPhaseGetProxyCount(
    const struct b2BroadPhase* p)
{
    return p->m_proxyCount;
}

B2_API
int32
b2BroadPhaseGetTreeHeight(
    const struct b2BroadPhase* p)
{
    return b2DynamicTreeGetHeight(&p->m_tree);
}

B2_API
int32
b2BroadPhaseGetTreeBalance(
    const struct b2BroadPhase* p)
{
    return b2DynamicTreeGetMaxBalance(&p->m_tree);
}

B2_API
float
b2BroadPhaseGetTreeQuality(
    const struct b2BroadPhase* p)
{
    return b2DynamicTreeGetAreaRatio(&p->m_tree);
}

B2_API
void
b2BroadPhaseShiftOrigin(
    struct b2BroadPhase* p,
    const b2Vec2 newOrigin)
{
    b2DynamicTreeShiftOrigin(&p->m_tree, newOrigin);
}

B2_API
void
b2BroadPhaseUpdatePairs(
    struct b2BroadPhase* p,
    void* obj,
    void* callback)
{
    typedef void(*AddPair)(void* obj, void* userDataA, void* userDataB);

    int32 i;

    // Reset pair buffer
    p->m_pairCount = 0;

    // Perform tree queries for all moving proxies.
    for (i = 0; i < p->m_moveCount; ++i)
    {
        const struct b2AABB* fatAABB;

        p->m_queryProxyId = p->m_moveBuffer[i];
        if (p->m_queryProxyId == b2BroadPhaseNullProxy)
        {
            continue;
        }

        // We have to query the tree with the fat AABB so that
        // we don't fail to create a pair that may touch later.
        fatAABB = b2DynamicTreeGetFatAABB(&p->m_tree, p->m_queryProxyId);

        // Query tree, create pairs and add them pair buffer.
        b2DynamicTreeQuery(&p->m_tree, fatAABB, p, &b2BroadPhaseQueryCallback);
    }

    // Send pairs to caller
    for (i = 0; i < p->m_pairCount; ++i)
    {
        struct b2Pair* primaryPair = p->m_pairBuffer + i;
        void* userDataA = b2DynamicTreeGetUserData(&p->m_tree, primaryPair->proxyIdA);
        void* userDataB = b2DynamicTreeGetUserData(&p->m_tree, primaryPair->proxyIdB);

        (*((AddPair)callback))(obj, userDataA, userDataB);
    }

    // Clear move flags
    for (i = 0; i < p->m_moveCount; ++i)
    {
        int32 proxyId = p->m_moveBuffer[i];
        if (proxyId == b2BroadPhaseNullProxy)
        {
            continue;
        }

        b2DynamicTreeClearMoved(&p->m_tree, proxyId);
    }

    // Reset move buffer
    p->m_moveCount = 0;
}

B2_API
void
b2BroadPhaseQuery(
    const struct b2BroadPhase* p,
    const struct b2AABB* aabb,
    void* obj,
    void* callback)
{
    b2DynamicTreeQuery(&p->m_tree, aabb, obj, callback);
}

B2_API
void
b2BroadPhaseRayCast(
    const struct b2BroadPhase* p,
    const struct b2RayCastInput* input,
    void* obj,
    void* callback)
{
    b2DynamicTreeRayCast(&p->m_tree, input, obj, callback);
}
