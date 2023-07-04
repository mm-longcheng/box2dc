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

#ifndef __mmB2BroadPhase_h__
#define __mmB2BroadPhase_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2DynamicTree.h"

#include "b2/mmB2Prefix.h"

struct b2Pair
{
    int32 proxyIdA;
    int32 proxyIdB;
};

enum
{
    b2BroadPhaseNullProxy = -1,
};

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
struct b2BroadPhase
{
    struct b2DynamicTree m_tree;

    int32 m_proxyCount;

    int32* m_moveBuffer;
    int32 m_moveCapacity;
    int32 m_moveCount;

    struct b2Pair* m_pairBuffer;
    int32 m_pairCapacity;
    int32 m_pairCount;

    int32 m_queryProxyId;
};

B2_API
void
b2BroadPhaseInit(
    struct b2BroadPhase* p);

B2_API
void
b2BroadPhaseDestroy(
    struct b2BroadPhase* p);

/// Create a proxy with an initial AABB. Pairs are not reported until
/// UpdatePairs is called.
B2_API
int32
b2BroadPhaseCreateProxy(
    struct b2BroadPhase* p,
    const struct b2AABB* aabb, 
    void* userData);

/// Delete a proxy. It is up to the client to remove any pairs.
B2_API
void
b2BroadPhaseDeleteProxy(
    struct b2BroadPhase* p, 
    int32 proxyId);

/// Call MoveProxy as many times as you like, then when you are done
/// call UpdatePairs to finalized the proxy pairs (for your time step).
B2_API
void
b2BroadPhaseMoveProxy(
    struct b2BroadPhase* p,
    int32 proxyId,
    const struct b2AABB* aabb,
    const b2Vec2 displacement);

/// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
B2_API
void
b2BroadPhaseTouchProxy(
    struct b2BroadPhase* p,
    int32 proxyId);

/// Get the fat AABB for a proxy.
B2_API
const struct b2AABB*
b2BroadPhaseGetFatAABB(
    const struct b2BroadPhase* p,
    int32 proxyId);

/// Get user data from a proxy. Returns nullptr if the id is invalid.
B2_API
void*
b2BroadPhaseGetUserData(
    const struct b2BroadPhase* p,
    int32 proxyId);

/// Test overlap of fat AABBs.
B2_API
int
b2BroadPhaseTestOverlap(
    const struct b2BroadPhase* p,
    int32 proxyIdA,
    int32 proxyIdB);

/// Get the number of proxies.
B2_API
int32
b2BroadPhaseGetProxyCount(
    const struct b2BroadPhase* p);

/// Get the height of the embedded tree.
B2_API
int32
b2BroadPhaseGetTreeHeight(
    const struct b2BroadPhase* p);

/// Get the balance of the embedded tree.
B2_API
int32
b2BroadPhaseGetTreeBalance(
    const struct b2BroadPhase* p);

/// Get the quality metric of the embedded tree.
B2_API
float
b2BroadPhaseGetTreeQuality(
    const struct b2BroadPhase* p);

/// Shift the world origin. Useful for large worlds.
/// The shift formula is: position -= newOrigin
/// @param newOrigin the new origin with respect to the old origin
B2_API
void
b2BroadPhaseShiftOrigin(
    struct b2BroadPhase* p,
    const b2Vec2 newOrigin);

/// Update the pairs. This results in pair callbacks. This can only add pairs.
/// typedef void(*AddPair)(void* obj, void* userDataA, void* userDataB);
B2_API
void
b2BroadPhaseUpdatePairs(
    struct b2BroadPhase* p,
    void* obj,
    void* callback);

/// Query an AABB for overlapping proxies. The callback class
/// is called for each proxy that overlaps the supplied AABB.
/// typedef int(*QueryCallback)(void* obj, int32 nodeId);
B2_API
void
b2BroadPhaseQuery(
    const struct b2BroadPhase* p,
    const struct b2AABB* aabb,
    void* obj,
    void* callback);

/// Ray-cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray-cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
/// @param callback a callback class that is called for each proxy that is hit by the ray.
/// typedef float(*RayCast)(void* obj, struct b2RayCastInput* subInput, int32 nodeId);
B2_API
void
b2BroadPhaseRayCast(
    const struct b2BroadPhase* p,
    const struct b2RayCastInput* input,
    void* obj,
    void* callback);

#include "b2/mmB2Suffix.h"

#endif//__mmB2BroadPhase_h__
