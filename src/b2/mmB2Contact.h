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

#ifndef __mmB2Contact_h__
#define __mmB2Contact_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Collision.h"

#include "b2/mmB2Prefix.h"

struct b2BlockAllocator;
struct b2ContactListener;

/// Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
/// For example, anything slides on ice.
static
inline 
float 
b2MixFriction(
    float friction1, 
    float friction2)
{
    return b2Sqrt(friction1 * friction2);
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
static
inline
float 
b2MixRestitution(
    float restitution1, 
    float restitution2)
{
    return restitution1 > restitution2 ? restitution1 : restitution2;
}

/// Restitution mixing law. This picks the lowest value.
static
inline
float 
b2MixRestitutionThreshold(
    float threshold1, 
    float threshold2)
{
    return threshold1 < threshold2 ? threshold1 : threshold2;
}

struct b2ContactRegister
{
    /// struct b2Contact*
    /// (*Produce)(
    ///     struct b2Contact* obj,
    ///     struct b2Fixture* fixtureA, int32 indexA,
    ///     struct b2Fixture* fixtureB, int32 indexB);
    /// 
    /// void
    /// (*Recycle)(
    ///     struct b2Contact* contact);
    const struct b2MetaAllocator* meta;

    int primary;
};

/// A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body. Each contact has two contact
/// nodes, one for each attached body.
struct b2ContactEdge
{
    struct b2Body* other;       ///< provides quick access to the other body attached.
    struct b2Contact* contact;  ///< the contact
    struct b2ContactEdge* prev; ///< the previous contact edge in the body's contact list
    struct b2ContactEdge* next; ///< the next contact edge in the body's contact list
};

// Flags stored in m_flags
enum
{
	// Used when crawling contact graph when forming islands.
	b2ContactFlagIsland     = 0x0001,

	// Set when the shapes are touching.
	b2ContactFlagTouching   = 0x0002,

	// This contact can be disabled (by user)
	b2ContactFlagEnabled    = 0x0004,

	// This contact needs filtering because a fixture filter was changed.
	b2ContactFlagFilter     = 0x0008,

	// This bullet contact had a TOI event
	b2ContactFlagBulletHit  = 0x0010,

	// This contact has a valid TOI in m_toi
	b2ContactFlagToi        = 0x0020,
};

struct b2ContactMeta
{
    /// Evaluate this contact with your own manifold and transforms.
    /// void (*Evaluate)(
    ///     void* obj,
    ///     struct b2Manifold* manifold, 
    ///     const b2Transform xfA, 
    ///     const b2Transform xfB);
    void* Evaluate;
};

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
struct b2Contact
{
    uint32 m_flags;

    // World pool and list pointers.
    struct b2Contact* m_prev;
    struct b2Contact* m_next;

    // Nodes for connecting bodies.
    struct b2ContactEdge m_nodeA;
    struct b2ContactEdge m_nodeB;

    struct b2Fixture* m_fixtureA;
    struct b2Fixture* m_fixtureB;

    int32 m_indexA;
    int32 m_indexB;

    struct b2Manifold m_manifold;

    int32 m_toiCount;
    float m_toi;

    float m_friction;
    float m_restitution;
    float m_restitutionThreshold;

    float m_tangentSpeed;

    const struct b2ContactMeta* Meta;
};

/// Get the contact manifold. Do not modify the manifold unless you understand the
/// internals of Box2D.
static
inline
struct b2Manifold*
b2ContactGetManifoldRef(
    struct b2Contact* p)
{
    return &p->m_manifold;
}

static
inline
const struct b2Manifold*
b2ContactGetManifold(
    const struct b2Contact* p)
{
    return &p->m_manifold;
}

/// Get the world manifold.
B2_API
void
b2ContactGetWorldManifold(
    const struct b2Contact* p,
    struct b2WorldManifold* worldManifold);

/// Is this contact touching?
B2_API
int
b2ContactIsTouching(
    const struct b2Contact* p);

/// Enable/disable this contact. This can be used inside the pre-solve
/// contact listener. The contact is only disabled for the current
/// time step (or sub-step in continuous collisions).
B2_API
void
b2ContactSetEnabled(
    struct b2Contact* p,
    int flag);

/// Has this contact been disabled?
B2_API
int
b2ContactIsEnabled(
    const struct b2Contact* p);

/// Get the next contact in the world's contact list.
static
inline
struct b2Contact*
b2ContactGetNextRef(
    struct b2Contact* p)
{
    return p->m_next;
}

static
inline
const struct b2Contact*
b2ContactGetNext(
    const struct b2Contact* p)
{
    return p->m_next;
}

/// Get fixture A in this contact.
static
inline
struct b2Fixture* 
b2ContactGetFixtureARef(
    struct b2Contact* p)
{
    return p->m_fixtureA;
}

static
inline
const struct b2Fixture* 
b2ContactGetFixtureA(
    const struct b2Contact* p)
{
    return p->m_fixtureA;
}

/// Get the child primitive index for fixture A.
static
inline
int32 
b2ContactGetChildIndexA(
    const struct b2Contact* p)
{
    return p->m_indexA;
}

/// Get fixture B in this contact.
static
inline
struct b2Fixture* 
b2ContactGetFixtureBRef(
    struct b2Contact* p)
{
    return p->m_fixtureB;
}

static
inline
const struct b2Fixture* 
b2ContactGetFixtureB(
    const struct b2Contact* p)
{
    return p->m_fixtureB;
}

/// Get the child primitive index for fixture B.
static
inline
int32 
b2ContactGetChildIndexB(
    const struct b2Contact* p)
{
    return p->m_indexB;
}

/// Override the default friction mixture. You can call this in b2ContactListener::PreSolve.
/// This value persists until set or reset.
static
inline
void 
b2ContactSetFriction(
    struct b2Contact* p,
    float friction)
{
    p->m_friction = friction;
}

/// Get the friction.
static
inline
float 
b2ContactGetFriction(
    const struct b2Contact* p)
{
    return p->m_friction;
}

/// Reset the friction mixture to the default value.
B2_API
void
b2ContactResetFriction(
    struct b2Contact* p);

/// Override the default restitution mixture. You can call this in b2ContactListener::PreSolve.
/// The value persists until you set or reset.
static
inline
void 
b2ContactSetRestitution(
    struct b2Contact* p,
    float restitution)
{
    p->m_restitution = restitution;
}

/// Get the restitution.
static
inline
float 
b2ContactGetRestitution(
    const struct b2Contact* p)
{
    return p->m_restitution;
}

/// Reset the restitution to the default value.
B2_API
void
b2ContactResetRestitution(
    struct b2Contact* p);

/// Override the default restitution velocity threshold mixture. You can call this in b2ContactListener::PreSolve.
/// The value persists until you set or reset.
static
inline
void 
b2ContactSetRestitutionThreshold(
    struct b2Contact* p,
    float threshold)
{
    p->m_restitutionThreshold = threshold;
}

/// Get the restitution threshold.
static
inline
float 
b2ContactGetRestitutionThreshold(
    const struct b2Contact* p)
{
    return p->m_restitutionThreshold;
}

/// Reset the restitution threshold to the default value.
B2_API
void
b2ContactResetRestitutionThreshold(
    struct b2Contact* p);

/// Set the desired tangent speed for a conveyor belt behavior. In meters per second.
static
inline
void 
b2ContactSetTangentSpeed(
    struct b2Contact* p,
    float speed)
{
    p->m_tangentSpeed = speed;
}

/// Get the desired tangent speed. In meters per second.
static
inline
float 
b2ContactGetTangentSpeed(
    const struct b2Contact* p)
{
    return p->m_tangentSpeed;
}

/// Evaluate this contact with your own manifold and transforms.
B2_API
void
b2ContactEvaluate(
    struct b2Contact* obj,
    struct b2Manifold* manifold,
    const b2Transform xfA, 
    const b2Transform xfB);

B2_API
void
b2ContactFlagForFiltering(
    struct b2Contact* p);

B2_API
void
b2ContactPrepare(
    struct b2Contact* p,
    struct b2Fixture* fixtureA, int32 indexA, 
    struct b2Fixture* fixtureB, int32 indexB);

B2_API
void
b2ContactDiscard(
    struct b2Contact* p);

B2_API
struct b2Contact*
b2ContactCreate(
    struct b2Fixture* fixtureA, int32 indexA,
    struct b2Fixture* fixtureB, int32 indexB,
    struct b2BlockAllocator* allocator);

B2_API
void
b2ContactDelete(
    struct b2Contact* contact,
    struct b2BlockAllocator* allocator);

B2_API
void
b2ContactUpdate(
    struct b2Contact* p,
    struct b2ContactListener* listener);

#include "b2/mmB2Suffix.h"

#endif//__mmB2Contact_h__
