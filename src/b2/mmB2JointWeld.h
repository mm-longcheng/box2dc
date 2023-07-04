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

#ifndef __mmB2JointWeld_h__
#define __mmB2JointWeld_h__

#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Joint.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

B2_API extern const struct b2MetaAllocator b2MetaAllocatorJointWeld;
B2_API extern const struct b2JointMeta b2JointWeldMeta;

/// Weld joint definition. You need to specify local anchor points
/// where they are attached and the relative body angle. The position
/// of the anchor points is important for computing the reaction torque.
struct b2JointWeldDef
{
    b2JointDefSuper;

    /// The local anchor point relative to bodyA's origin.
    b2Vec2 localAnchorA;

    /// The local anchor point relative to bodyB's origin.
    b2Vec2 localAnchorB;

    /// The bodyB angle minus bodyA angle in the reference state (radians).
    float referenceAngle;

    /// The rotational stiffness in N*m
    /// Disable softness with a value of 0
    float stiffness;

    /// The rotational damping in N*m*s
    float damping;
};

B2_API
void
b2JointWeldDefReset(
    struct b2JointWeldDef* p);

/// Initialize the bodies, anchors, reference angle, stiffness, and damping.
/// @param bodyA the first body connected by this joint
/// @param bodyB the second body connected by this joint
/// @param anchor the point of connection in world coordinates
B2_API
void
b2JointWeldDefInitialize(
    struct b2JointWeldDef* p,
    struct b2Body* bodyA, 
    struct b2Body* bodyB, 
    const b2Vec2 anchor);

/// A weld joint essentially glues two bodies together. A weld joint may
/// distort somewhat because the island constraint solver is approximate.
struct b2JointWeld
{
    b2JointSuper;

    float m_stiffness;
    float m_damping;
    float m_bias;

    // Solver shared
    b2Vec2 m_localAnchorA;
    b2Vec2 m_localAnchorB;
    float m_referenceAngle;
    float m_gamma;
    b2Vec3 m_impulse;

    // Solver temp
    int32 m_indexA;
    int32 m_indexB;
    b2Vec2 m_rA;
    b2Vec2 m_rB;
    b2Vec2 m_localCenterA;
    b2Vec2 m_localCenterB;
    float m_invMassA;
    float m_invMassB;
    float m_invIA;
    float m_invIB;
    b2Mat33 m_mass;
};

B2_API
void
b2JointWeldPrepare(
    struct b2JointWeld* p,
    const struct b2JointWeldDef* def);

B2_API
void
b2JointWeldDiscard(
    struct b2JointWeld* p);

B2_API
void
b2JointWeldGetAnchorA(
    const struct b2JointWeld* p, 
    b2Vec2 anchor);

B2_API
void
b2JointWeldGetAnchorB(
    const struct b2JointWeld* p, 
    b2Vec2 anchor);

B2_API
void
b2JointWeldGetReactionForce(
    const struct b2JointWeld* p, 
    float inv_dt, 
    b2Vec2 force);

B2_API
float
b2JointWeldGetReactionTorque(
    const struct b2JointWeld* p, 
    float inv_dt);

B2_API
void
b2JointWeldDump(
    const struct b2JointWeld* p);

B2_API
void
b2JointWeldShiftOrigin(
    struct b2JointWeld* p, 
    const b2Vec2 newOrigin);

B2_API
void
b2JointWeldDraw(
    const struct b2JointWeld* p, 
    struct b2Draw* draw);

B2_API
void
b2JointWeldInitVelocityConstraints(
    struct b2JointWeld* p, 
    const struct b2SolverData* data);

B2_API
void
b2JointWeldSolveVelocityConstraints(
    struct b2JointWeld* p, 
    const struct b2SolverData* data);

B2_API
int
b2JointWeldSolvePositionConstraints(
    struct b2JointWeld* p, 
    const struct b2SolverData* data);

/// The local anchor point relative to bodyA's origin.
static
inline
b2Vec2ConstRef
b2JointWeldGetLocalAnchorA(
    const struct b2JointWeld* p)
{
    return p->m_localAnchorA;
}

/// The local anchor point relative to bodyB's origin.
static
inline
b2Vec2ConstRef
b2JointWeldGetLocalAnchorB(
    const struct b2JointWeld* p)
{
    return p->m_localAnchorB;
}

/// Get the reference angle.
static
inline
float
b2JointWeldGetReferenceAngle(
    const struct b2JointWeld* p)
{
    return p->m_referenceAngle;
}

/// Set/get stiffness in N*m
static
inline
void
b2JointWeldSetStiffness(
    struct b2JointWeld* p,
    float stiffness)
{
    p->m_stiffness = stiffness;
}

static
inline
float
b2JointWeldGetStiffness(
    const struct b2JointWeld* p)
{
    return p->m_stiffness;
}

/// Set/get damping in N*m*s
static
inline
void
b2JointWeldSetDamping(
    struct b2JointWeld* p,
    float damping)
{
    p->m_damping = damping;
}

static
inline
float
b2JointWeldGetDamping(
    const struct b2JointWeld* p)
{
    return p->m_damping;
}

#include "b2/mmB2Suffix.h"

#endif//__mmB2JointWeld_h__
