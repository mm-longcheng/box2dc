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

#ifndef __mmB2JointFriction_h__
#define __mmB2JointFriction_h__

#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Joint.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

B2_API extern const struct b2MetaAllocator b2MetaAllocatorJointFriction;
B2_API extern const struct b2JointMeta b2JointFrictionMeta;

/// Friction joint definition.
struct b2JointFrictionDef
{
    b2JointDefSuper;

    /// The local anchor point relative to bodyA's origin.
    b2Vec2 localAnchorA;

    /// The local anchor point relative to bodyB's origin.
    b2Vec2 localAnchorB;

    /// The maximum friction force in N.
    float maxForce;

    /// The maximum friction torque in N-m.
    float maxTorque;
};

B2_API
void
b2JointFrictionDefReset(
    struct b2JointFrictionDef* p);

/// Initialize the bodies, anchors, axis, and reference angle using the world
/// anchor and world axis.
B2_API
void
b2JointFrictionDefInitialize(
    struct b2JointFrictionDef* p,
    struct b2Body* bodyA, 
    struct b2Body* bodyB, 
    const b2Vec2 anchor);

/// Friction joint. This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
struct b2JointFriction
{
    b2JointSuper;

    b2Vec2 m_localAnchorA;
    b2Vec2 m_localAnchorB;

    // Solver shared
    b2Vec2 m_linearImpulse;
    float m_angularImpulse;
    float m_maxForce;
    float m_maxTorque;

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
    b2Mat22 m_linearMass;
    float m_angularMass;
};

B2_API
void
b2JointFrictionPrepare(
    struct b2JointFriction* p,
    const struct b2JointFrictionDef* def);

B2_API
void
b2JointFrictionDiscard(
    struct b2JointFriction* p);

B2_API
void
b2JointFrictionGetAnchorA(
    const struct b2JointFriction* p, 
    b2Vec2 anchor);

B2_API
void
b2JointFrictionGetAnchorB(
    const struct b2JointFriction* p, 
    b2Vec2 anchor);

B2_API
void
b2JointFrictionGetReactionForce(
    const struct b2JointFriction* p, 
    float inv_dt, 
    b2Vec2 force);

B2_API
float
b2JointFrictionGetReactionTorque(
    const struct b2JointFriction* p, 
    float inv_dt);

B2_API
void
b2JointFrictionDump(
    const struct b2JointFriction* p);

B2_API
void
b2JointFrictionShiftOrigin(
    struct b2JointFriction* p, 
    const b2Vec2 newOrigin);

B2_API
void
b2JointFrictionDraw(
    const struct b2JointFriction* p, 
    struct b2Draw* draw);

B2_API
void
b2JointFrictionInitVelocityConstraints(
    struct b2JointFriction* p, 
    const struct b2SolverData* data);

B2_API
void
b2JointFrictionSolveVelocityConstraints(
    struct b2JointFriction* p, 
    const struct b2SolverData* data);

B2_API
int
b2JointFrictionSolvePositionConstraints(
    struct b2JointFriction* p, 
    const struct b2SolverData* data);


/// The local anchor point relative to bodyA's origin.
static
inline
b2Vec2ConstRef
b2JointFrictionGetLocalAnchorA(
    const struct b2JointFriction* p)
{
    return p->m_localAnchorA;
}

/// The local anchor point relative to bodyB's origin.
static
inline
b2Vec2ConstRef
b2JointFrictionGetLocalAnchorB(
    const struct b2JointFriction* p)
{
    return p->m_localAnchorB;
}

/// Set the maximum friction force in N.
B2_API
void
b2JointFrictionSetMaxForce(
    struct b2JointFriction* p,
    float force);

/// Get the maximum friction force in N.
static
inline
float
b2JointFrictionGetMaxForce(
    const struct b2JointFriction* p)
{
    return p->m_maxForce;
}

/// Set the maximum friction torque in N*m.
B2_API
void
b2JointFrictionSetMaxTorque(
    struct b2JointFriction* p,
    float torque);

/// Get the maximum friction torque in N*m.
static
inline
float
b2JointFrictionGetMaxTorque(
    const struct b2JointFriction* p)
{
    return p->m_maxTorque;
}

#include "b2/mmB2Suffix.h"

#endif//__mmB2JointFriction_h__
