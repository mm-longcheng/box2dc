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

#ifndef __mmB2JointDistance_h__
#define __mmB2JointDistance_h__

#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Joint.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

B2_API extern const struct b2MetaAllocator b2MetaAllocatorJointDistance;
B2_API extern const struct b2JointMeta b2JointDistanceMeta;

/// Distance joint definition. This requires defining an anchor point on both
/// bodies and the non-zero distance of the distance joint. The definition uses
/// local anchor points so that the initial configuration can violate the
/// constraint slightly. This helps when saving and loading a game.
struct b2JointDistanceDef
{
    b2JointDefSuper;

    /// The local anchor point relative to bodyA's origin.
    b2Vec2 localAnchorA;

    /// The local anchor point relative to bodyB's origin.
    b2Vec2 localAnchorB;

    /// The rest length of this joint. Clamped to a stable minimum value.
    float length;

    /// Minimum length. Clamped to a stable minimum value.
    float minLength;

    /// Maximum length. Must be greater than or equal to the minimum length.
    float maxLength;

    /// The linear stiffness in N/m.
    float stiffness;

    /// The linear damping in N*s/m.
    float damping;
};

B2_API
void
b2JointDistanceDefReset(
    struct b2JointDistanceDef* p);

/// Initialize the bodies, anchors, and rest length using world space anchors.
/// The minimum and maximum lengths are set to the rest length.
B2_API
void
b2JointDistanceDefInitialize(
    struct b2JointDistanceDef* p,
    struct b2Body* bodyA, 
    struct b2Body* bodyB,
    const b2Vec2 anchorA, 
    const b2Vec2 anchorB);

/// A distance joint constrains two points on two bodies to remain at a fixed
/// distance from each other. You can view this as a massless, rigid rod.
struct b2JointDistance
{
    b2JointSuper;

    float m_stiffness;
    float m_damping;
    float m_bias;
    float m_length;
    float m_minLength;
    float m_maxLength;

    // Solver shared
    b2Vec2 m_localAnchorA;
    b2Vec2 m_localAnchorB;
    float m_gamma;
    float m_impulse;
    float m_lowerImpulse;
    float m_upperImpulse;

    // Solver temp
    int32 m_indexA;
    int32 m_indexB;
    b2Vec2 m_u;
    b2Vec2 m_rA;
    b2Vec2 m_rB;
    b2Vec2 m_localCenterA;
    b2Vec2 m_localCenterB;
    float m_currentLength;
    float m_invMassA;
    float m_invMassB;
    float m_invIA;
    float m_invIB;
    float m_softMass;
    float m_mass;
};

B2_API
void
b2JointDistancePrepare(
    struct b2JointDistance* p,
    const struct b2JointDistanceDef* def);

B2_API
void
b2JointDistanceDiscard(
    struct b2JointDistance* p);

B2_API
void
b2JointDistanceGetAnchorA(
    const struct b2JointDistance* p, 
    b2Vec2 anchor);

B2_API
void
b2JointDistanceGetAnchorB(
    const struct b2JointDistance* p, 
    b2Vec2 anchor);

/// Get the reaction force given the inverse time step.
/// Unit is N.
B2_API
void
b2JointDistanceGetReactionForce(
    const struct b2JointDistance* p, 
    float inv_dt, 
    b2Vec2 force);

/// Get the reaction torque given the inverse time step.
/// Unit is N*m. This is always zero for a distance joint.
B2_API
float
b2JointDistanceGetReactionTorque(
    const struct b2JointDistance* p, 
    float inv_dt);

/// Dump joint to dmLog
B2_API
void
b2JointDistanceDump(
    const struct b2JointDistance* p);

B2_API
void
b2JointDistanceShiftOrigin(
    struct b2JointDistance* p, 
    const b2Vec2 newOrigin);

B2_API
void
b2JointDistanceDraw(
    const struct b2JointDistance* p, 
    struct b2Draw* draw);

B2_API
void
b2JointDistanceInitVelocityConstraints(
    struct b2JointDistance* p, 
    const struct b2SolverData* data);

B2_API
void
b2JointDistanceSolveVelocityConstraints(
    struct b2JointDistance* p, 
    const struct b2SolverData* data);

B2_API
int
b2JointDistanceSolvePositionConstraints(
    struct b2JointDistance* p, 
    const struct b2SolverData* data);


/// The local anchor point relative to bodyA's origin.
static
inline
b2Vec2ConstRef
b2JointDistanceGetLocalAnchorA(
    const struct b2JointDistance* p) 
{
    return p->m_localAnchorA;
}

/// The local anchor point relative to bodyB's origin.
static
inline
b2Vec2ConstRef
b2JointDistanceGetLocalAnchorB(
    const struct b2JointDistance* p) 
{
    return p->m_localAnchorB;
}

/// Get the rest length
static
inline
float
b2JointDistanceGetLength(
    const struct b2JointDistance* p)
{
    return p->m_length;
}

/// Set the rest length
/// @returns clamped rest length
B2_API
float
b2JointDistanceSetLength(
    struct b2JointDistance* p,
    float length);

/// Get the minimum length
static
inline
float
b2JointDistanceGetMinLength(
    const struct b2JointDistance* p)
{
    return p->m_minLength;
}

/// Set the minimum length
/// @returns the clamped minimum length
B2_API
float
b2JointDistanceSetMinLength(
    struct b2JointDistance* p,
    float minLength);

/// Get the maximum length
static
inline
float
b2JointDistanceGetMaxLength(
    const struct b2JointDistance* p) 
{
    return p->m_maxLength;
}

/// Set the maximum length
/// @returns the clamped maximum length
B2_API
float
b2JointDistanceSetMaxLength(
    struct b2JointDistance* p,
    float maxLength);

/// Get the current length
B2_API
float
b2JointDistanceGetCurrentLength(
    const struct b2JointDistance* p);

/// Set/get the linear stiffness in N/m
static
inline
void
b2JointDistanceSetStiffness(
    struct b2JointDistance* p,
    float stiffness) 
{
    p->m_stiffness = stiffness;
}

static
inline
float
b2JointDistanceGetStiffness(
    const struct b2JointDistance* p)
{
    return p->m_stiffness;
}

/// Set/get linear damping in N*s/m
static
inline
void
b2JointDistanceSetDamping(
    struct b2JointDistance* p,
    float damping) 
{
    p->m_damping = damping;
}

static
inline
float
b2JointDistanceGetDamping(
    const struct b2JointDistance* p) 
{
    return p->m_damping;
}

#include "b2/mmB2Suffix.h"

#endif//__mmB2JointDistance_h__
