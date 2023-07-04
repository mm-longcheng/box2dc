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

#ifndef __mmB2JointMotor_h__
#define __mmB2JointMotor_h__

#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Joint.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

B2_API extern const struct b2MetaAllocator b2MetaAllocatorJointMotor;
B2_API extern const struct b2JointMeta b2JointMotorMeta;

/// Motor joint definition.
struct b2JointMotorDef
{
    b2JointDefSuper;

    /// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
    b2Vec2 linearOffset;

    /// The bodyB angle minus bodyA angle in radians.
    float angularOffset;

    /// The maximum motor force in N.
    float maxForce;

    /// The maximum motor torque in N-m.
    float maxTorque;

    /// Position correction factor in the range [0,1].
    float correctionFactor;
};

B2_API
void
b2JointMotorDefReset(
    struct b2JointMotorDef* p);

/// Initialize the bodies and offsets using the current transforms.
B2_API
void
b2JointMotorDefInitialize(
    struct b2JointMotorDef* p,
    struct b2Body* bodyA, 
    struct b2Body* bodyB);

/// A motor joint is used to control the relative motion
/// between two bodies. A typical usage is to control the movement
/// of a dynamic body with respect to the ground.
struct b2JointMotor
{
    b2JointSuper;

    // Solver shared
    b2Vec2 m_linearOffset;
    float m_angularOffset;
    b2Vec2 m_linearImpulse;
    float m_angularImpulse;
    float m_maxForce;
    float m_maxTorque;
    float m_correctionFactor;

    // Solver temp
    int32 m_indexA;
    int32 m_indexB;
    b2Vec2 m_rA;
    b2Vec2 m_rB;
    b2Vec2 m_localCenterA;
    b2Vec2 m_localCenterB;
    b2Vec2 m_linearError;
    float m_angularError;
    float m_invMassA;
    float m_invMassB;
    float m_invIA;
    float m_invIB;
    b2Mat22 m_linearMass;
    float m_angularMass;
};

B2_API
void
b2JointMotorPrepare(
    struct b2JointMotor* p,
    const struct b2JointMotorDef* def);

B2_API
void
b2JointMotorDiscard(
    struct b2JointMotor* p);

B2_API
void
b2JointMotorGetAnchorA(
    const struct b2JointMotor* p, 
    b2Vec2 anchor);

B2_API
void
b2JointMotorGetAnchorB(
    const struct b2JointMotor* p, 
    b2Vec2 anchor);

B2_API
void
b2JointMotorGetReactionForce(
    const struct b2JointMotor* p, 
    float inv_dt, 
    b2Vec2 force);

B2_API
float
b2JointMotorGetReactionTorque(
    const struct b2JointMotor* p, 
    float inv_dt);

B2_API
void
b2JointMotorDump(
    const struct b2JointMotor* p);

B2_API
void
b2JointMotorShiftOrigin(
    struct b2JointMotor* p, 
    const b2Vec2 newOrigin);

B2_API
void
b2JointMotorDraw(
    const struct b2JointMotor* p, 
    struct b2Draw* draw);

B2_API
void
b2JointMotorInitVelocityConstraints(
    struct b2JointMotor* p, 
    const struct b2SolverData* data);

B2_API
void
b2JointMotorSolveVelocityConstraints(
    struct b2JointMotor* p, 
    const struct b2SolverData* data);

B2_API
int
b2JointMotorSolvePositionConstraints(
    struct b2JointMotor* p, 
    const struct b2SolverData* data);

/// Set/get the target linear offset, in frame A, in meters.
B2_API
void
b2JointMotorSetLinearOffset(
    struct b2JointMotor* p,
    const b2Vec2 linearOffset);

static
inline
b2Vec2ConstRef
b2JointMotorGetLinearOffset(
    const struct b2JointMotor* p)
{
    return p->m_linearOffset;
}

/// Set/get the target angular offset, in radians.
B2_API
void
b2JointMotorSetAngularOffset(
    struct b2JointMotor* p,
    float angularOffset);

static
inline
float 
b2JointMotorGetAngularOffset(
    const struct b2JointMotor* p)
{
    return p->m_angularOffset;
}

/// Set the maximum friction force in N.
B2_API
void
b2JointMotorSetMaxForce(
    struct b2JointMotor* p,
    float force);

/// Get the maximum friction force in N.
static
inline
float 
b2JointMotorGetMaxForce(
    const struct b2JointMotor* p)
{
    return p->m_maxForce;
}

/// Set the maximum friction torque in N*m.
B2_API
void
b2JointMotorSetMaxTorque(
    struct b2JointMotor* p,
    float torque);

/// Get the maximum friction torque in N*m.
static
inline
float 
b2JointMotorGetMaxTorque(
    const struct b2JointMotor* p)
{
    return p->m_maxTorque;
}

/// Set the position correction factor in the range [0,1].
B2_API
void
b2JointMotorSetCorrectionFactor(
    struct b2JointMotor* p,
    float factor);

/// Get the position correction factor in the range [0,1].
static
inline
float 
b2JointMotorGetCorrectionFactor(
    const struct b2JointMotor* p)
{
    return p->m_correctionFactor;
}

#include "b2/mmB2Suffix.h"

#endif//__mmB2JointMotor_h__
