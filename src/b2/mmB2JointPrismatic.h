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

#ifndef __mmB2JointPrismatic_h__
#define __mmB2JointPrismatic_h__

#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Joint.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

B2_API extern const struct b2MetaAllocator b2MetaAllocatorJointPrismatic;
B2_API extern const struct b2JointMeta b2JointPrismaticMeta;

/// Prismatic joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
struct b2JointPrismaticDef
{
    b2JointDefSuper;

    /// The local anchor point relative to bodyA's origin.
    b2Vec2 localAnchorA;

    /// The local anchor point relative to bodyB's origin.
    b2Vec2 localAnchorB;

    /// The local translation unit axis in bodyA.
    b2Vec2 localAxisA;

    /// The constrained angle between the bodies: bodyB_angle - bodyA_angle.
    float referenceAngle;

    /// Enable/disable the joint limit.
    int enableLimit;

    /// The lower translation limit, usually in meters.
    float lowerTranslation;

    /// The upper translation limit, usually in meters.
    float upperTranslation;

    /// Enable/disable the joint motor.
    int enableMotor;

    /// The maximum motor torque, usually in N-m.
    float maxMotorForce;

    /// The desired motor speed in radians per second.
    float motorSpeed;
};

B2_API
void
b2JointPrismaticDefReset(
    struct b2JointPrismaticDef* p);

/// Initialize the bodies, anchors, axis, and reference angle using the world
/// anchor and unit world axis.
B2_API
void
b2JointPrismaticDefInitialize(
    struct b2JointPrismaticDef* p,
    struct b2Body* bodyA, 
    struct b2Body* bodyB, 
    const b2Vec2 anchor,
    const b2Vec2 axis);

/// A prismatic joint. This joint provides one degree of freedom: translation
/// along an axis fixed in bodyA. Relative rotation is prevented. You can
/// use a joint limit to restrict the range of motion and a joint motor to
/// drive the motion or to model joint friction.
struct b2JointPrismatic
{
    b2JointSuper;

    b2Vec2 m_localAnchorA;
    b2Vec2 m_localAnchorB;
    b2Vec2 m_localXAxisA;
    b2Vec2 m_localYAxisA;
    float m_referenceAngle;
    b2Vec2 m_impulse;
    float m_motorImpulse;
    float m_lowerImpulse;
    float m_upperImpulse;
    float m_lowerTranslation;
    float m_upperTranslation;
    float m_maxMotorForce;
    float m_motorSpeed;
    int m_enableLimit;
    int m_enableMotor;

    // Solver temp
    int32 m_indexA;
    int32 m_indexB;
    b2Vec2 m_localCenterA;
    b2Vec2 m_localCenterB;
    float m_invMassA;
    float m_invMassB;
    float m_invIA;
    float m_invIB;
    b2Vec2 m_axis, m_perp;
    float m_s1, m_s2;
    float m_a1, m_a2;
    b2Mat22 m_K;
    float m_translation;
    float m_axialMass;
};

B2_API
void
b2JointPrismaticPrepare(
    struct b2JointPrismatic* p,
    const struct b2JointPrismaticDef* def);

B2_API
void
b2JointPrismaticDiscard(
    struct b2JointPrismatic* p);

B2_API
void
b2JointPrismaticGetAnchorA(
    const struct b2JointPrismatic* p, 
    b2Vec2 anchor);

B2_API
void
b2JointPrismaticGetAnchorB(
    const struct b2JointPrismatic* p, 
    b2Vec2 anchor);

B2_API
void
b2JointPrismaticGetReactionForce(
    const struct b2JointPrismatic* p, 
    float inv_dt, 
    b2Vec2 force);

B2_API
float
b2JointPrismaticGetReactionTorque(
    const struct b2JointPrismatic* p, 
    float inv_dt);

B2_API
void
b2JointPrismaticDump(
    const struct b2JointPrismatic* p);

B2_API
void
b2JointPrismaticShiftOrigin(
    struct b2JointPrismatic* p, 
    const b2Vec2 newOrigin);

B2_API
void
b2JointPrismaticDraw(
    const struct b2JointPrismatic* p, 
    struct b2Draw* draw);

B2_API
void
b2JointPrismaticInitVelocityConstraints(
    struct b2JointPrismatic* p, 
    const struct b2SolverData* data);

B2_API
void
b2JointPrismaticSolveVelocityConstraints(
    struct b2JointPrismatic* p, 
    const struct b2SolverData* data);

B2_API
int
b2JointPrismaticSolvePositionConstraints(
    struct b2JointPrismatic* p, 
    const struct b2SolverData* data);


/// The local anchor point relative to bodyA's origin.
static
inline
b2Vec2ConstRef
b2JointPrismaticGetLocalAnchorA(
    const struct b2JointPrismatic* p)
{
    return p->m_localAnchorA;
}

/// The local anchor point relative to bodyB's origin.
static
inline
b2Vec2ConstRef
b2JointPrismaticGetLocalAnchorB(
    const struct b2JointPrismatic* p)
{
    return p->m_localAnchorB;
}

/// The local joint axis relative to bodyA.
static
inline
b2Vec2ConstRef
b2JointPrismaticGetLocalAxisA(
    const struct b2JointPrismatic* p)
{
    return p->m_localXAxisA;
}

/// Get the reference angle.
static
inline
float
b2JointPrismaticGetReferenceAngle(
    const struct b2JointPrismatic* p)
{
    return p->m_referenceAngle;
}

/// Get the current joint translation, usually in meters.
B2_API
float
b2JointPrismaticGetJointTranslation(
    const struct b2JointPrismatic* p);

/// Get the current joint translation speed, usually in meters per second.
B2_API
float
b2JointPrismaticGetJointSpeed(
    const struct b2JointPrismatic* p);

/// Is the joint limit enabled?
static
inline
int 
b2JointPrismaticIsLimitEnabled(
    const struct b2JointPrismatic* p)
{
    return p->m_enableLimit;
}

/// Enable/disable the joint limit.
B2_API
void
b2JointPrismaticEnableLimit(
    struct b2JointPrismatic* p,
    int flag);

/// Get the lower joint limit, usually in meters.
static
inline
float 
b2JointPrismaticGetLowerLimit(
    const struct b2JointPrismatic* p)
{
    return p->m_lowerTranslation;
}

/// Get the upper joint limit, usually in meters.
static
inline
float 
b2JointPrismaticGetUpperLimit(
    const struct b2JointPrismatic* p)
{
    return p->m_upperTranslation;
}

/// Set the joint limits, usually in meters.
B2_API
void
b2JointPrismaticSetLimits(
    struct b2JointPrismatic* p,
    float lower, 
    float upper);

/// Is the joint motor enabled?
static
inline
int 
b2JointPrismaticIsMotorEnabled(
    const struct b2JointPrismatic* p)
{
    return p->m_enableMotor;
}

/// Enable/disable the joint motor.
B2_API
void
b2JointPrismaticEnableMotor(
    struct b2JointPrismatic* p,
    int flag);

/// Set the motor speed, usually in meters per second.
B2_API
void
b2JointPrismaticSetMotorSpeed(
    struct b2JointPrismatic* p,
    float speed);

/// Get the motor speed, usually in meters per second.
static
inline
float 
b2JointPrismaticGetMotorSpeed(
    const struct b2JointPrismatic* p)
{
    return p->m_motorSpeed;
}

/// Set the maximum motor force, usually in N.
B2_API
void
b2JointPrismaticSetMaxMotorForce(
    struct b2JointPrismatic* p,
    float force);

static
inline
float
b2JointPrismaticGetMaxMotorForce(
    const struct b2JointPrismatic* p)
{
    return p->m_maxMotorForce;
}

/// Get the current motor force given the inverse time step, usually in N.
static
inline
float 
b2JointPrismaticGetMotorForce(
    struct b2JointPrismatic* p,
    float inv_dt)
{
    return inv_dt * p->m_motorImpulse;
}

#include "b2/mmB2Suffix.h"

#endif//__mmB2JointPrismatic_h__
