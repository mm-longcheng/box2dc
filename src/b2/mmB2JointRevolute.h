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

#ifndef __mmB2JointRevolute_h__
#define __mmB2JointRevolute_h__

#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Joint.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

B2_API extern const struct b2MetaAllocator b2MetaAllocatorJointRevolute;
B2_API extern const struct b2JointMeta b2JointRevoluteMeta;

/// Revolute joint definition. This requires defining an anchor point where the
/// bodies are joined. The definition uses local anchor points so that the
/// initial configuration can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This helps when saving
/// and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
struct b2JointRevoluteDef
{
    b2JointDefSuper;

    /// The local anchor point relative to bodyA's origin.
    b2Vec2 localAnchorA;

    /// The local anchor point relative to bodyB's origin.
    b2Vec2 localAnchorB;

    /// The bodyB angle minus bodyA angle in the reference state (radians).
    float referenceAngle;

    /// A flag to enable joint limits.
    int enableLimit;

    /// The lower angle for the joint limit (radians).
    float lowerAngle;

    /// The upper angle for the joint limit (radians).
    float upperAngle;

    /// A flag to enable the joint motor.
    int enableMotor;

    /// The desired motor speed. Usually in radians per second.
    float motorSpeed;

    /// The maximum motor torque used to achieve the desired motor speed.
    /// Usually in N-m.
    float maxMotorTorque;
};

B2_API
void
b2JointRevoluteDefReset(
    struct b2JointRevoluteDef* p);

/// Initialize the bodies, anchors, and reference angle using a world
/// anchor point.
B2_API
void
b2JointRevoluteDefInitialize(
    struct b2JointRevoluteDef* p,
    struct b2Body* bodyA, 
    struct b2Body* bodyB, 
    const b2Vec2 anchor);

/// A revolute joint constrains two bodies to share a common point while they
/// are free to rotate about the point. The relative rotation about the shared
/// point is the joint angle. You can limit the relative rotation with
/// a joint limit that specifies a lower and upper angle. You can use a motor
/// to drive the relative rotation about the shared point. A maximum motor torque
/// is provided so that infinite forces are not generated.
struct b2JointRevolute
{
    b2JointSuper;

    // Solver shared
    b2Vec2 m_localAnchorA;
    b2Vec2 m_localAnchorB;
    b2Vec2 m_impulse;
    float m_motorImpulse;
    float m_lowerImpulse;
    float m_upperImpulse;
    int m_enableMotor;
    float m_maxMotorTorque;
    float m_motorSpeed;
    int m_enableLimit;
    float m_referenceAngle;
    float m_lowerAngle;
    float m_upperAngle;

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
    b2Mat22 m_K;
    float m_angle;
    float m_axialMass;
};

B2_API
void
b2JointRevolutePrepare(
    struct b2JointRevolute* p,
    const struct b2JointRevoluteDef* def);

B2_API
void
b2JointRevoluteDiscard(
    struct b2JointRevolute* p);

B2_API
void
b2JointRevoluteGetAnchorA(
    const struct b2JointRevolute* p, 
    b2Vec2 anchor);

B2_API
void
b2JointRevoluteGetAnchorB(
    const struct b2JointRevolute* p, 
    b2Vec2 anchor);

/// Get the reaction force given the inverse time step.
/// Unit is N.
B2_API
void
b2JointRevoluteGetReactionForce(
    const struct b2JointRevolute* p, 
    float inv_dt, 
    b2Vec2 force);

/// Get the reaction torque given the inverse time step.
/// Unit is N*m. This is always zero for a distance joint.
B2_API
float
b2JointRevoluteGetReactionTorque(
    const struct b2JointRevolute* p, 
    float inv_dt);

/// Dump joint to dmLog
B2_API
void
b2JointRevoluteDump(
    const struct b2JointRevolute* p);

B2_API
void
b2JointRevoluteShiftOrigin(
    struct b2JointRevolute* p, 
    const b2Vec2 newOrigin);

B2_API
void
b2JointRevoluteDraw(
    const struct b2JointRevolute* p, 
    struct b2Draw* draw);

B2_API
void
b2JointRevoluteInitVelocityConstraints(
    struct b2JointRevolute* p, 
    const struct b2SolverData* data);

B2_API
void
b2JointRevoluteSolveVelocityConstraints(
    struct b2JointRevolute* p, 
    const struct b2SolverData* data);

B2_API
int
b2JointRevoluteSolvePositionConstraints(
    struct b2JointRevolute* p, 
    const struct b2SolverData* data);


/// The local anchor point relative to bodyA's origin.
static
inline
b2Vec2ConstRef
b2JointRevoluteGetLocalAnchorA(
    const struct b2JointRevolute* p)
{
    return p->m_localAnchorA;
}

/// The local anchor point relative to bodyB's origin.
static
inline
b2Vec2ConstRef
b2JointRevoluteGetLocalAnchorB(
    const struct b2JointRevolute* p)
{
    return p->m_localAnchorB;
}

/// Get the reference angle.
static
inline
float
b2JointRevoluteGetReferenceAngle(
    const struct b2JointRevolute* p)
{
    return p->m_referenceAngle;
}

/// Get the current joint angle in radians.
B2_API
float
b2JointRevoluteGetJointAngle(
    const struct b2JointRevolute* p);

/// Get the current joint angle speed in radians per second.
B2_API
float
b2JointRevoluteGetJointSpeed(
    const struct b2JointRevolute* p);

/// Is the joint limit enabled?
static
inline
int
b2JointRevoluteIsLimitEnabled(
    const struct b2JointRevolute* p)
{
    return p->m_enableLimit;
}

/// Enable/disable the joint limit.
B2_API
void
b2JointRevoluteEnableLimit(
    struct b2JointRevolute* p, 
    int flag);

/// Get the lower joint limit in radians.
static
inline
float
b2JointRevoluteGetLowerLimit(
    const struct b2JointRevolute* p)
{
    return p->m_lowerAngle;
}

/// Get the upper joint limit in radians.
static
inline
float
b2JointRevoluteGetUpperLimit(
    const struct b2JointRevolute* p)
{
    return p->m_upperAngle;
}

/// Set the joint limits in radians.
B2_API
void
b2JointRevoluteSetLimits(
    struct b2JointRevolute* p,
    float lower, 
    float upper);

/// Is the joint motor enabled?
static
inline
int
b2JointRevoluteIsMotorEnabled(
    const struct b2JointRevolute* p)
{
    return p->m_enableMotor;
}

/// Enable/disable the joint motor.
B2_API
void
b2JointRevoluteEnableMotor(
    struct b2JointRevolute* p,
    int flag);

/// Set the motor speed in radians per second.
B2_API
void
b2JointRevoluteSetMotorSpeed(
    struct b2JointRevolute* p,
    float speed);

/// Get the motor speed in radians per second.
static
inline
float
b2JointRevoluteGetMotorSpeed(
    const struct b2JointRevolute* p)
{
    return p->m_motorSpeed;
}

/// Set the maximum motor torque, usually in N-m.
B2_API
void
b2JointRevoluteSetMaxMotorTorque(
    struct b2JointRevolute* p,
    float torque);

static
inline
float
b2JointRevoluteGetMaxMotorTorque(
    const struct b2JointRevolute* p)
{
    return p->m_maxMotorTorque;
}

/// Get the current motor torque given the inverse time step.
/// Unit is N*m.
static
inline
float
b2JointRevoluteGetMotorTorque(
    const struct b2JointRevolute* p,
    float inv_dt)
{
    return inv_dt * p->m_motorImpulse;
}

#include "b2/mmB2Suffix.h"

#endif//__mmB2JointRevolute_h__
