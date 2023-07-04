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

#ifndef __mmB2JointWheel_h__
#define __mmB2JointWheel_h__

#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Joint.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

B2_API extern const struct b2MetaAllocator b2MetaAllocatorJointWheel;
B2_API extern const struct b2JointMeta b2JointWheelMeta;

/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
struct b2JointWheelDef
{
    b2JointDefSuper;

    /// The local anchor point relative to bodyA's origin.
    b2Vec2 localAnchorA;

    /// The local anchor point relative to bodyB's origin.
    b2Vec2 localAnchorB;

    /// The local translation axis in bodyA.
    b2Vec2 localAxisA;

    /// Enable/disable the joint limit.
    int enableLimit;

    /// The lower translation limit, usually in meters.
    float lowerTranslation;

    /// The upper translation limit, usually in meters.
    float upperTranslation;

    /// Enable/disable the joint motor.
    int enableMotor;

    /// The maximum motor torque, usually in N-m.
    float maxMotorTorque;

    /// The desired motor speed in radians per second.
    float motorSpeed;

    /// Suspension stiffness. Typically in units N/m.
    float stiffness;

    /// Suspension damping. Typically in units of N*s/m.
    float damping;
};

B2_API
void
b2JointWheelDefReset(
    struct b2JointWheelDef* p);

/// Initialize the bodies, anchors, axis, and reference angle using the world
/// anchor and world axis.
B2_API
void
b2JointWheelDefInitialize(
    struct b2JointWheelDef* p,
    struct b2Body* bodyA, 
    struct b2Body* bodyB, 
    const b2Vec2 anchor, 
    const b2Vec2 axis);

/// A wheel joint. This joint provides two degrees of freedom: translation
/// along an axis fixed in bodyA and rotation in the plane. In other words, it is a point to
/// line constraint with a rotational motor and a linear spring/damper. The spring/damper is
/// initialized upon creation. This joint is designed for vehicle suspensions.
struct b2JointWheel
{
    b2JointSuper;

    b2Vec2 m_localAnchorA;
    b2Vec2 m_localAnchorB;
    b2Vec2 m_localXAxisA;
    b2Vec2 m_localYAxisA;

    float m_impulse;
    float m_motorImpulse;
    float m_springImpulse;

    float m_lowerImpulse;
    float m_upperImpulse;
    float m_translation;
    float m_lowerTranslation;
    float m_upperTranslation;

    float m_maxMotorTorque;
    float m_motorSpeed;

    int m_enableLimit;
    int m_enableMotor;

    float m_stiffness;
    float m_damping;

    // Solver temp
    int32 m_indexA;
    int32 m_indexB;
    b2Vec2 m_localCenterA;
    b2Vec2 m_localCenterB;
    float m_invMassA;
    float m_invMassB;
    float m_invIA;
    float m_invIB;

    b2Vec2 m_ax, m_ay;
    float m_sAx, m_sBx;
    float m_sAy, m_sBy;

    float m_mass;
    float m_motorMass;
    float m_axialMass;
    float m_springMass;

    float m_bias;
    float m_gamma;
};

B2_API
void
b2JointWheelPrepare(
    struct b2JointWheel* p,
    const struct b2JointWheelDef* def);

B2_API
void
b2JointWheelDiscard(
    struct b2JointWheel* p);

B2_API
void
b2JointWheelGetAnchorA(
    const struct b2JointWheel* p, 
    b2Vec2 anchor);

B2_API
void
b2JointWheelGetAnchorB(
    const struct b2JointWheel* p, 
    b2Vec2 anchor);

B2_API
void
b2JointWheelGetReactionForce(
    const struct b2JointWheel* p, 
    float inv_dt, 
    b2Vec2 force);

B2_API
float
b2JointWheelGetReactionTorque(
    const struct b2JointWheel* p, 
    float inv_dt);

B2_API
void
b2JointWheelDump(
    const struct b2JointWheel* p);

B2_API
void
b2JointWheelShiftOrigin(
    struct b2JointWheel* p, 
    const b2Vec2 newOrigin);

B2_API
void
b2JointWheelDraw(
    const struct b2JointWheel* p, 
    struct b2Draw* draw);

B2_API
void
b2JointWheelInitVelocityConstraints(
    struct b2JointWheel* p, 
    const struct b2SolverData* data);

B2_API
void
b2JointWheelSolveVelocityConstraints(
    struct b2JointWheel* p, 
    const struct b2SolverData* data);

B2_API
int
b2JointWheelSolvePositionConstraints(
    struct b2JointWheel* p, 
    const struct b2SolverData* data);


/// The local anchor point relative to bodyA's origin.
static
inline
b2Vec2ConstRef
b2JointWheelGetLocalAnchorA(
    const struct b2JointWheel* p)
{
    return p->m_localAnchorA;
}

/// The local anchor point relative to bodyB's origin.
static
inline
b2Vec2ConstRef
b2JointWheelGetLocalAnchorB(
    const struct b2JointWheel* p)
{
    return p->m_localAnchorB;
}

/// The local joint axis relative to bodyA.
static
inline
b2Vec2ConstRef
b2JointWheelGetLocalAxisA(
    const struct b2JointWheel* p)
{
    return p->m_localXAxisA;
}

/// Get the current joint translation, usually in meters.
B2_API
float
b2JointWheelGetJointTranslation(
    const struct b2JointWheel* p);

/// Get the current joint linear speed, usually in meters per second.
B2_API
float
b2JointWheelGetJointLinearSpeed(
    const struct b2JointWheel* p);

/// Get the current joint angle in radians.
B2_API
float
b2JointWheelGetJointAngle(
    const struct b2JointWheel* p);

/// Get the current joint angular speed in radians per second.
B2_API
float
b2JointWheelGetJointAngularSpeed(
    const struct b2JointWheel* p);

/// Is the joint limit enabled?
static
inline
int 
b2JointWheelIsLimitEnabled(
    const struct b2JointWheel* p)
{
    return p->m_enableLimit;
}

/// Enable/disable the joint translation limit.
B2_API
void
b2JointWheelEnableLimit(
    struct b2JointWheel* p,
    int flag);

/// Get the lower joint translation limit, usually in meters.
static
inline
float 
b2JointWheelGetLowerLimit(
    const struct b2JointWheel* p)
{
    return p->m_lowerTranslation;
}

/// Get the upper joint translation limit, usually in meters.
static
inline
float
b2JointWheelGetUpperLimit(
    const struct b2JointWheel* p)
{
    return p->m_upperTranslation;
}

/// Set the joint translation limits, usually in meters.
B2_API
void
b2JointWheelSetLimits(
    struct b2JointWheel* p,
    float lower, float upper);

/// Is the joint motor enabled?
static
inline
int
b2JointWheelIsMotorEnabled(
    const struct b2JointWheel* p)
{
    return p->m_enableMotor;
}

/// Enable/disable the joint motor.
B2_API
void
b2JointWheelEnableMotor(
    struct b2JointWheel* p,
    int flag);

/// Set the motor speed, usually in radians per second.
B2_API
void
b2JointWheelSetMotorSpeed(
    struct b2JointWheel* p,
    float speed);

/// Get the motor speed, usually in radians per second.
static
inline
float
b2JointWheelGetMotorSpeed(
    const struct b2JointWheel* p)
{
    return p->m_motorSpeed;
}

/// Set/Get the maximum motor force, usually in N-m.
B2_API
void
b2JointWheelSetMaxMotorTorque(
    struct b2JointWheel* p,
    float torque);

static
inline
float
b2JointWheelGetMaxMotorTorque(
    const struct b2JointWheel* p)
{
    return p->m_maxMotorTorque;
}

/// Get the current motor torque given the inverse time step, usually in N-m.
static
inline
float
b2JointWheelGetMotorTorque(
    const struct b2JointWheel* p,
    float inv_dt)
{
    return inv_dt * p->m_motorImpulse;
}

/// Access spring stiffness
static
inline
void
b2JointWheelSetStiffness(
    struct b2JointWheel* p,
    float stiffness)
{
    p->m_stiffness = stiffness;
}

static
inline
float
b2JointWheelGetStiffness(
    const struct b2JointWheel* p)
{
    return p->m_stiffness;
}

/// Access damping
static
inline
void
b2JointWheelSetDamping(
    struct b2JointWheel* p,
    float damping)
{
    p->m_damping = damping;
}

static
inline
float
b2JointWheelGetDamping(
    const struct b2JointWheel* p)
{
    return p->m_damping;
}

#include "b2/mmB2Suffix.h"

#endif//__mmB2JointWheel_h__
