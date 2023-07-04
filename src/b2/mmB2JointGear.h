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

#ifndef __mmB2JointGear_h__
#define __mmB2JointGear_h__

#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Joint.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

B2_API extern const struct b2MetaAllocator b2MetaAllocatorJointGear;
B2_API extern const struct b2JointMeta b2JointGearMeta;

/// Gear joint definition. This definition requires two existing
/// revolute or prismatic joints (any combination will work).
/// @warning bodyB on the input joints must both be dynamic
struct b2JointGearDef
{
    b2JointDefSuper;

    /// The first revolute/prismatic joint attached to the gear joint.
    struct b2Joint* joint1;

    /// The second revolute/prismatic joint attached to the gear joint.
    struct b2Joint* joint2;

    /// The gear ratio.
    /// @see b2GearJoint for explanation.
    float ratio;
};

B2_API
void
b2JointGearDefReset(
    struct b2JointGearDef* p);

/// A gear joint is used to connect two joints together. Either joint
/// can be a revolute or prismatic joint. You specify a gear ratio
/// to bind the motions together:
/// coordinate1 + ratio * coordinate2 = constant
/// The ratio can be negative or positive. If one joint is a revolute joint
/// and the other joint is a prismatic joint, then the ratio will have units
/// of length or units of 1/length.
/// @warning You have to manually destroy the gear joint if joint1 or joint2
/// is destroyed.
struct b2JointGear
{
    b2JointSuper;

    struct b2Joint* m_joint1;
    struct b2Joint* m_joint2;

    enum b2JointType m_typeA;
    enum b2JointType m_typeB;

    // Body A is connected to body C
    // Body B is connected to body D
    struct b2Body* m_bodyC;
    struct b2Body* m_bodyD;

    // Solver shared
    b2Vec2 m_localAnchorA;
    b2Vec2 m_localAnchorB;
    b2Vec2 m_localAnchorC;
    b2Vec2 m_localAnchorD;

    b2Vec2 m_localAxisC;
    b2Vec2 m_localAxisD;

    float m_referenceAngleA;
    float m_referenceAngleB;

    float m_constant;
    float m_ratio;
    float m_tolerance;

    float m_impulse;

    // Solver temp
    int32 m_indexA, m_indexB, m_indexC, m_indexD;
    b2Vec2 m_lcA, m_lcB, m_lcC, m_lcD;
    float m_mA, m_mB, m_mC, m_mD;
    float m_iA, m_iB, m_iC, m_iD;
    b2Vec2 m_JvAC, m_JvBD;
    float m_JwA, m_JwB, m_JwC, m_JwD;
    float m_mass;
};

B2_API
void
b2JointGearPrepare(
    struct b2JointGear* p,
    const struct b2JointGearDef* def);

B2_API
void
b2JointGearDiscard(
    struct b2JointGear* p);

B2_API
void
b2JointGearGetAnchorA(
    const struct b2JointGear* p, 
    b2Vec2 anchor);

B2_API
void
b2JointGearGetAnchorB(
    const struct b2JointGear* p, 
    b2Vec2 anchor);

B2_API
void
b2JointGearGetReactionForce(
    const struct b2JointGear* p, 
    float inv_dt, 
    b2Vec2 force);

B2_API
float
b2JointGearGetReactionTorque(
    const struct b2JointGear* p, 
    float inv_dt);

B2_API
void
b2JointGearDump(
    const struct b2JointGear* p);

B2_API
void
b2JointGearShiftOrigin(
    struct b2JointGear* p, 
    const b2Vec2 newOrigin);

B2_API
void
b2JointGearDraw(
    const struct b2JointGear* p, 
    struct b2Draw* draw);

B2_API
void
b2JointGearInitVelocityConstraints(
    struct b2JointGear* p, 
    const struct b2SolverData* data);

B2_API
void
b2JointGearSolveVelocityConstraints(
    struct b2JointGear* p, 
    const struct b2SolverData* data);

B2_API
int
b2JointGearSolvePositionConstraints(
    struct b2JointGear* p, 
    const struct b2SolverData* data);

/// Get the first joint.
static
inline
struct b2Joint*
b2JointGearGetJoint1Ref(
    struct b2JointGear* p)
{
    return p->m_joint1;
}

/// Get the second joint.
static
inline
struct b2Joint*
b2JointGearGetJoint2Ref(
    struct b2JointGear* p)
{
    return p->m_joint2;
}

/// Set/Get the gear ratio.
B2_API
void
b2JointGearSetRatio(
    struct b2JointGear* p,
    float ratio);

static
inline
float
b2JointGearGetRatio(
    const struct b2JointGear* p)
{
    return p->m_ratio;
}

#include "b2/mmB2Suffix.h"

#endif//__mmB2JointGear_h__
