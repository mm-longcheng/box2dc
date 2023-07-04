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

#ifndef __mmB2JointMouse_h__
#define __mmB2JointMouse_h__

#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Joint.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

B2_API extern const struct b2MetaAllocator b2MetaAllocatorJointMouse;
B2_API extern const struct b2JointMeta b2JointMouseMeta;

/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
struct b2JointMouseDef
{
    b2JointDefSuper;

    /// The initial world target point. This is assumed
    /// to coincide with the body anchor initially.
    b2Vec2 target;

    /// The maximum constraint force that can be exerted
    /// to move the candidate body. Usually you will express
    /// as some multiple of the weight (multiplier * mass * gravity).
    float maxForce;

    /// The linear stiffness in N/m
    float stiffness;

    /// The linear damping in N*s/m
    float damping;
};

B2_API
void
b2JointMouseDefReset(
    struct b2JointMouseDef* p);

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch and without
/// applying huge forces.
/// NOTE: this joint is not documented in the manual because it was
/// developed to be used in the testbed. If you want to learn how to
/// use the mouse joint, look at the testbed.
struct b2JointMouse
{
    b2JointSuper;

    b2Vec2 m_localAnchorB;
    b2Vec2 m_targetA;
    float m_stiffness;
    float m_damping;
    float m_beta;

    // Solver shared
    b2Vec2 m_impulse;
    float m_maxForce;
    float m_gamma;

    // Solver temp
    int32 m_indexA;
    int32 m_indexB;
    b2Vec2 m_rB;
    b2Vec2 m_localCenterB;
    float m_invMassB;
    float m_invIB;
    b2Mat22 m_mass;
    b2Vec2 m_C;
};

B2_API
void
b2JointMousePrepare(
    struct b2JointMouse* p,
    const struct b2JointMouseDef* def);

B2_API
void
b2JointMouseDiscard(
    struct b2JointMouse* p);

B2_API
void
b2JointMouseGetAnchorA(
    const struct b2JointMouse* p, 
    b2Vec2 anchor);

B2_API
void
b2JointMouseGetAnchorB(
    const struct b2JointMouse* p, 
    b2Vec2 anchor);

B2_API
void
b2JointMouseGetReactionForce(
    const struct b2JointMouse* p, 
    float inv_dt, 
    b2Vec2 force);

B2_API
float
b2JointMouseGetReactionTorque(
    const struct b2JointMouse* p, 
    float inv_dt);

B2_API
void
b2JointMouseDump(
    const struct b2JointMouse* p);

B2_API
void
b2JointMouseShiftOrigin(
    struct b2JointMouse* p, 
    const b2Vec2 newOrigin);

B2_API
void
b2JointMouseDraw(
    const struct b2JointMouse* p, 
    struct b2Draw* draw);

B2_API
void
b2JointMouseInitVelocityConstraints(
    struct b2JointMouse* p, 
    const struct b2SolverData* data);

B2_API
void
b2JointMouseSolveVelocityConstraints(
    struct b2JointMouse* p, 
    const struct b2SolverData* data);

B2_API
int
b2JointMouseSolvePositionConstraints(
    struct b2JointMouse* p, 
    const struct b2SolverData* data);


/// Use this to update the target point.
B2_API
void
b2JointMouseSetTarget(
    struct b2JointMouse* p,
    const b2Vec2 target);

static
inline
b2Vec2ConstRef
b2JointMouseGetTarget(
    const struct b2JointMouse* p)
{
    return p->m_targetA;
}

/// Set/get the maximum force in Newtons.
static
inline
void
b2JointMouseSetMaxForce(
    struct b2JointMouse* p,
    float force)
{
    p->m_maxForce = force;
}

static
inline
float
b2JointMouseGetMaxForce(
    const struct b2JointMouse* p)
{
    return p->m_maxForce;
}

/// Set/get the linear stiffness in N/m
static
inline
void
b2JointMouseSetStiffness(
    struct b2JointMouse* p,
    float stiffness)
{
    p->m_stiffness = stiffness;
}

static
inline
float
b2JointMouseGetStiffness(
    const struct b2JointMouse* p)
{
    return p->m_stiffness;
}

/// Set/get linear damping in N*s/m
static
inline
void
b2JointMouseSetDamping(
    struct b2JointMouse* p,
    float damping) 
{
    p->m_damping = damping;
}

static
inline
float
b2JointMouseGetDamping(
    const struct b2JointMouse* p)
{
    return p->m_damping;
}

#include "b2/mmB2Suffix.h"

#endif//__mmB2JointMouse_h__
