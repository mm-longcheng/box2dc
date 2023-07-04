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

#ifndef __mmB2JointPulley_h__
#define __mmB2JointPulley_h__

#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Joint.h"
#include "b2/mmB2MetaAllocator.h"

#include "b2/mmB2Prefix.h"

B2_API extern const struct b2MetaAllocator b2MetaAllocatorJointPulley;
B2_API extern const struct b2JointMeta b2JointPulleyMeta;

/// b2_minPulleyLength = 2.0f
B2_API extern const float b2_minPulleyLength;

/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, and a pulley ratio.
struct b2JointPulleyDef
{
    b2JointDefSuper;

    /// The first ground anchor in world coordinates. This point never moves.
    b2Vec2 groundAnchorA;

    /// The second ground anchor in world coordinates. This point never moves.
    b2Vec2 groundAnchorB;

    /// The local anchor point relative to bodyA's origin.
    b2Vec2 localAnchorA;

    /// The local anchor point relative to bodyB's origin.
    b2Vec2 localAnchorB;

    /// The a reference length for the segment attached to bodyA.
    float lengthA;

    /// The a reference length for the segment attached to bodyB.
    float lengthB;

    /// The pulley ratio, used to simulate a block-and-tackle.
    float ratio;
};

B2_API
void
b2JointPulleyDefReset(
    struct b2JointPulleyDef* p);

/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
B2_API
void
b2JointPulleyDefInitialize(
    struct b2JointPulleyDef* p,
    struct b2Body* bodyA, 
    struct b2Body* bodyB,
    const b2Vec2 groundAnchorA, 
    const b2Vec2 groundAnchorB,
    const b2Vec2 anchorA, 
    const b2Vec2 anchorB,
    float ratio);

/// The pulley joint is connected to two bodies and two fixed ground points.
/// The pulley supports a ratio such that:
/// length1 + ratio * length2 <= constant
/// Yes, the force transmitted is scaled by the ratio.
/// Warning: the pulley joint can get a bit squirrelly by itself. They often
/// work better when combined with prismatic joints. You should also cover the
/// the anchor points with static shapes to prevent one side from going to
/// zero length.
struct b2JointPulley
{
    b2JointSuper;

    b2Vec2 m_groundAnchorA;
    b2Vec2 m_groundAnchorB;
    float m_lengthA;
    float m_lengthB;

    // Solver shared
    b2Vec2 m_localAnchorA;
    b2Vec2 m_localAnchorB;
    float m_constant;
    float m_ratio;
    float m_impulse;

    // Solver temp
    int32 m_indexA;
    int32 m_indexB;
    b2Vec2 m_uA;
    b2Vec2 m_uB;
    b2Vec2 m_rA;
    b2Vec2 m_rB;
    b2Vec2 m_localCenterA;
    b2Vec2 m_localCenterB;
    float m_invMassA;
    float m_invMassB;
    float m_invIA;
    float m_invIB;
    float m_mass;
};

B2_API
void
b2JointPulleyPrepare(
    struct b2JointPulley* p,
    const struct b2JointPulleyDef* def);

B2_API
void
b2JointPulleyDiscard(
    struct b2JointPulley* p);

B2_API
void
b2JointPulleyGetAnchorA(
    const struct b2JointPulley* p, 
    b2Vec2 anchor);

B2_API
void
b2JointPulleyGetAnchorB(
    const struct b2JointPulley* p, 
    b2Vec2 anchor);

B2_API
void
b2JointPulleyGetReactionForce(
    const struct b2JointPulley* p, 
    float inv_dt, 
    b2Vec2 force);

B2_API
float
b2JointPulleyGetReactionTorque(
    const struct b2JointPulley* p, 
    float inv_dt);

B2_API
void
b2JointPulleyDump(
    const struct b2JointPulley* p);

B2_API
void
b2JointPulleyShiftOrigin(
    struct b2JointPulley* p, 
    const b2Vec2 newOrigin);

B2_API
void
b2JointPulleyDraw(
    const struct b2JointPulley* p, 
    struct b2Draw* draw);

B2_API
void
b2JointPulleyInitVelocityConstraints(
    struct b2JointPulley* p, 
    const struct b2SolverData* data);

B2_API
void
b2JointPulleySolveVelocityConstraints(
    struct b2JointPulley* p, 
    const struct b2SolverData* data);

B2_API
int
b2JointPulleySolvePositionConstraints(
    struct b2JointPulley* p, 
    const struct b2SolverData* data);


/// Get the first ground anchor.
static
inline
b2Vec2ConstRef
b2JointPulleyGetGroundAnchorA(
    const struct b2JointPulley* p)
{
    return p->m_groundAnchorA;
}

/// Get the second ground anchor.
static
inline
b2Vec2ConstRef
b2JointPulleyGetGroundAnchorB(
    const struct b2JointPulley* p)
{
    return p->m_groundAnchorB;
}

/// Get the current length of the segment attached to bodyA.
static
inline
float
b2JointPulleyGetLengthA(
    const struct b2JointPulley* p)
{
    return p->m_lengthA;
}

/// Get the current length of the segment attached to bodyB.
static
inline
float
b2JointPulleyGetLengthB(
    const struct b2JointPulley* p)
{
    return p->m_lengthB;
}

/// Get the pulley ratio.
static
inline
float
b2JointPulleyGetRatio(
    const struct b2JointPulley* p)
{
    return p->m_ratio;
}

/// Get the current length of the segment attached to bodyA.
B2_API
float
b2JointPulleyGetCurrentLengthA(
    const struct b2JointPulley* p);

/// Get the current length of the segment attached to bodyB.
B2_API
float
b2JointPulleyGetCurrentLengthB(
    const struct b2JointPulley* p);

#include "b2/mmB2Suffix.h"

#endif//__mmB2JointPulley_h__
