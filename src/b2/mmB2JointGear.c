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

#include "mmB2JointGear.h"
#include "mmB2Common.h"
#include "mmB2Body.h"
#include "mmB2Draw.h"
#include "mmB2TimeStep.h"
#include "mmB2JointRevolute.h"
#include "mmB2JointPrismatic.h"

#include <assert.h>
#include <string.h>

B2_API const struct b2MetaAllocator b2MetaAllocatorJointGear =
{
    "b2JointGear",
    sizeof(struct b2JointGear),
    &b2JointGearPrepare,
    &b2JointGearDiscard,
};

B2_API const struct b2JointMeta b2JointGearMeta =
{
    &b2JointGearGetAnchorA,
    &b2JointGearGetAnchorB,
    &b2JointGearGetReactionForce,
    &b2JointGearGetReactionTorque,
    &b2JointGearDump,
    &b2JointGearShiftOrigin,
    &b2JointGearDraw,
    &b2JointGearInitVelocityConstraints,
    &b2JointGearSolveVelocityConstraints,
    &b2JointGearSolvePositionConstraints,
};

// Gear Joint:
// C0 = (coordinate1 + ratio * coordinate2)_initial
// C = (coordinate1 + ratio * coordinate2) - C0 = 0
// J = [J1 ratio * J2]
// K = J * invM * JT
//   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
// Revolute:
// coordinate = rotation
// Cdot = angularVelocity
// J = [0 0 1]
// K = J * invM * JT = invI
//
// Prismatic:
// coordinate = dot(p - pg, ug)
// Cdot = dot(v + cross(w, r), ug)
// J = [ug cross(r, ug)]
// K = J * invM * JT = invMass + invI * cross(r, ug)^2

B2_API
void
b2JointGearDefReset(
    struct b2JointGearDef* p)
{
    b2JointDefReset((struct b2JointDef*)p);
    p->type = b2JointTypeGear;
    p->joint1 = NULL;
    p->joint2 = NULL;
    p->ratio = 1.0f;
}

B2_API
void
b2JointGearPrepare(
    struct b2JointGear* p,
    const struct b2JointGearDef* def)
{
    float coordinateA, coordinateB;

    b2TransformConstRef xfA;
    float aA;
    b2TransformConstRef xfC;
    float aC;

    b2TransformConstRef xfB;
    float aB;
    b2TransformConstRef xfD;
    float aD;

    b2JointFromDef((struct b2Joint*)p, (struct b2JointDef*)def);
    p->Meta = &b2JointGearMeta;

    p->m_joint1 = def->joint1;
    p->m_joint2 = def->joint2;

    p->m_typeA = b2JointGetType(p->m_joint1);
    p->m_typeB = b2JointGetType(p->m_joint2);

    b2Assert(p->m_typeA == b2JointTypeRevolute || p->m_typeA == b2JointTypePrismatic);
    b2Assert(p->m_typeB == b2JointTypeRevolute || p->m_typeB == b2JointTypePrismatic);

    // TODO_ERIN there might be some problem with the joint edges in b2Joint.

    p->m_bodyC = b2JointGetBodyA(p->m_joint1);
    p->m_bodyA = b2JointGetBodyB(p->m_joint1);

    // Body B on joint1 must be dynamic
    b2Assert(p->m_bodyA->m_type == b2BodyTypeDynamic);

    // Get geometry of joint1
    xfA = p->m_bodyA->m_xf;
    aA = p->m_bodyA->m_sweep.a;
    xfC = p->m_bodyC->m_xf;
    aC = p->m_bodyC->m_sweep.a;

    if (p->m_typeA == b2JointTypeRevolute)
    {
        struct b2JointRevolute* revolute = (struct b2JointRevolute*)def->joint1;
        b2Vec2Assign(p->m_localAnchorC, revolute->m_localAnchorA);
        b2Vec2Assign(p->m_localAnchorA, revolute->m_localAnchorB);
        p->m_referenceAngleA = revolute->m_referenceAngle;
        b2Vec2SetZero(p->m_localAxisC);

        coordinateA = aA - aC - p->m_referenceAngleA;

        // position error is measured in radians
        p->m_tolerance = b2_angularSlop;
    }
    else
    {
        b2Vec2 v;
        b2Vec2 v1, v2;

        b2Vec2 pC;
        b2Vec2 pA;

        struct b2JointPrismatic* prismatic = (struct b2JointPrismatic*)def->joint1;
        b2Vec2Assign(p->m_localAnchorC, prismatic->m_localAnchorA);
        b2Vec2Assign(p->m_localAnchorA, prismatic->m_localAnchorB);
        p->m_referenceAngleA = prismatic->m_referenceAngle;
        b2Vec2Assign(p->m_localAxisC, prismatic->m_localXAxisA);

        b2Vec2Assign(pC, p->m_localAnchorC);
        b2RotMulVec2(v1, xfA[1], p->m_localAnchorA);
        b2RotMulTVec2(v1, xfC[1], v1);
        b2Vec2Sub(v2, xfA[0], xfC[0]);
        b2Vec2Add(pA, v1, v2);
        b2Vec2Sub(v, pA, pC);
        coordinateA = b2Vec2DotProduct(v, p->m_localAxisC);

        // position error is measured in meters
        p->m_tolerance = b2_linearSlop;
    }

    p->m_bodyD = b2JointGetBodyA(p->m_joint2);
    p->m_bodyB = b2JointGetBodyB(p->m_joint2);

    // Body B on joint2 must be dynamic
    b2Assert(p->m_bodyB->m_type == b2BodyTypeDynamic);

    // Get geometry of joint2
    xfB = p->m_bodyB->m_xf;
    aB = p->m_bodyB->m_sweep.a;
    xfD = p->m_bodyD->m_xf;
    aD = p->m_bodyD->m_sweep.a;

    if (p->m_typeB == b2JointTypeRevolute)
    {
        struct b2JointRevolute* revolute = (struct b2JointRevolute*)def->joint2;
        b2Vec2Assign(p->m_localAnchorD, revolute->m_localAnchorA);
        b2Vec2Assign(p->m_localAnchorB, revolute->m_localAnchorB);
        p->m_referenceAngleB = revolute->m_referenceAngle;
        b2Vec2SetZero(p->m_localAxisD);

        coordinateB = aB - aD - p->m_referenceAngleB;
    }
    else
    {
        b2Vec2 v;
        b2Vec2 v1, v2;

        b2Vec2 pD;
        b2Vec2 pB;

        struct b2JointPrismatic* prismatic = (struct b2JointPrismatic*)def->joint2;
        b2Vec2Assign(p->m_localAnchorD, prismatic->m_localAnchorA);
        b2Vec2Assign(p->m_localAnchorB, prismatic->m_localAnchorB);
        p->m_referenceAngleB = prismatic->m_referenceAngle;
        b2Vec2Assign(p->m_localAxisD, prismatic->m_localXAxisA);

        b2Vec2Assign(pD, p->m_localAnchorD);
        b2RotMulVec2(v1, xfB[1], p->m_localAnchorB);
        b2RotMulTVec2(v1, xfD[1], v1);
        b2Vec2Sub(v2, xfB[0], xfD[0]);
        b2Vec2Add(pB, v1, v2);
        b2Vec2Sub(v, pB, pD);
        coordinateB = b2Vec2DotProduct(v, p->m_localAxisD);
    }

    p->m_ratio = def->ratio;

    p->m_constant = coordinateA + p->m_ratio * coordinateB;

    p->m_impulse = 0.0f;
}


B2_API
void
b2JointGearDiscard(
    struct b2JointGear* p)
{
    memset(p, 0, sizeof(struct b2JointGear));
}

B2_API
void
b2JointGearGetAnchorA(
    const struct b2JointGear* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyA, p->m_localAnchorA, anchor);
}

B2_API
void
b2JointGearGetAnchorB(
    const struct b2JointGear* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyB, p->m_localAnchorB, anchor);
}

B2_API
void
b2JointGearGetReactionForce(
    const struct b2JointGear* p,
    float inv_dt,
    b2Vec2 force)
{
    b2Vec2 P;

    b2Vec2Scale(P, p->m_JvAC, p->m_impulse);
    b2Vec2Scale(force, P, inv_dt);
}

/// Get the reaction torque given the inverse time step.
/// Unit is N*m. This is always zero for a distance joint.
B2_API
float
b2JointGearGetReactionTorque(
    const struct b2JointGear* p,
    float inv_dt)
{
    float L = p->m_impulse * p->m_JwA;
    return inv_dt * L;
}

/// Dump joint to dmLog
B2_API
void
b2JointGearDump(
    const struct b2JointGear* p)
{
    int32 indexA = p->m_bodyA->m_islandIndex;
    int32 indexB = p->m_bodyB->m_islandIndex;

    int32 index1 = p->m_joint1->m_index;
    int32 index2 = p->m_joint2->m_index;

    b2Dump("  b2GearJointDef jd;\n");
    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
    b2Dump("  jd.collideConnected = bool(%d);\n", p->m_collideConnected);
    b2Dump("  jd.joint1 = joints[%d];\n", index1);
    b2Dump("  jd.joint2 = joints[%d];\n", index2);
    b2Dump("  jd.ratio = %.9g;\n", p->m_ratio);
    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", p->m_index);
}

B2_API
void
b2JointGearShiftOrigin(
    struct b2JointGear* p,
    const b2Vec2 newOrigin)
{
    B2_NOT_USED(newOrigin);
}

B2_API
void
b2JointGearDraw(
    const struct b2JointGear* p,
    struct b2Draw* draw)
{
    static const b2Color color = { 0.5f, 0.8f, 0.8f, 1.0f };

    b2TransformConstRef xf1;
    b2TransformConstRef xf2;
    b2Vec2 x1;
    b2Vec2 x2;
    b2Vec2 p1;
    b2Vec2 p2;

    xf1 = b2BodyGetTransform(p->m_bodyA);
    xf2 = b2BodyGetTransform(p->m_bodyB);
    b2Vec2Assign(x1, xf1[0]);
    b2Vec2Assign(x2, xf2[0]);
    b2JointGearGetAnchorA(p, p1);
    b2JointGearGetAnchorB(p, p2);

    b2DrawSegment(draw, x1, p1, color);
    b2DrawSegment(draw, p1, p2, color);
    b2DrawSegment(draw, x2, p2, color);
}

B2_API
void
b2JointGearInitVelocityConstraints(
    struct b2JointGear* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;

    float aA;
    b2Vec2 vA;
    float wA;

    float aB;
    b2Vec2 vB;
    float wB;

    float aC;
    b2Vec2 vC;
    float wC;

    float aD;
    b2Vec2 vD;
    float wD;

    b2Rot qA, qB, qC, qD;

    p->m_indexA = p->m_bodyA->m_islandIndex;
    p->m_indexB = p->m_bodyB->m_islandIndex;
    p->m_indexC = p->m_bodyC->m_islandIndex;
    p->m_indexD = p->m_bodyD->m_islandIndex;
    b2Vec2Assign(p->m_lcA, p->m_bodyA->m_sweep.localCenter);
    b2Vec2Assign(p->m_lcB, p->m_bodyB->m_sweep.localCenter);
    b2Vec2Assign(p->m_lcC, p->m_bodyC->m_sweep.localCenter);
    b2Vec2Assign(p->m_lcD, p->m_bodyD->m_sweep.localCenter);
    p->m_mA = p->m_bodyA->m_invMass;
    p->m_mB = p->m_bodyB->m_invMass;
    p->m_mC = p->m_bodyC->m_invMass;
    p->m_mD = p->m_bodyD->m_invMass;
    p->m_iA = p->m_bodyA->m_invI;
    p->m_iB = p->m_bodyB->m_invI;
    p->m_iC = p->m_bodyC->m_invI;
    p->m_iD = p->m_bodyD->m_invI;

    aA = data->positions[p->m_indexA].a;
    b2Vec2Assign(vA, data->velocities[p->m_indexA].v);
    wA = data->velocities[p->m_indexA].w;

    aB = data->positions[p->m_indexB].a;
    b2Vec2Assign(vB, data->velocities[p->m_indexB].v);
    wB = data->velocities[p->m_indexB].w;

    aC = data->positions[p->m_indexC].a;
    b2Vec2Assign(vC, data->velocities[p->m_indexC].v);
    wC = data->velocities[p->m_indexC].w;

    aD = data->positions[p->m_indexD].a;
    b2Vec2Assign(vD, data->velocities[p->m_indexD].v);
    wD = data->velocities[p->m_indexD].w;

    b2RotFromAngle(qA, aA);
    b2RotFromAngle(qB, aB);
    b2RotFromAngle(qC, aC);
    b2RotFromAngle(qD, aD);

    p->m_mass = 0.0f;

    if (p->m_typeA == b2JointTypeRevolute)
    {
        b2Vec2SetZero(p->m_JvAC);
        p->m_JwA = 1.0f;
        p->m_JwC = 1.0f;
        p->m_mass += p->m_iA + p->m_iC;
    }
    else
    {
        b2Vec2 u;
        b2Vec2 rC;
        b2Vec2 rA;

        b2RotMulVec2(u, qC, p->m_localAxisC);
        b2Vec2Sub(v, p->m_localAnchorC, p->m_lcC);
        b2RotMulVec2(rC, qC, v);
        b2Vec2Sub(v, p->m_localAnchorA, p->m_lcA);
        b2RotMulVec2(rA, qA, v);
        b2Vec2Assign(p->m_JvAC, u);
        p->m_JwC = b2Vec2CrossProduct(rC, u);
        p->m_JwA = b2Vec2CrossProduct(rA, u);
        p->m_mass += p->m_mC + p->m_mA + p->m_iC * p->m_JwC * p->m_JwC + p->m_iA * p->m_JwA * p->m_JwA;
    }

    if (p->m_typeB == b2JointTypeRevolute)
    {
        b2Vec2SetZero(p->m_JvBD);
        p->m_JwB = p->m_ratio;
        p->m_JwD = p->m_ratio;
        p->m_mass += p->m_ratio * p->m_ratio * (p->m_iB + p->m_iD);
    }
    else
    {
        b2Vec2 u;
        b2Vec2 rD;
        b2Vec2 rB;

        b2RotMulVec2(u, qD, p->m_localAxisD);
        b2Vec2Sub(v, p->m_localAnchorD, p->m_lcD);
        b2RotMulVec2(rD, qD, v);
        b2Vec2Sub(v, p->m_localAnchorB, p->m_lcB);
        b2RotMulVec2(rB, qB, v);
        b2Vec2Scale(p->m_JvBD, u, p->m_ratio);
        p->m_JwD = p->m_ratio * b2Vec2CrossProduct(rD, u);
        p->m_JwB = p->m_ratio * b2Vec2CrossProduct(rB, u);
        p->m_mass += p->m_ratio * p->m_ratio * (p->m_mD + p->m_mB) + p->m_iD * p->m_JwD * p->m_JwD + p->m_iB * p->m_JwB * p->m_JwB;
    }

    // Compute effective mass.
    p->m_mass = p->m_mass > 0.0f ? 1.0f / p->m_mass : 0.0f;

    if (data->step.warmStarting)
    {
        b2Vec2Scale(v, p->m_JvAC, p->m_mA * p->m_impulse);
        b2Vec2Add(vA, vA, v);
        wA += p->m_iA * p->m_impulse * p->m_JwA;
        b2Vec2Scale(v, p->m_JvBD, p->m_mB * p->m_impulse);
        b2Vec2Add(vB, vB, v);
        wB += p->m_iB * p->m_impulse * p->m_JwB;
        b2Vec2Scale(v, p->m_JvAC, p->m_mC * p->m_impulse);
        b2Vec2Sub(vC, vC, v);
        wC -= p->m_iC * p->m_impulse * p->m_JwC;
        b2Vec2Scale(v, p->m_JvBD, p->m_mD * p->m_impulse);
        b2Vec2Sub(vD, vD, v);
        wD -= p->m_iD * p->m_impulse * p->m_JwD;
    }
    else
    {
        p->m_impulse = 0.0f;
    }

    b2Vec2Assign(data->velocities[p->m_indexA].v, vA);
    data->velocities[p->m_indexA].w = wA;
    b2Vec2Assign(data->velocities[p->m_indexB].v, vB);
    data->velocities[p->m_indexB].w = wB;
    b2Vec2Assign(data->velocities[p->m_indexC].v, vC);
    data->velocities[p->m_indexC].w = wC;
    b2Vec2Assign(data->velocities[p->m_indexD].v, vD);
    data->velocities[p->m_indexD].w = wD;
}

B2_API
void
b2JointGearSolveVelocityConstraints(
    struct b2JointGear* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;
    b2Vec2 v1, v2;

    b2Vec2 vA;
    float wA;
    b2Vec2 vB;
    float wB;
    b2Vec2 vC;
    float wC;
    b2Vec2 vD;
    float wD;

    float Cdot;

    float impulse;

    b2Vec2Assign(vA, data->velocities[p->m_indexA].v);
    wA = data->velocities[p->m_indexA].w;
    b2Vec2Assign(vB, data->velocities[p->m_indexB].v);
    wB = data->velocities[p->m_indexB].w;
    b2Vec2Assign(vC, data->velocities[p->m_indexC].v);
    wC = data->velocities[p->m_indexC].w;
    b2Vec2Assign(vD, data->velocities[p->m_indexD].v);
    wD = data->velocities[p->m_indexD].w;

    b2Vec2Sub(v1, vA, vC);
    b2Vec2Sub(v2, vB, vD);
    Cdot = b2Vec2DotProduct(p->m_JvAC, v1) + b2Vec2DotProduct(p->m_JvBD, v2);
    Cdot += (p->m_JwA * wA - p->m_JwC * wC) + (p->m_JwB * wB - p->m_JwD * wD);

    impulse = -p->m_mass * Cdot;
    p->m_impulse += impulse;

    b2Vec2Scale(v, p->m_JvAC, p->m_mA * impulse);
    b2Vec2Add(vA, vA, v);
    wA += p->m_iA * impulse * p->m_JwA;
    b2Vec2Scale(v, p->m_JvBD, p->m_mB * impulse);
    b2Vec2Add(vB, vB, v);
    wB += p->m_iB * impulse * p->m_JwB;
    b2Vec2Scale(v, p->m_JvAC, p->m_mC * impulse);
    b2Vec2Sub(vC, vC, v);
    wC -= p->m_iC * impulse * p->m_JwC;
    b2Vec2Scale(v, p->m_JvBD, p->m_mD * impulse);
    b2Vec2Sub(vD, vD, v);
    wD -= p->m_iD * impulse * p->m_JwD;

    b2Vec2Assign(data->velocities[p->m_indexA].v, vA);
    data->velocities[p->m_indexA].w = wA;
    b2Vec2Assign(data->velocities[p->m_indexB].v, vB);
    data->velocities[p->m_indexB].w = wB;
    b2Vec2Assign(data->velocities[p->m_indexC].v, vC);
    data->velocities[p->m_indexC].w = wC;
    b2Vec2Assign(data->velocities[p->m_indexD].v, vD);
    data->velocities[p->m_indexD].w = wD;
}

B2_API
int
b2JointGearSolvePositionConstraints(
    struct b2JointGear* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;

    b2Vec2 cA;
    float aA;
    b2Vec2 cB;
    float aB;
    b2Vec2 cC;
    float aC;
    b2Vec2 cD;
    float aD;

    b2Rot qA, qB, qC, qD;

    float coordinateA, coordinateB;

    b2Vec2 JvAC, JvBD;
    float JwA, JwB, JwC, JwD;
    float mass;

    float C;

    float impulse;

    b2Vec2Assign(cA, data->positions[p->m_indexA].c);
    aA = data->positions[p->m_indexA].a;
    b2Vec2Assign(cB, data->positions[p->m_indexB].c);
    aB = data->positions[p->m_indexB].a;
    b2Vec2Assign(cC, data->positions[p->m_indexC].c);
    aC = data->positions[p->m_indexC].a;
    b2Vec2Assign(cD, data->positions[p->m_indexD].c);
    aD = data->positions[p->m_indexD].a;

    b2RotFromAngle(qA, aA);
    b2RotFromAngle(qB, aB);
    b2RotFromAngle(qC, aC);
    b2RotFromAngle(qD, aD);

    mass = 0.0f;

    if (p->m_typeA == b2JointTypeRevolute)
    {
        b2Vec2SetZero(JvAC);
        JwA = 1.0f;
        JwC = 1.0f;
        mass += p->m_iA + p->m_iC;

        coordinateA = aA - aC - p->m_referenceAngleA;
    }
    else
    {
        b2Vec2 u;
        b2Vec2 rC;
        b2Vec2 rA;

        b2Vec2 pC;
        b2Vec2 pA;

        b2RotMulVec2(u, qC, p->m_localAxisC);
        b2Vec2Sub(v, p->m_localAnchorC, p->m_lcC);
        b2RotMulVec2(rC, qC, v);
        b2Vec2Sub(v, p->m_localAnchorA, p->m_lcA);
        b2RotMulVec2(rA, qA, v);
        b2Vec2Assign(JvAC, u);
        JwC = b2Vec2CrossProduct(rC, u);
        JwA = b2Vec2CrossProduct(rA, u);
        mass += p->m_mC + p->m_mA + p->m_iC * JwC * JwC + p->m_iA * JwA * JwA;

        b2Vec2Sub(pC, p->m_localAnchorC, p->m_lcC);
        b2Vec2Sub(v, cA, cC);
        b2Vec2Add(v, rA, v);
        b2RotMulTVec2(pA, qC, v);
        b2Vec2Sub(v, pA, pC);
        coordinateA = b2Vec2DotProduct(v, p->m_localAxisC);
    }

    if (p->m_typeB == b2JointTypeRevolute)
    {
        b2Vec2SetZero(JvBD);
        JwB = p->m_ratio;
        JwD = p->m_ratio;
        mass += p->m_ratio * p->m_ratio * (p->m_iB + p->m_iD);

        coordinateB = aB - aD - p->m_referenceAngleB;
    }
    else
    {
        b2Vec2 u;
        b2Vec2 rD;
        b2Vec2 rB;

        b2Vec2 pD;
        b2Vec2 pB;

        b2RotMulVec2(u, qD, p->m_localAxisD);
        b2Vec2Sub(v, p->m_localAnchorD, p->m_lcD);
        b2RotMulVec2(rD, qD, v);
        b2Vec2Sub(v, p->m_localAnchorB, p->m_lcB);
        b2RotMulVec2(rB, qB, v);
        b2Vec2Scale(JvBD, u, p->m_ratio);
        JwD = p->m_ratio * b2Vec2CrossProduct(rD, u);
        JwB = p->m_ratio * b2Vec2CrossProduct(rB, u);
        mass += p->m_ratio * p->m_ratio * (p->m_mD + p->m_mB) + p->m_iD * JwD * JwD + p->m_iB * JwB * JwB;

        b2Vec2Sub(pD, p->m_localAnchorD, p->m_lcD);
        b2Vec2Sub(v, cB, cD);
        b2Vec2Add(v, rB, v);
        b2RotMulTVec2(pB, qD, v);
        b2Vec2Sub(v, pB, pD);
        coordinateB = b2Vec2DotProduct(v, p->m_localAxisD);
    }

    C = (coordinateA + p->m_ratio * coordinateB) - p->m_constant;

    if (mass > 0.0f)
    {
        impulse = -C / mass;
    }
    else
    {
        impulse = 0.0f;
    }

    b2Vec2Scale(v, JvAC, p->m_mA * impulse);
    b2Vec2Add(cA, cA, v);
    aA += p->m_iA * impulse * JwA;
    b2Vec2Scale(v, JvBD, p->m_mB * impulse);
    b2Vec2Add(cB, cB, v);
    aB += p->m_iB * impulse * JwB;
    b2Vec2Scale(v, JvAC, p->m_mC * impulse);
    b2Vec2Sub(cC, cC, v);
    aC -= p->m_iC * impulse * JwC;
    b2Vec2Scale(v, JvBD, p->m_mD * impulse);
    b2Vec2Sub(cD, cD, v);
    aD -= p->m_iD * impulse * JwD;

    b2Vec2Assign(data->positions[p->m_indexA].c, cA);
    data->positions[p->m_indexA].a = aA;
    b2Vec2Assign(data->positions[p->m_indexB].c, cB);
    data->positions[p->m_indexB].a = aB;
    b2Vec2Assign(data->positions[p->m_indexC].c, cC);
    data->positions[p->m_indexC].a = aC;
    b2Vec2Assign(data->positions[p->m_indexD].c, cD);
    data->positions[p->m_indexD].a = aD;

    if (b2AbsFloat(C) < p->m_tolerance)
    {
        return b2True;
    }
    else
    {
        return b2False;
    }
}

B2_API
void
b2JointGearSetRatio(
    struct b2JointGear* p,
    float ratio)
{
    b2Assert(b2IsValid(ratio));
    p->m_ratio = ratio;
}
