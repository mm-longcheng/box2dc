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

#include "mmB2JointPrismatic.h"
#include "mmB2Common.h"
#include "mmB2Body.h"
#include "mmB2Draw.h"
#include "mmB2TimeStep.h"

#include <assert.h>
#include <string.h>

B2_API const struct b2MetaAllocator b2MetaAllocatorJointPrismatic =
{
    "b2JointPrismatic",
    sizeof(struct b2JointPrismatic),
    &b2JointPrismaticPrepare,
    &b2JointPrismaticDiscard,
};

B2_API const struct b2JointMeta b2JointPrismaticMeta =
{
    &b2JointPrismaticGetAnchorA,
    &b2JointPrismaticGetAnchorB,
    &b2JointPrismaticGetReactionForce,
    &b2JointPrismaticGetReactionTorque,
    &b2JointPrismaticDump,
    &b2JointPrismaticShiftOrigin,
    &b2JointPrismaticDraw,
    &b2JointPrismaticInitVelocityConstraints,
    &b2JointPrismaticSolveVelocityConstraints,
    &b2JointPrismaticSolvePositionConstraints,
};

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)

// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want C2 = C1 + h * Cdot >= 0
// Or:
// Cdot + C1/h >= 0
// I do not apply a negative constraint error because that is handled in position correction.
// So:
// Cdot + max(C1, 0)/h >= 0

// Block Solver
// We develop a block solver that includes the angular and linear constraints. This makes the limit stiffer.
//
// The Jacobian has 2 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//
// u = perp
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

B2_API
void
b2JointPrismaticDefReset(
    struct b2JointPrismaticDef* p)
{
    b2JointDefReset((struct b2JointDef*)p);
    p->type = b2JointTypePrismatic;
    b2Vec2SetZero(p->localAnchorA);
    b2Vec2SetZero(p->localAnchorB);
    b2Vec2Make(p->localAxisA, 1.0f, 0.0f);
    p->referenceAngle = 0.0f;
    p->enableLimit = b2False;
    p->lowerTranslation = 0.0f;
    p->upperTranslation = 0.0f;
    p->enableMotor = b2False;
    p->maxMotorForce = 0.0f;
    p->motorSpeed = 0.0f;
}

B2_API
void
b2JointPrismaticDefInitialize(
    struct b2JointPrismaticDef* p,
    struct b2Body* bodyA,
    struct b2Body* bodyB,
    const b2Vec2 anchor,
    const b2Vec2 axis)
{
    p->bodyA = bodyA;
    p->bodyB = bodyB;
    b2BodyGetLocalPoint(bodyA, anchor, p->localAnchorA);
    b2BodyGetLocalPoint(bodyB, anchor, p->localAnchorB);
    b2BodyGetLocalVector(bodyA, axis, p->localAxisA);
    p->referenceAngle = b2BodyGetAngle(bodyB) - b2BodyGetAngle(bodyA);
}

B2_API
void
b2JointPrismaticPrepare(
    struct b2JointPrismatic* p,
    const struct b2JointPrismaticDef* def)
{
    b2JointFromDef((struct b2Joint*)p, (struct b2JointDef*)def);
    p->Meta = &b2JointPrismaticMeta;

    b2Vec2Assign(p->m_localAnchorA, def->localAnchorA);
    b2Vec2Assign(p->m_localAnchorB, def->localAnchorB);
    b2Vec2Assign(p->m_localXAxisA, def->localAxisA);
    b2Vec2Normalize(p->m_localXAxisA, p->m_localXAxisA);
    b2Vec2CrossProductKL(p->m_localYAxisA, 1.0f, p->m_localXAxisA);
    p->m_referenceAngle = def->referenceAngle;

    b2Vec2SetZero(p->m_impulse);
    p->m_axialMass = 0.0f;
    p->m_motorImpulse = 0.0f;
    p->m_lowerImpulse = 0.0f;
    p->m_upperImpulse = 0.0f;

    p->m_lowerTranslation = def->lowerTranslation;
    p->m_upperTranslation = def->upperTranslation;

    b2Assert(p->m_lowerTranslation <= p->m_upperTranslation);

    p->m_maxMotorForce = def->maxMotorForce;
    p->m_motorSpeed = def->motorSpeed;
    p->m_enableLimit = def->enableLimit;
    p->m_enableMotor = def->enableMotor;

    p->m_translation = 0.0f;
    b2Vec2SetZero(p->m_axis);
    b2Vec2SetZero(p->m_perp);
}

B2_API
void
b2JointPrismaticDiscard(
    struct b2JointPrismatic* p)
{
    memset(p, 0, sizeof(struct b2JointPrismatic));
}

B2_API
void
b2JointPrismaticGetAnchorA(
    const struct b2JointPrismatic* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyA, p->m_localAnchorA, anchor);
}

B2_API
void
b2JointPrismaticGetAnchorB(
    const struct b2JointPrismatic* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyB, p->m_localAnchorB, anchor);
}

B2_API
void
b2JointPrismaticGetReactionForce(
    const struct b2JointPrismatic* p,
    float inv_dt,
    b2Vec2 force)
{
    b2Vec2 v;
    b2Vec2 v1, v2;
    b2Vec2Scale(v1, p->m_perp, p->m_impulse[0]);
    b2Vec2Scale(v2, p->m_axis, p->m_motorImpulse + p->m_lowerImpulse - p->m_upperImpulse);
    b2Vec2Add(v, v1, v2);
    b2Vec2Scale(force, v, inv_dt);
}

/// Get the reaction torque given the inverse time step.
/// Unit is N*m. This is always zero for a distance joint.
B2_API
float
b2JointPrismaticGetReactionTorque(
    const struct b2JointPrismatic* p,
    float inv_dt)
{
    return inv_dt * p->m_impulse[1];
}

/// Dump joint to dmLog
B2_API
void
b2JointPrismaticDump(
    const struct b2JointPrismatic* p)
{
    // FLT_DECIMAL_DIG == 9

    int32 indexA = p->m_bodyA->m_islandIndex;
    int32 indexB = p->m_bodyB->m_islandIndex;

    b2Dump("  b2PrismaticJointDef jd;\n");
    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
    b2Dump("  jd.collideConnected = bool(%d);\n", p->m_collideConnected);
    b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", p->m_localAnchorA[0], p->m_localAnchorA[1]);
    b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", p->m_localAnchorB[0], p->m_localAnchorB[1]);
    b2Dump("  jd.localAxisA.Set(%.9g, %.9g);\n", p->m_localXAxisA[0], p->m_localXAxisA[1]);
    b2Dump("  jd.referenceAngle = %.9g;\n", p->m_referenceAngle);
    b2Dump("  jd.enableLimit = bool(%d);\n", p->m_enableLimit);
    b2Dump("  jd.lowerTranslation = %.9g;\n", p->m_lowerTranslation);
    b2Dump("  jd.upperTranslation = %.9g;\n", p->m_upperTranslation);
    b2Dump("  jd.enableMotor = bool(%d);\n", p->m_enableMotor);
    b2Dump("  jd.motorSpeed = %.9g;\n", p->m_motorSpeed);
    b2Dump("  jd.maxMotorForce = %.9g;\n", p->m_maxMotorForce);
    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", p->m_index);
}

B2_API
void
b2JointPrismaticShiftOrigin(
    struct b2JointPrismatic* p,
    const b2Vec2 newOrigin)
{
    B2_NOT_USED(newOrigin);
}

B2_API
void
b2JointPrismaticDraw(
    const struct b2JointPrismatic* p,
    struct b2Draw* draw)
{
    static const b2Color c1 = { 0.7f, 0.7f, 0.7f, 1.0f };
    static const b2Color c2 = { 0.3f, 0.9f, 0.3f, 1.0f };
    static const b2Color c3 = { 0.9f, 0.3f, 0.3f, 1.0f };
    static const b2Color c4 = { 0.3f, 0.3f, 0.9f, 1.0f };
    static const b2Color c5 = { 0.4f, 0.4f, 0.4f, 1.0f };

    b2Vec2 v;
    b2Vec2 v1, v2;

    b2TransformConstRef xfA;
    b2TransformConstRef xfB;
    b2Vec2 pA;
    b2Vec2 pB;

    b2Vec2 axis;

    xfA = b2BodyGetTransform(p->m_bodyA);
    xfB = b2BodyGetTransform(p->m_bodyB);
    b2TransformMulVec2(pA, xfA, p->m_localAnchorA);
    b2TransformMulVec2(pB, xfB, p->m_localAnchorB);

    b2RotMulVec2(axis, xfA[1], p->m_localXAxisA);

    b2DrawSegment(draw, pA, pB, c5);

    if (p->m_enableLimit)
    {
        b2Vec2 lower;
        b2Vec2 upper;
        b2Vec2 perp;

        b2Vec2Scale(v, axis, p->m_lowerTranslation);
        b2Vec2Add(lower, pA, v);
        b2Vec2Scale(v, axis, p->m_upperTranslation);
        b2Vec2Add(upper, pA, v);
        b2RotMulVec2(perp, xfA[1], p->m_localYAxisA);
        b2DrawSegment(draw, lower, upper, c1);

        b2Vec2Scale(v1, perp, 0.5f);
        b2Vec2Sub(v1, lower, v1);
        b2Vec2Scale(v2, perp, 0.5f);
        b2Vec2Add(v2, lower, v2);
        b2DrawSegment(draw, v1, v2, c2);

        b2Vec2Scale(v1, perp, 0.5f);
        b2Vec2Sub(v1, upper, v1);
        b2Vec2Scale(v2, perp, 0.5f);
        b2Vec2Add(v2, upper, v2);
        b2DrawSegment(draw, v1, v2, c3);
    }
    else
    {
        b2Vec2Scale(v1, axis, 1.0f);
        b2Vec2Sub(v1, pA, v1);
        b2Vec2Scale(v2, axis, 1.0f);
        b2Vec2Add(v2, pA, v2);
        b2DrawSegment(draw, v1, v2, c1);
    }

    b2DrawPoint(draw, pA, 5.0f, c1);
    b2DrawPoint(draw, pB, 5.0f, c4);
}

B2_API
void
b2JointPrismaticInitVelocityConstraints(
    struct b2JointPrismatic* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;
    b2Vec2 v1, v2;

    b2Vec2 cA;
    float aA;
    b2Vec2 vA;
    float wA;

    b2Vec2 cB;
    float aB;
    b2Vec2 vB;
    float wB;

    b2Rot qA, qB;

    b2Vec2 rA;
    b2Vec2 rB;
    b2Vec2 d;

    float mA, mB;
    float iA, iB;

    p->m_indexA = p->m_bodyA->m_islandIndex;
    p->m_indexB = p->m_bodyB->m_islandIndex;
    b2Vec2Assign(p->m_localCenterA, p->m_bodyA->m_sweep.localCenter);
    b2Vec2Assign(p->m_localCenterB, p->m_bodyB->m_sweep.localCenter);
    p->m_invMassA = p->m_bodyA->m_invMass;
    p->m_invMassB = p->m_bodyB->m_invMass;
    p->m_invIA = p->m_bodyA->m_invI;
    p->m_invIB = p->m_bodyB->m_invI;

    b2Vec2Assign(cA, data->positions[p->m_indexA].c);
    aA = data->positions[p->m_indexA].a;
    b2Vec2Assign(vA, data->velocities[p->m_indexA].v);
    wA = data->velocities[p->m_indexA].w;

    b2Vec2Assign(cB, data->positions[p->m_indexB].c);
    aB = data->positions[p->m_indexB].a;
    b2Vec2Assign(vB, data->velocities[p->m_indexB].v);
    wB = data->velocities[p->m_indexB].w;

    b2RotFromAngle(qA, aA);
    b2RotFromAngle(qB, aB);

    // Compute the effective masses.
    b2Vec2Sub(v, p->m_localAnchorA, p->m_localCenterA);
    b2RotMulVec2(rA, qA, v);
    b2Vec2Sub(v, p->m_localAnchorB, p->m_localCenterB);
    b2RotMulVec2(rB, qB, v);
    b2Vec2Sub(v, cB, cA);
    b2Vec2Add(v, v, rB);
    b2Vec2Sub(d, v, rA);

    mA = p->m_invMassA, mB = p->m_invMassB;
    iA = p->m_invIA   , iB = p->m_invIB;

    // Compute motor Jacobian and effective mass.
    {
        b2RotMulVec2(p->m_axis, qA, p->m_localXAxisA);
        b2Vec2Add(v, d, rA);
        p->m_a1 = b2Vec2CrossProduct(v, p->m_axis);
        p->m_a2 = b2Vec2CrossProduct(rB, p->m_axis);

        p->m_axialMass = mA + mB + iA * p->m_a1 * p->m_a1 + iB * p->m_a2 * p->m_a2;
        if (p->m_axialMass > 0.0f)
        {
            p->m_axialMass = 1.0f / p->m_axialMass;
        }
    }

    // Prismatic constraint.
    {
        float k11;
        float k12;
        float k22;

        b2RotMulVec2(p->m_perp, qA, p->m_localYAxisA);

        b2Vec2Add(v, d, rA);
        p->m_s1 = b2Vec2CrossProduct(v, p->m_perp);
        p->m_s2 = b2Vec2CrossProduct(rB, p->m_perp);

        k11 = mA + mB + iA * p->m_s1 * p->m_s1 + iB * p->m_s2 * p->m_s2;
        k12 = iA * p->m_s1 + iB * p->m_s2;
        k22 = iA + iB;
        if (k22 == 0.0f)
        {
            // For bodies with fixed rotation.
            k22 = 1.0f;
        }

        b2Vec2Make(p->m_K[0], k11, k12);
        b2Vec2Make(p->m_K[1], k12, k22);
    }

    if (p->m_enableLimit)
    {
        p->m_translation = b2Vec2DotProduct(p->m_axis, d);
    }
    else
    {
        p->m_lowerImpulse = 0.0f;
        p->m_upperImpulse = 0.0f;
    }

    if (p->m_enableMotor == b2False)
    {
        p->m_motorImpulse = 0.0f;
    }

    if (data->step.warmStarting)
    {
        float axialImpulse;
        b2Vec2 P;
        float LA;
        float LB;

        // Account for variable time step.
        b2Vec2Scale(p->m_impulse, p->m_impulse, data->step.dtRatio);
        p->m_motorImpulse *= data->step.dtRatio;
        p->m_lowerImpulse *= data->step.dtRatio;
        p->m_upperImpulse *= data->step.dtRatio;

        axialImpulse = p->m_motorImpulse + p->m_lowerImpulse - p->m_upperImpulse;
        b2Vec2Scale(v1, p->m_perp, p->m_impulse[0]);
        b2Vec2Scale(v2, p->m_axis, axialImpulse);
        b2Vec2Add(P, v1, v2);
        LA = p->m_impulse[0] * p->m_s1 + p->m_impulse[1] + axialImpulse * p->m_a1;
        LB = p->m_impulse[0] * p->m_s2 + p->m_impulse[1] + axialImpulse * p->m_a2;

        b2Vec2Scale(v, P, mA);
        b2Vec2Sub(vA, vA, v);
        wA -= iA * LA;

        b2Vec2Scale(v, P, mB);
        b2Vec2Add(vB, vB, v);
        wB += iB * LB;
    }
    else
    {
        b2Vec2SetZero(p->m_impulse);
        p->m_motorImpulse = 0.0f;
        p->m_lowerImpulse = 0.0f;
        p->m_upperImpulse = 0.0f;
    }

    b2Vec2Assign(data->velocities[p->m_indexA].v, vA);
    data->velocities[p->m_indexA].w = wA;
    b2Vec2Assign(data->velocities[p->m_indexB].v, vB);
    data->velocities[p->m_indexB].w = wB;
}

B2_API
void
b2JointPrismaticSolveVelocityConstraints(
    struct b2JointPrismatic* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;

    b2Vec2 vA;
    float wA;
    b2Vec2 vB;
    float wB;

    float mA, mB;
    float iA, iB;


    b2Vec2Assign(vA, data->velocities[p->m_indexA].v);
    wA = data->velocities[p->m_indexA].w;
    b2Vec2Assign(vB, data->velocities[p->m_indexB].v);
    wB = data->velocities[p->m_indexB].w;

    mA = p->m_invMassA, mB = p->m_invMassB;
    iA = p->m_invIA   , iB = p->m_invIB;

    // Solve linear motor constraint
    if (p->m_enableMotor)
    {
        float Cdot;
        float impulse;
        float oldImpulse;
        float maxImpulse;

        b2Vec2 P;
        float LA;
        float LB;

        b2Vec2Sub(v, vB, vA);
        Cdot = b2Vec2DotProduct(p->m_axis, v) + p->m_a2 * wB - p->m_a1 * wA;
        impulse = p->m_axialMass * (p->m_motorSpeed - Cdot);
        oldImpulse = p->m_motorImpulse;
        maxImpulse = data->step.dt * p->m_maxMotorForce;
        p->m_motorImpulse = b2ClampFloat(p->m_motorImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = p->m_motorImpulse - oldImpulse;

        b2Vec2Scale(P, p->m_axis, impulse);
        LA = impulse * p->m_a1;
        LB = impulse * p->m_a2;

        b2Vec2Scale(v, P, mA);
        b2Vec2Sub(vA, vA, v);
        wA -= iA * LA;
        b2Vec2Scale(v, P, mB);
        b2Vec2Add(vB, vB, v);
        wB += iB * LB;
    }

    if (p->m_enableLimit)
    {
        // Lower limit
        {
            float C;
            float Cdot;
            float impulse;
            float oldImpulse;

            b2Vec2 P;
            float LA;
            float LB;

            C = p->m_translation - p->m_lowerTranslation;
            b2Vec2Sub(v, vB, vA);
            Cdot = b2Vec2DotProduct(p->m_axis, v) + p->m_a2 * wB - p->m_a1 * wA;
            impulse = -p->m_axialMass * (Cdot + b2MaxFloat(C, 0.0f) * data->step.inv_dt);
            oldImpulse = p->m_lowerImpulse;
            p->m_lowerImpulse = b2MaxFloat(p->m_lowerImpulse + impulse, 0.0f);
            impulse = p->m_lowerImpulse - oldImpulse;

            b2Vec2Scale(P, p->m_axis, impulse);
            LA = impulse * p->m_a1;
            LB = impulse * p->m_a2;

            b2Vec2Scale(v, P, mA);
            b2Vec2Sub(vA, vA, v);
            wA -= iA * LA;
            b2Vec2Scale(v, P, mB);
            b2Vec2Add(vB, vB, v);
            wB += iB * LB;
        }

        // Upper limit
        // Note: signs are flipped to keep C positive when the constraint is satisfied.
        // This also keeps the impulse positive when the limit is active.
        {
            float C;
            float Cdot;
            float impulse;
            float oldImpulse;

            b2Vec2 P;
            float LA;
            float LB;

            C = p->m_upperTranslation - p->m_translation;
            b2Vec2Sub(v, vA, vB);
            Cdot = b2Vec2DotProduct(p->m_axis, v) + p->m_a1 * wA - p->m_a2 * wB;
            impulse = -p->m_axialMass * (Cdot + b2MaxFloat(C, 0.0f) * data->step.inv_dt);
            oldImpulse = p->m_upperImpulse;
            p->m_upperImpulse = b2MaxFloat(p->m_upperImpulse + impulse, 0.0f);
            impulse = p->m_upperImpulse - oldImpulse;

            b2Vec2Scale(P, p->m_axis, impulse);
            LA = impulse * p->m_a1;
            LB = impulse * p->m_a2;

            b2Vec2Scale(v, P, mA);
            b2Vec2Add(vA, vA, v);
            wA += iA * LA;
            b2Vec2Scale(v, P, mB);
            b2Vec2Sub(vB, vB, v);
            wB -= iB * LB;
        }
    }

    // Solve the prismatic constraint in block form.
    {
        b2Vec2 Cdot;
        b2Vec2 df;

        b2Vec2 P;
        float LA;
        float LB;

        b2Vec2Sub(v, vB, vA);
        Cdot[0] = b2Vec2DotProduct(p->m_perp, v) + p->m_s2 * wB - p->m_s1 * wA;
        Cdot[1] = wB - wA;

        b2Vec2Negate(v, Cdot);
        b2Mat22Solve(df, p->m_K, v);
        b2Vec2Add(p->m_impulse, p->m_impulse, df);

        b2Vec2Scale(P, p->m_perp, df[0]);
        LA = df[0] * p->m_s1 + df[1];
        LB = df[0] * p->m_s2 + df[1];

        b2Vec2Scale(v, P, mA);
        b2Vec2Sub(vA, vA, v);
        wA -= iA * LA;

        b2Vec2Scale(v, P, mB);
        b2Vec2Add(vB, vB, v);
        wB += iB * LB;
    }

    b2Vec2Assign(data->velocities[p->m_indexA].v, vA);
    data->velocities[p->m_indexA].w = wA;
    b2Vec2Assign(data->velocities[p->m_indexB].v, vB);
    data->velocities[p->m_indexB].w = wB;
}

// A velocity based solver computes reaction forces(impulses) using the velocity constraint solver.Under this context,
// the position solver is not there to resolve forces.It is only there to cope with integration error.
//
// Therefore, the pseudo impulses in the position solver do not have any physical meaning.Thus it is okay if they suck.
//
// We could take the active state from the velocity solver.However, the joint might push past the limit when the velocity
// solver indicates the limit is inactive.
B2_API
int
b2JointPrismaticSolvePositionConstraints(
    struct b2JointPrismatic* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;
    b2Vec2 v1, v2;

    b2Vec2 cA;
    float aA;
    b2Vec2 cB;
    float aB;

    b2Rot qA, qB;

    float mA, mB;
    float iA, iB;

    b2Vec2 rA;
    b2Vec2 rB;
    b2Vec2 d;

    b2Vec2 axis;
    float a1;
    float a2;
    b2Vec2 perp;

    float s1;
    float s2;

    b2Vec3 impulse;
    b2Vec2 C1;

    float linearError;
    float angularError;

    int active;
    float C2;

    b2Vec2 P;
    float LA;
    float LB;

    b2Vec2Assign(cA, data->positions[p->m_indexA].c);
    aA = data->positions[p->m_indexA].a;
    b2Vec2Assign(cB, data->positions[p->m_indexB].c);
    aB = data->positions[p->m_indexB].a;

    b2RotFromAngle(qA, aA);
    b2RotFromAngle(qB, aB);

    mA = p->m_invMassA, mB = p->m_invMassB;
    iA = p->m_invIA   , iB = p->m_invIB;

    // Compute fresh Jacobians
    b2Vec2Sub(v, p->m_localAnchorA, p->m_localCenterA);
    b2RotMulVec2(rA, qA, v);
    b2Vec2Sub(v, p->m_localAnchorB, p->m_localCenterB);
    b2RotMulVec2(rB, qB, v);
    b2Vec2Add(v, cB, rB);
    b2Vec2Sub(v, v, cA);
    b2Vec2Sub(d, v, rA);

    b2RotMulVec2(axis, qA, p->m_localXAxisA);
    b2Vec2Add(v, d, rA);
    a1 = b2Vec2CrossProduct(v, axis);
    a2 = b2Vec2CrossProduct(rB, axis);
    b2RotMulVec2(perp, qA, p->m_localYAxisA);

    b2Vec2Add(v, d, rA);
    s1 = b2Vec2CrossProduct(v, perp);
    s2 = b2Vec2CrossProduct(rB, perp);

    C1[0] = b2Vec2DotProduct(perp, d);
    C1[1] = aB - aA - p->m_referenceAngle;

    linearError = b2AbsFloat(C1[0]);
    angularError = b2AbsFloat(C1[1]);

    active = b2False;
    C2 = 0.0f;
    if (p->m_enableLimit)
    {
        float translation = b2Vec2DotProduct(axis, d);
        if (b2AbsFloat(p->m_upperTranslation - p->m_lowerTranslation) < 2.0f * b2_linearSlop)
        {
            C2 = translation;
            linearError = b2MaxFloat(linearError, b2AbsFloat(translation));
            active = b2True;
        }
        else if (translation <= p->m_lowerTranslation)
        {
            C2 = b2MinFloat(translation - p->m_lowerTranslation, 0.0f);
            linearError = b2MaxFloat(linearError, p->m_lowerTranslation - translation);
            active = b2True;
        }
        else if (translation >= p->m_upperTranslation)
        {
            C2 = b2MaxFloat(translation - p->m_upperTranslation, 0.0f);
            linearError = b2MaxFloat(linearError, translation - p->m_upperTranslation);
            active = b2True;
        }
    }

    if (active)
    {
        float k11;
        float k12;
        float k13;
        float k22;
        float k23;
        float k33;

        b2Mat33 K;

        b2Vec3 C;
        b2Vec3 V;

        k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
        k12 = iA * s1 + iB * s2;
        k13 = iA * s1 * a1 + iB * s2 * a2;
        k22 = iA + iB;
        if (k22 == 0.0f)
        {
            // For fixed rotation
            k22 = 1.0f;
        }
        k23 = iA * a1 + iB * a2;
        k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

        b2Vec3Make(K[0], k11, k12, k13);
        b2Vec3Make(K[1], k12, k22, k23);
        b2Vec3Make(K[2], k13, k23, k33);

        C[0] = C1[0];
        C[1] = C1[1];
        C[2] = C2;

        b2Vec3Negate(V, C);
        b2Mat33Solve33(impulse, K, V);
    }
    else
    {
        float k11;
        float k12;
        float k22;

        b2Mat22 K;

        b2Vec2 impulse1;

        k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
        k12 = iA * s1 + iB * s2;
        k22 = iA + iB;
        if (k22 == 0.0f)
        {
            k22 = 1.0f;
        }

        b2Vec2Make(K[0], k11, k12);
        b2Vec2Make(K[1], k12, k22);

        b2Vec2Negate(v, C1);
        b2Mat22Solve(impulse1, K, v);
        impulse[0] = impulse1[0];
        impulse[1] = impulse1[1];
        impulse[2] = 0.0f;
    }

    b2Vec2Scale(v1, perp, impulse[0]);
    b2Vec2Scale(v2, axis, impulse[2]);
    b2Vec2Add(P, v1, v2);
    LA = impulse[0] * s1 + impulse[1] + impulse[2] * a1;
    LB = impulse[0] * s2 + impulse[1] + impulse[2] * a2;

    b2Vec2Scale(v, P, mA);
    b2Vec2Sub(cA, cA, v);
    aA -= iA * LA;
    b2Vec2Scale(v, P, mB);
    b2Vec2Add(cB, cB, v);
    aB += iB * LB;

    b2Vec2Assign(data->positions[p->m_indexA].c, cA);
    data->positions[p->m_indexA].a = aA;
    b2Vec2Assign(data->positions[p->m_indexB].c, cB);
    data->positions[p->m_indexB].a = aB;

    return linearError <= b2_linearSlop && angularError <= b2_angularSlop;
}

B2_API
float
b2JointPrismaticGetJointTranslation(
    const struct b2JointPrismatic* p)
{
    b2Vec2 pA;
    b2Vec2 pB;
    b2Vec2 d;
    b2Vec2 axis;

    float translation;

    b2BodyGetWorldPoint(p->m_bodyA, p->m_localAnchorA, pA);
    b2BodyGetWorldPoint(p->m_bodyB, p->m_localAnchorB, pB);
    b2Vec2Sub(d, pB, pA);
    b2BodyGetWorldVector(p->m_bodyA, p->m_localXAxisA, axis);
    translation = b2Vec2DotProduct(d, axis);
    return translation;
}

B2_API
float
b2JointPrismaticGetJointSpeed(
    const struct b2JointPrismatic* p)
{
    b2Vec2 v;
    b2Vec2 v1, v2, v3;

    struct b2Body* bA;
    struct b2Body* bB;

    b2Vec2 rA;
    b2Vec2 rB;
    b2Vec2 p1;
    b2Vec2 p2;
    b2Vec2 d;
    b2Vec2 axis;

    b2Vec2 vA;
    b2Vec2 vB;
    float wA;
    float wB;

    float speed;

    bA = p->m_bodyA;
    bB = p->m_bodyB;

    b2Vec2Sub(v, p->m_localAnchorA, bA->m_sweep.localCenter);
    b2RotMulVec2(rA, bA->m_xf[1], v);
    b2Vec2Sub(v, p->m_localAnchorB, bB->m_sweep.localCenter);
    b2RotMulVec2(rB, bB->m_xf[1], v);
    b2Vec2Add(p1, bA->m_sweep.c, rA);
    b2Vec2Add(p2, bB->m_sweep.c, rB);
    b2Vec2Sub(d, p2, p1);
    b2RotMulVec2(axis, bA->m_xf[1], p->m_localXAxisA);

    b2Vec2Assign(vA, bA->m_linearVelocity);
    b2Vec2Assign(vB, bB->m_linearVelocity);
    wA = bA->m_angularVelocity;
    wB = bB->m_angularVelocity;

    b2Vec2CrossProductKL(v1, wA, axis);
    b2Vec2CrossProductKL(v2, wB, rB);
    b2Vec2CrossProductKL(v3, wA, rA);
    b2Vec2Add(v2, vB, v2);
    b2Vec2Add(v3, vA, v3);
    b2Vec2Sub(v2, v2, v3);
    speed = b2Vec2DotProduct(d, v1) + b2Vec2DotProduct(axis, v2);
    return speed;
}

B2_API
void
b2JointPrismaticEnableLimit(
    struct b2JointPrismatic* p,
    int flag)
{
    if (flag != p->m_enableLimit)
    {
        b2BodySetAwake(p->m_bodyA, b2True);
        b2BodySetAwake(p->m_bodyB, b2True);
        p->m_enableLimit = flag;
        p->m_lowerImpulse = 0.0f;
        p->m_upperImpulse = 0.0f;
    }
}

B2_API
void
b2JointPrismaticSetLimits(
    struct b2JointPrismatic* p,
    float lower,
    float upper)
{
    b2Assert(lower <= upper);
    if (lower != p->m_lowerTranslation || 
        upper != p->m_upperTranslation)
    {
        b2BodySetAwake(p->m_bodyA, b2True);
        b2BodySetAwake(p->m_bodyB, b2True);
        p->m_lowerTranslation = lower;
        p->m_upperTranslation = upper;
        p->m_lowerImpulse = 0.0f;
        p->m_upperImpulse = 0.0f;
    }
}

B2_API
void
b2JointPrismaticEnableMotor(
    struct b2JointPrismatic* p,
    int flag)
{
    if (flag != p->m_enableMotor)
    {
        b2BodySetAwake(p->m_bodyA, b2True);
        b2BodySetAwake(p->m_bodyB, b2True);
        p->m_enableMotor = flag;
    }
}

B2_API
void
b2JointPrismaticSetMotorSpeed(
    struct b2JointPrismatic* p,
    float speed)
{
    if (speed != p->m_motorSpeed)
    {
        b2BodySetAwake(p->m_bodyA, b2True);
        b2BodySetAwake(p->m_bodyB, b2True);
        p->m_motorSpeed = speed;
    }
}

B2_API
void
b2JointPrismaticSetMaxMotorForce(
    struct b2JointPrismatic* p,
    float force)
{
    if (force != p->m_maxMotorForce)
    {
        b2BodySetAwake(p->m_bodyA, b2True);
        b2BodySetAwake(p->m_bodyB, b2True);
        p->m_maxMotorForce = force;
    }
}
