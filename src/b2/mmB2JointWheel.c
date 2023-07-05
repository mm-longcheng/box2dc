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

#include "mmB2JointWheel.h"
#include "mmB2Common.h"
#include "mmB2Body.h"
#include "mmB2Draw.h"
#include "mmB2TimeStep.h"

#include <assert.h>
#include <string.h>

B2_API const struct b2MetaAllocator b2MetaAllocatorJointWheel =
{
    "b2JointWheel",
    sizeof(struct b2JointWheel),
    &b2JointWheelPrepare,
    &b2JointWheelDiscard,
};

B2_API const struct b2JointMeta b2JointWheelMeta =
{
    &b2JointWheelGetAnchorA,
    &b2JointWheelGetAnchorB,
    &b2JointWheelGetReactionForce,
    &b2JointWheelGetReactionTorque,
    &b2JointWheelDump,
    &b2JointWheelShiftOrigin,
    &b2JointWheelDraw,
    &b2JointWheelInitVelocityConstraints,
    &b2JointWheelSolveVelocityConstraints,
    &b2JointWheelSolvePositionConstraints,
};

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

B2_API
void
b2JointWheelDefReset(
    struct b2JointWheelDef* p)
{
    b2JointDefReset((struct b2JointDef*)p);
    p->type = b2JointTypeWheel;
    b2Vec2SetZero(p->localAnchorA);
    b2Vec2SetZero(p->localAnchorB);
    b2Vec2Make(p->localAxisA, 1.0f, 0.0f);
    p->enableLimit = b2False;
    p->lowerTranslation = 0.0f;
    p->upperTranslation = 0.0f;
    p->enableMotor = b2False;
    p->maxMotorTorque = 0.0f;
    p->motorSpeed = 0.0f;
    p->stiffness = 0.0f;
    p->damping = 0.0f;
}

B2_API
void
b2JointWheelDefInitialize(
    struct b2JointWheelDef* p,
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
}

B2_API
void
b2JointWheelPrepare(
    struct b2JointWheel* p,
    const struct b2JointWheelDef* def)
{
    b2JointFromDef((struct b2Joint*)p, (struct b2JointDef*)def);
    p->Meta = &b2JointWheelMeta;

    b2Vec2Assign(p->m_localAnchorA, def->localAnchorA);
    b2Vec2Assign(p->m_localAnchorB, def->localAnchorB);
    b2Vec2Assign(p->m_localXAxisA, def->localAxisA);
    b2Vec2CrossProductKL(p->m_localYAxisA, 1.0f, p->m_localXAxisA);

    p->m_mass = 0.0f;
    p->m_impulse = 0.0f;
    p->m_motorMass = 0.0f;
    p->m_motorImpulse = 0.0f;
    p->m_springMass = 0.0f;
    p->m_springImpulse = 0.0f;

    p->m_axialMass = 0.0f;
    p->m_lowerImpulse = 0.0f;
    p->m_upperImpulse = 0.0f;
    p->m_lowerTranslation = def->lowerTranslation;
    p->m_upperTranslation = def->upperTranslation;
    p->m_enableLimit = def->enableLimit;

    p->m_maxMotorTorque = def->maxMotorTorque;
    p->m_motorSpeed = def->motorSpeed;
    p->m_enableMotor = def->enableMotor;

    p->m_bias = 0.0f;
    p->m_gamma = 0.0f;

    b2Vec2SetZero(p->m_ax);
    b2Vec2SetZero(p->m_ay);

    p->m_stiffness = def->stiffness;
    p->m_damping = def->damping;
}

B2_API
void
b2JointWheelDiscard(
    struct b2JointWheel* p)
{
    memset(p, 0, sizeof(struct b2JointWheel));
}

B2_API
void
b2JointWheelGetAnchorA(
    const struct b2JointWheel* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyA, p->m_localAnchorA, anchor);
}

B2_API
void
b2JointWheelGetAnchorB(
    const struct b2JointWheel* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyB, p->m_localAnchorB, anchor);
}

B2_API
void
b2JointWheelGetReactionForce(
    const struct b2JointWheel* p,
    float inv_dt,
    b2Vec2 force)
{
    b2Vec2 v1, v2;
    b2Vec2Scale(v1, p->m_ay, p->m_impulse);
    b2Vec2Scale(v2, p->m_ax, p->m_springImpulse + p->m_lowerImpulse - p->m_upperImpulse);
    b2Vec2Add(force, v1, v2);
    b2Vec2Scale(force, force, inv_dt);
}

B2_API
float
b2JointWheelGetReactionTorque(
    const struct b2JointWheel* p,
    float inv_dt)
{
    return inv_dt * p->m_motorImpulse;
}

B2_API
void
b2JointWheelDump(
    const struct b2JointWheel* p)
{
    // FLT_DECIMAL_DIG == 9

    int32 indexA = p->m_bodyA->m_islandIndex;
    int32 indexB = p->m_bodyB->m_islandIndex;

    b2Dump("  b2WheelJointDef jd;\n");
    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
    b2Dump("  jd.collideConnected = bool(%d);\n", p->m_collideConnected);
    b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", p->m_localAnchorA[0], p->m_localAnchorA[1]);
    b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", p->m_localAnchorB[0], p->m_localAnchorB[1]);
    b2Dump("  jd.localAxisA.Set(%.9g, %.9g);\n", p->m_localXAxisA[0], p->m_localXAxisA[1]);
    b2Dump("  jd.enableMotor = bool(%d);\n", p->m_enableMotor);
    b2Dump("  jd.motorSpeed = %.9g;\n", p->m_motorSpeed);
    b2Dump("  jd.maxMotorTorque = %.9g;\n", p->m_maxMotorTorque);
    b2Dump("  jd.stiffness = %.9g;\n", p->m_stiffness);
    b2Dump("  jd.damping = %.9g;\n", p->m_damping);
    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", p->m_index);
}

B2_API
void
b2JointWheelShiftOrigin(
    struct b2JointWheel* p,
    const b2Vec2 newOrigin)
{
    B2_NOT_USED(newOrigin);
}

B2_API
void
b2JointWheelDraw(
    const struct b2JointWheel* p,
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
b2JointWheelInitVelocityConstraints(
    struct b2JointWheel* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;
    b2Vec2 v1, v2;

    float mA, mB;
    float iA, iB;

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

    float invMass;

    p->m_indexA = p->m_bodyA->m_islandIndex;
    p->m_indexB = p->m_bodyB->m_islandIndex;
    b2Vec2Assign(p->m_localCenterA, p->m_bodyA->m_sweep.localCenter);
    b2Vec2Assign(p->m_localCenterB, p->m_bodyB->m_sweep.localCenter);
    p->m_invMassA = p->m_bodyA->m_invMass;
    p->m_invMassB = p->m_bodyB->m_invMass;
    p->m_invIA = p->m_bodyA->m_invI;
    p->m_invIB = p->m_bodyB->m_invI;

    mA = p->m_invMassA; mB = p->m_invMassB;
    iA = p->m_invIA   ; iB = p->m_invIB;

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
    b2Vec2Add(v, cB, rB);
    b2Vec2Sub(v, v, cA);
    b2Vec2Sub(d, v, rA);

    // Point to line constraint
    {
        b2RotMulVec2(p->m_ay, qA, p->m_localYAxisA);
        b2Vec2Add(v, d, rA);
        p->m_sAy = b2Vec2CrossProduct(v, p->m_ay);
        p->m_sBy = b2Vec2CrossProduct(rB, p->m_ay);

        p->m_mass = mA + mB + iA * p->m_sAy * p->m_sAy + iB * p->m_sBy * p->m_sBy;

        if (p->m_mass > 0.0f)
        {
            p->m_mass = 1.0f / p->m_mass;
        }
    }

    // Spring constraint
    b2RotMulVec2(p->m_ax, qA, p->m_localXAxisA);
    //p->m_ax = b2Mul(qA, p->m_localXAxisA);
    b2Vec2Add(v, d, rA);
    p->m_sAx = b2Vec2CrossProduct(v, p->m_ax);
    p->m_sBx = b2Vec2CrossProduct(rB, p->m_ax);

    invMass = mA + mB + iA * p->m_sAx * p->m_sAx + iB * p->m_sBx * p->m_sBx;
    if (invMass > 0.0f)
    {
        p->m_axialMass = 1.0f / invMass;
    }
    else
    {
        p->m_axialMass = 0.0f;
    }

    p->m_springMass = 0.0f;
    p->m_bias = 0.0f;
    p->m_gamma = 0.0f;

    if (p->m_stiffness > 0.0f && invMass > 0.0f)
    {
        float C;
        float h;

        p->m_springMass = 1.0f / invMass;

        C = b2Vec2DotProduct(d, p->m_ax);

        // magic formulas
        h = data->step.dt;
        p->m_gamma = h * (p->m_damping + h * p->m_stiffness);
        if (p->m_gamma > 0.0f)
        {
            p->m_gamma = 1.0f / p->m_gamma;
        }

        p->m_bias = C * h * p->m_stiffness * p->m_gamma;

        p->m_springMass = invMass + p->m_gamma;
        if (p->m_springMass > 0.0f)
        {
            p->m_springMass = 1.0f / p->m_springMass;
        }
    }
    else
    {
        p->m_springImpulse = 0.0f;
    }

    if (p->m_enableLimit)
    {
        p->m_translation = b2Vec2DotProduct(p->m_ax, d);
    }
    else
    {
        p->m_lowerImpulse = 0.0f;
        p->m_upperImpulse = 0.0f;
    }

    if (p->m_enableMotor)
    {
        p->m_motorMass = iA + iB;
        if (p->m_motorMass > 0.0f)
        {
            p->m_motorMass = 1.0f / p->m_motorMass;
        }
    }
    else
    {
        p->m_motorMass = 0.0f;
        p->m_motorImpulse = 0.0f;
    }

    if (data->step.warmStarting)
    {
        float axialImpulse;
        b2Vec2 P;
        float LA;
        float LB;

        // Account for variable time step.
        p->m_impulse *= data->step.dtRatio;
        p->m_springImpulse *= data->step.dtRatio;
        p->m_motorImpulse *= data->step.dtRatio;

        axialImpulse = p->m_springImpulse + p->m_lowerImpulse - p->m_upperImpulse;
        b2Vec2Scale(v1, p->m_ay, p->m_impulse);
        b2Vec2Scale(v2, p->m_ax, axialImpulse);
        b2Vec2Add(P, v1, v2);
        LA = p->m_impulse * p->m_sAy + axialImpulse * p->m_sAx + p->m_motorImpulse;
        LB = p->m_impulse * p->m_sBy + axialImpulse * p->m_sBx + p->m_motorImpulse;

        b2Vec2Scale(v, P, p->m_invMassA);
        b2Vec2Sub(vA, vA, v);
        wA -= p->m_invIA * LA;

        b2Vec2Scale(v, P, p->m_invMassB);
        b2Vec2Add(vB, vB, v);
        wB += p->m_invIB * LB;
    }
    else
    {
        p->m_impulse = 0.0f;
        p->m_springImpulse = 0.0f;
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
b2JointWheelSolveVelocityConstraints(
    struct b2JointWheel* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;

    float mA, mB;
    float iA, iB;

    b2Vec2 vA;
    float wA;
    b2Vec2 vB;
    float wB;


    mA = p->m_invMassA; mB = p->m_invMassB;
    iA = p->m_invIA   ; iB = p->m_invIB;

    b2Vec2Assign(vA, data->velocities[p->m_indexA].v);
    wA = data->velocities[p->m_indexA].w;
    b2Vec2Assign(vB, data->velocities[p->m_indexB].v);
    wB = data->velocities[p->m_indexB].w;

    // Solve spring constraint
    {
        float Cdot;
        float impulse;

        b2Vec2 P;
        float LA;
        float LB;

        b2Vec2Sub(v, vB, vA);
        Cdot = b2Vec2DotProduct(p->m_ax, v) + p->m_sBx * wB - p->m_sAx * wA;
        impulse = -p->m_springMass * (Cdot + p->m_bias + p->m_gamma * p->m_springImpulse);
        p->m_springImpulse += impulse;

        b2Vec2Scale(P, p->m_ax, impulse);
        LA = impulse * p->m_sAx;
        LB = impulse * p->m_sBx;

        b2Vec2Scale(v, P, mA);
        b2Vec2Sub(vA, vA, v);
        wA -= iA * LA;

        b2Vec2Scale(v, P, mB);
        b2Vec2Add(vB, vB, v);
        wB += iB * LB;
    }

    // Solve rotational motor constraint
    {
        float Cdot;
        float impulse;

        float oldImpulse;
        float maxImpulse;

        Cdot = wB - wA - p->m_motorSpeed;
        impulse = -p->m_motorMass * Cdot;

        oldImpulse = p->m_motorImpulse;
        maxImpulse = data->step.dt * p->m_maxMotorTorque;
        p->m_motorImpulse = b2ClampFloat(p->m_motorImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = p->m_motorImpulse - oldImpulse;

        wA -= iA * impulse;
        wB += iB * impulse;
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
            Cdot = b2Vec2DotProduct(p->m_ax, v) + p->m_sBx * wB - p->m_sAx * wA;
            impulse = -p->m_axialMass * (Cdot + b2MaxFloat(C, 0.0f) * data->step.inv_dt);
            oldImpulse = p->m_lowerImpulse;
            p->m_lowerImpulse = b2MaxFloat(p->m_lowerImpulse + impulse, 0.0f);
            impulse = p->m_lowerImpulse - oldImpulse;

            b2Vec2Scale(P, p->m_ax, impulse);
            LA = impulse * p->m_sAx;
            LB = impulse * p->m_sBx;

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
            Cdot = b2Vec2DotProduct(p->m_ax, v) + p->m_sAx * wA - p->m_sBx * wB;
            impulse = -p->m_axialMass * (Cdot + b2MaxFloat(C, 0.0f) * data->step.inv_dt);
            oldImpulse = p->m_upperImpulse;
            p->m_upperImpulse = b2MaxFloat(p->m_upperImpulse + impulse, 0.0f);
            impulse = p->m_upperImpulse - oldImpulse;

            b2Vec2Scale(P, p->m_ax, impulse);
            LA = impulse * p->m_sAx;
            LB = impulse * p->m_sBx;

            b2Vec2Scale(v, P, mA);
            b2Vec2Add(vA, vA, v);
            wA += iA * LA;
            b2Vec2Scale(v, P, mB);
            b2Vec2Sub(vB, vB, v);
            wB -= iB * LB;
        }
    }

    // Solve point to line constraint
    {
        float Cdot;
        float impulse;

        b2Vec2 P;
        float LA;
        float LB;

        b2Vec2Sub(v, vB, vA);
        Cdot = b2Vec2DotProduct(p->m_ay, v) + p->m_sBy * wB - p->m_sAy * wA;
        impulse = -p->m_mass * Cdot;
        p->m_impulse += impulse;

        b2Vec2Scale(P, p->m_ay, impulse);
        LA = impulse * p->m_sAy;
        LB = impulse * p->m_sBy;

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

B2_API
int
b2JointWheelSolvePositionConstraints(
    struct b2JointWheel* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;

    b2Vec2 cA;
    float aA;
    b2Vec2 cB;
    float aB;

    float linearError;

    b2Vec2Assign(cA, data->positions[p->m_indexA].c);
    aA = data->positions[p->m_indexA].a;
    b2Vec2Assign(cB, data->positions[p->m_indexB].c);
    aB = data->positions[p->m_indexB].a;

    linearError = 0.0f;

    if (p->m_enableLimit)
    {
        b2Rot qA, qB;

        b2Vec2 rA;
        b2Vec2 rB;
        b2Vec2 d;

        b2Vec2 ax;
        float sAx;
        float sBx;

        float C;
        float translation;

        b2RotFromAngle(qA, aA);
        b2RotFromAngle(qB, aB);

        b2Vec2Sub(v, p->m_localAnchorA, p->m_localCenterA);
        b2RotMulVec2(rA, qA, v);
        b2Vec2Sub(v, p->m_localAnchorB, p->m_localCenterB);
        b2RotMulVec2(rB, qB, v);
        b2Vec2Sub(v, cB, cA);
        b2Vec2Add(v, v, rB);
        b2Vec2Sub(d, v, rA);

        b2RotMulVec2(ax, qA, p->m_localXAxisA);
        b2Vec2Add(v, d, rA);
        sAx = b2Vec2CrossProduct(v, p->m_ax);
        sBx = b2Vec2CrossProduct(rB, p->m_ax);

        C = 0.0f;
        translation = b2Vec2DotProduct(ax, d);
        if (b2AbsFloat(p->m_upperTranslation - p->m_lowerTranslation) < 2.0f * b2_linearSlop)
        {
            C = translation;
        }
        else if (translation <= p->m_lowerTranslation)
        {
            C = b2MinFloat(translation - p->m_lowerTranslation, 0.0f);
        }
        else if (translation >= p->m_upperTranslation)
        {
            C = b2MaxFloat(translation - p->m_upperTranslation, 0.0f);
        }

        if (C != 0.0f)
        {
            float invMass;
            float impulse;

            b2Vec2 P;
            float LA;
            float LB;

            invMass = p->m_invMassA + p->m_invMassB + p->m_invIA * sAx * sAx + p->m_invIB * sBx * sBx;
            impulse = 0.0f;
            if (invMass != 0.0f)
            {
                impulse = -C / invMass;
            }

            b2Vec2Scale(P, ax, impulse);
            LA = impulse * sAx;
            LB = impulse * sBx;

            b2Vec2Scale(v, P, p->m_invMassA);
            b2Vec2Sub(cA, cA, v);
            aA -= p->m_invIA * LA;
            b2Vec2Scale(v, P, p->m_invMassB);
            b2Vec2Add(cB, cB, v);
            aB += p->m_invIB * LB;

            linearError = b2AbsFloat(C);
        }
    }

    // Solve perpendicular constraint
    {
        b2Rot qA, qB;

        b2Vec2 rA;
        b2Vec2 rB;
        b2Vec2 d;

        b2Vec2 ay;

        float sAy;
        float sBy;

        float C;

        float invMass;

        float impulse;

        b2Vec2 P;
        float LA;
        float LB;

        b2RotFromAngle(qA, aA);
        b2RotFromAngle(qB, aB);

        b2Vec2Sub(v, p->m_localAnchorA, p->m_localCenterA);
        b2RotMulVec2(rA, qA, v);
        b2Vec2Sub(v, p->m_localAnchorB, p->m_localCenterB);
        b2RotMulVec2(rB, qB, v);
        b2Vec2Sub(v, cB, cA);
        b2Vec2Add(v, v, rB);
        b2Vec2Sub(d, v, rA);

        b2RotMulVec2(ay, qA, p->m_localYAxisA);

        b2Vec2Sub(v, d, rA);
        sAy = b2Vec2CrossProduct(v, ay);
        sBy = b2Vec2CrossProduct(rB, ay);

        C = b2Vec2DotProduct(d, ay);

        invMass = p->m_invMassA + p->m_invMassB + p->m_invIA * p->m_sAy * p->m_sAy + p->m_invIB * p->m_sBy * p->m_sBy;

        impulse = 0.0f;
        if (invMass != 0.0f)
        {
            impulse = -C / invMass;
        }

        b2Vec2Scale(P, ay, impulse);
        LA = impulse * sAy;
        LB = impulse * sBy;

        b2Vec2Scale(v, P, p->m_invMassA);
        b2Vec2Sub(cA, cA, v);
        aA -= p->m_invIA * LA;
        b2Vec2Scale(v, P, p->m_invMassB);
        b2Vec2Add(cB, cB, v);
        aB += p->m_invIB * LB;

        linearError = b2MaxFloat(linearError, b2AbsFloat(C));
    }

    b2Vec2Assign(data->positions[p->m_indexA].c, cA);
    data->positions[p->m_indexA].a = aA;
    b2Vec2Assign(data->positions[p->m_indexB].c, cB);
    data->positions[p->m_indexB].a = aB;

    return linearError <= b2_linearSlop;
}

B2_API
float
b2JointWheelGetJointTranslation(
    const struct b2JointWheel* p)
{
    struct b2Body* bA;
    struct b2Body* bB;

    b2Vec2 pA;
    b2Vec2 pB;
    b2Vec2 d;
    b2Vec2 axis;

    float translation;

    bA = p->m_bodyA;
    bB = p->m_bodyB;

    b2BodyGetWorldPoint(bA, p->m_localAnchorA, pA);
    b2BodyGetWorldPoint(bB, p->m_localAnchorB, pB);
    b2Vec2Sub(d, pB, pA);
    b2BodyGetWorldVector(bA, p->m_localXAxisA, axis);

    translation = b2Vec2DotProduct(d, axis);
    return translation;
}

B2_API
float
b2JointWheelGetJointLinearSpeed(
    const struct b2JointWheel* p)
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
float
b2JointWheelGetJointAngle(
    const struct b2JointWheel* p)
{
    struct b2Body* bA = p->m_bodyA;
    struct b2Body* bB = p->m_bodyB;
    return bB->m_sweep.a - bA->m_sweep.a;
}

B2_API
float
b2JointWheelGetJointAngularSpeed(
    const struct b2JointWheel* p)
{
    float wA = p->m_bodyA->m_angularVelocity;
    float wB = p->m_bodyB->m_angularVelocity;
    return wB - wA;
}

B2_API
void
b2JointWheelEnableLimit(
    struct b2JointWheel* p,
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
b2JointWheelSetLimits(
    struct b2JointWheel* p,
    float lower, float upper)
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
b2JointWheelEnableMotor(
    struct b2JointWheel* p,
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
b2JointWheelSetMotorSpeed(
    struct b2JointWheel* p,
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
b2JointWheelSetMaxMotorTorque(
    struct b2JointWheel* p,
    float torque)
{
    if (torque != p->m_maxMotorTorque)
    {
        b2BodySetAwake(p->m_bodyA, b2True);
        b2BodySetAwake(p->m_bodyB, b2True);
        p->m_maxMotorTorque = torque;
    }
}
