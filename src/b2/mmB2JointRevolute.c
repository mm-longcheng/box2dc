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

#include "mmB2JointRevolute.h"
#include "mmB2Common.h"
#include "mmB2Body.h"
#include "mmB2Draw.h"
#include "mmB2TimeStep.h"

#include <assert.h>
#include <string.h>

B2_API const struct b2MetaAllocator b2MetaAllocatorJointRevolute =
{
    "b2JointRevolute",
    sizeof(struct b2JointRevolute),
    &b2JointRevolutePrepare,
    &b2JointRevoluteDiscard,
};

B2_API const struct b2JointMeta b2JointRevoluteMeta =
{
    &b2JointRevoluteGetAnchorA,
    &b2JointRevoluteGetAnchorB,
    &b2JointRevoluteGetReactionForce,
    &b2JointRevoluteGetReactionTorque,
    &b2JointRevoluteDump,
    &b2JointRevoluteShiftOrigin,
    &b2JointRevoluteDraw,
    &b2JointRevoluteInitVelocityConstraints,
    &b2JointRevoluteSolveVelocityConstraints,
    &b2JointRevoluteSolvePositionConstraints,
};

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

B2_API
void
b2JointRevoluteDefReset(
    struct b2JointRevoluteDef* p)
{
    b2JointDefReset((struct b2JointDef*)p);
    p->type = b2JointTypeRevolute;
    b2Vec2Make(p->localAnchorA, 0.0f, 0.0f);
    b2Vec2Make(p->localAnchorB, 0.0f, 0.0f);
    p->referenceAngle = 0.0f;
    p->lowerAngle = 0.0f;
    p->upperAngle = 0.0f;
    p->maxMotorTorque = 0.0f;
    p->motorSpeed = 0.0f;
    p->enableLimit = b2False;
    p->enableMotor = b2False;
}

B2_API
void
b2JointRevoluteDefInitialize(
    struct b2JointRevoluteDef* p,
    struct b2Body* bodyA,
    struct b2Body* bodyB,
    const b2Vec2 anchor)
{
    p->bodyA = bodyA;
    p->bodyB = bodyB;
    b2BodyGetLocalPoint(bodyA, anchor, p->localAnchorA);
    b2BodyGetLocalPoint(bodyB, anchor, p->localAnchorB);
    p->referenceAngle = b2BodyGetAngle(bodyB) - b2BodyGetAngle(bodyA);
}

B2_API
void
b2JointRevolutePrepare(
    struct b2JointRevolute* p,
    const struct b2JointRevoluteDef* def)
{
    b2JointFromDef((struct b2Joint*)p, (struct b2JointDef*)def);
    p->Meta = &b2JointRevoluteMeta;

    b2Vec2Assign(p->m_localAnchorA, def->localAnchorA);
    b2Vec2Assign(p->m_localAnchorB, def->localAnchorB);
    p->m_referenceAngle = def->referenceAngle;

    b2Vec2SetZero(p->m_impulse);
    p->m_axialMass = 0.0f;
    p->m_motorImpulse = 0.0f;
    p->m_lowerImpulse = 0.0f;
    p->m_upperImpulse = 0.0f;

    p->m_lowerAngle = def->lowerAngle;
    p->m_upperAngle = def->upperAngle;
    p->m_maxMotorTorque = def->maxMotorTorque;
    p->m_motorSpeed = def->motorSpeed;
    p->m_enableLimit = def->enableLimit;
    p->m_enableMotor = def->enableMotor;

    p->m_angle = 0.0f;
}

B2_API
void
b2JointRevoluteDiscard(
    struct b2JointRevolute* p)
{
    memset(p, 0, sizeof(struct b2JointRevolute));
}

B2_API
void
b2JointRevoluteGetAnchorA(
    const struct b2JointRevolute* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyA, p->m_localAnchorA, anchor);
}

B2_API
void
b2JointRevoluteGetAnchorB(
    const struct b2JointRevolute* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyB, p->m_localAnchorB, anchor);
}

B2_API
void
b2JointRevoluteGetReactionForce(
    const struct b2JointRevolute* p,
    float inv_dt,
    b2Vec2 force)
{
    b2Vec2 P;
    P[0] = p->m_impulse[0];
    P[1] = p->m_impulse[1];
    b2Vec2Scale(force, P, inv_dt);
}

/// Get the reaction torque given the inverse time step.
/// Unit is N*m. This is always zero for a distance joint.
B2_API
float
b2JointRevoluteGetReactionTorque(
    const struct b2JointRevolute* p,
    float inv_dt)
{
    return inv_dt * (p->m_motorImpulse + p->m_lowerImpulse - p->m_upperImpulse);
}

/// Dump joint to dmLog
B2_API
void
b2JointRevoluteDump(
    const struct b2JointRevolute* p)
{
    int32 indexA = p->m_bodyA->m_islandIndex;
    int32 indexB = p->m_bodyB->m_islandIndex;

    b2Dump("  b2RevoluteJointDef jd;\n");
    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
    b2Dump("  jd.collideConnected = bool(%d);\n", p->m_collideConnected);
    b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", p->m_localAnchorA[0], p->m_localAnchorA[1]);
    b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", p->m_localAnchorB[0], p->m_localAnchorB[1]);
    b2Dump("  jd.referenceAngle = %.9g;\n", p->m_referenceAngle);
    b2Dump("  jd.enableLimit = bool(%d);\n", p->m_enableLimit);
    b2Dump("  jd.lowerAngle = %.9g;\n", p->m_lowerAngle);
    b2Dump("  jd.upperAngle = %.9g;\n", p->m_upperAngle);
    b2Dump("  jd.enableMotor = bool(%d);\n", p->m_enableMotor);
    b2Dump("  jd.motorSpeed = %.9g;\n", p->m_motorSpeed);
    b2Dump("  jd.maxMotorTorque = %.9g;\n", p->m_maxMotorTorque);
    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", p->m_index);
}

B2_API
void
b2JointRevoluteShiftOrigin(
    struct b2JointRevolute* p,
    const b2Vec2 newOrigin)
{
    B2_NOT_USED(newOrigin);
}

B2_API
void
b2JointRevoluteDraw(
    const struct b2JointRevolute* p,
    struct b2Draw* draw)
{
    static const b2Color c1 = { 0.7f, 0.7f, 0.7f, 1.0f };
    static const b2Color c2 = { 0.3f, 0.9f, 0.3f, 1.0f };
    static const b2Color c3 = { 0.9f, 0.3f, 0.3f, 1.0f };
    static const b2Color c4 = { 0.3f, 0.3f, 0.9f, 1.0f };
    static const b2Color c5 = { 0.4f, 0.4f, 0.4f, 1.0f };

    static const b2Color c6 = { 0.5f, 0.8f, 0.8f, 1.0f };

    b2Vec2 v;

    b2TransformConstRef xfA;
    b2TransformConstRef xfB;
    b2Vec2 pA;
    b2Vec2 pB;

    float aA;
    float aB;
    float angle;

    b2Vec2 r;

    const float L = 0.5f;

    xfA = b2BodyGetTransform(p->m_bodyA);
    xfB = b2BodyGetTransform(p->m_bodyB);
    b2TransformMulVec2(pA, xfA, p->m_localAnchorA);
    b2TransformMulVec2(pB, xfB, p->m_localAnchorB);

	b2DrawPoint(draw, pA, 5.0f, c4);
	b2DrawPoint(draw, pB, 5.0f, c5);

	aA = b2BodyGetAngle(p->m_bodyA);
	aB = b2BodyGetAngle(p->m_bodyB);
	angle = aB - aA - p->m_referenceAngle;

    r[0] = L * cosf(angle);
    r[1] = L * sinf(angle);
    b2Vec2Add(v, pB, r);
	b2DrawSegment(draw, pB, v, c1);
	b2DrawCircle(draw, pB, L, c1);

	if (p->m_enableLimit)
	{
        b2Vec2 rlo;
        b2Vec2 rhi;

        rlo[0] = L * cosf(p->m_lowerAngle);
        rlo[1] = L * sinf(p->m_lowerAngle);
        rhi[0] = L * cosf(p->m_upperAngle);
        rhi[1] = L * sinf(p->m_upperAngle);

        b2Vec2Add(v, pB, rlo);
		b2DrawSegment(draw, pB, v, c2);
        b2Vec2Add(v, pB, rhi);
		b2DrawSegment(draw, pB, v, c3);
	}

	b2DrawSegment(draw, xfA[0], pA, c6);
	b2DrawSegment(draw, pA, pB, c6);
	b2DrawSegment(draw, xfB[0], pB, c6);
}

B2_API
void
b2JointRevoluteInitVelocityConstraints(
    struct b2JointRevolute* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;

    float aA;
    b2Vec2 vA;
    float wA;

    float aB;
    b2Vec2 vB;
    float wB;

    b2Rot qA, qB;

    float mA, mB;
    float iA, iB;

    int fixedRotation;

    p->m_indexA = p->m_bodyA->m_islandIndex;
    p->m_indexB = p->m_bodyB->m_islandIndex;
    b2Vec2Assign(p->m_localCenterA, p->m_bodyA->m_sweep.localCenter);
    b2Vec2Assign(p->m_localCenterB, p->m_bodyB->m_sweep.localCenter);
    p->m_invMassA = p->m_bodyA->m_invMass;
    p->m_invMassB = p->m_bodyB->m_invMass;
    p->m_invIA = p->m_bodyA->m_invI;
    p->m_invIB = p->m_bodyB->m_invI;

    aA = data->positions[p->m_indexA].a;
    b2Vec2Assign(vA, data->velocities[p->m_indexA].v);
    wA = data->velocities[p->m_indexA].w;

    aB = data->positions[p->m_indexB].a;
    b2Vec2Assign(vB, data->velocities[p->m_indexB].v);
    wB = data->velocities[p->m_indexB].w;

    b2RotFromAngle(qA, aA);
    b2RotFromAngle(qB, aB);

    b2Vec2Sub(v, p->m_localAnchorA, p->m_localCenterA);
    b2RotMulVec2(p->m_rA, qA, v);
    b2Vec2Sub(v, p->m_localAnchorB, p->m_localCenterB);
    b2RotMulVec2(p->m_rB, qB, v);

    // J = [-I -r1_skew I r2_skew]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
    //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

    mA = p->m_invMassA; mB = p->m_invMassB;
    iA = p->m_invIA   ; iB = p->m_invIB;

    p->m_K[0][0] = mA + mB + p->m_rA[1] * p->m_rA[1] * iA + p->m_rB[1] * p->m_rB[1] * iB;
    p->m_K[1][0] = -p->m_rA[1] * p->m_rA[0] * iA - p->m_rB[1] * p->m_rB[0] * iB;
    p->m_K[0][1] = p->m_K[1][0];
    p->m_K[1][1] = mA + mB + p->m_rA[0] * p->m_rA[0] * iA + p->m_rB[0] * p->m_rB[0] * iB;

    p->m_axialMass = iA + iB;
    if (p->m_axialMass > 0.0f)
    {
        p->m_axialMass = 1.0f / p->m_axialMass;
        fixedRotation = b2False;
    }
    else
    {
        fixedRotation = b2True;
    }

    p->m_angle = aB - aA - p->m_referenceAngle;
    if (p->m_enableLimit == b2False || fixedRotation)
    {
        p->m_lowerImpulse = 0.0f;
        p->m_upperImpulse = 0.0f;
    }

    if (p->m_enableMotor == b2False || fixedRotation)
    {
        p->m_motorImpulse = 0.0f;
    }

    if (data->step.warmStarting)
    {
        float axialImpulse;
        b2Vec2 P;

        // Scale impulses to support a variable time step.
        b2Vec2Scale(p->m_impulse, p->m_impulse, data->step.dtRatio);
        p->m_motorImpulse *= data->step.dtRatio;
        p->m_lowerImpulse *= data->step.dtRatio;
        p->m_upperImpulse *= data->step.dtRatio;

        axialImpulse = p->m_motorImpulse + p->m_lowerImpulse - p->m_upperImpulse;
        P[0] = p->m_impulse[0];
        P[1] = p->m_impulse[1];

        b2Vec2Scale(v, P, mA);
        b2Vec2Sub(vA, vA, v);
        wA -= iA * (b2Vec2CrossProduct(p->m_rA, P) + axialImpulse);

        b2Vec2Scale(v, P, mB);
        b2Vec2Add(vB, vB, v);
        wB += iB * (b2Vec2CrossProduct(p->m_rB, P) + axialImpulse);
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
b2JointRevoluteSolveVelocityConstraints(
    struct b2JointRevolute* p,
    const struct b2SolverData* data)
{
    b2Vec2 vA;
    float wA;
    b2Vec2 vB;
    float wB;

    float mA, mB;
    float iA, iB ;

    int fixedRotation;

    b2Vec2Assign(vA, data->velocities[p->m_indexA].v);
    wA = data->velocities[p->m_indexA].w;
    b2Vec2Assign(vB, data->velocities[p->m_indexB].v);
    wB = data->velocities[p->m_indexB].w;

    mA = p->m_invMassA; mB = p->m_invMassB;
    iA = p->m_invIA   ; iB = p->m_invIB;

    fixedRotation = (iA + iB == 0.0f);

    // Solve motor constraint.
    if (p->m_enableMotor && fixedRotation == b2False)
    {
        float Cdot;
        float impulse;
        float oldImpulse;
        float maxImpulse;

        Cdot = wB - wA - p->m_motorSpeed;
        impulse = -p->m_axialMass * Cdot;
        oldImpulse = p->m_motorImpulse;
        maxImpulse = data->step.dt * p->m_maxMotorTorque;
        p->m_motorImpulse = b2ClampFloat(p->m_motorImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = p->m_motorImpulse - oldImpulse;

        wA -= iA * impulse;
        wB += iB * impulse;
    }

    if (p->m_enableLimit && fixedRotation == b2False)
    {
        // Lower limit
        {
            float C;
            float Cdot;
            float impulse;
            float oldImpulse;

            C = p->m_angle - p->m_lowerAngle;
            Cdot = wB - wA;
            impulse = -p->m_axialMass * (Cdot + b2MaxFloat(C, 0.0f) * data->step.inv_dt);
            oldImpulse = p->m_lowerImpulse;
            p->m_lowerImpulse = b2MaxFloat(p->m_lowerImpulse + impulse, 0.0f);
            impulse = p->m_lowerImpulse - oldImpulse;

            wA -= iA * impulse;
            wB += iB * impulse;
        }

        // Upper limit
        // Note: signs are flipped to keep C positive when the constraint is satisfied.
        // This also keeps the impulse positive when the limit is active.
        {
            float C;
            float Cdot;
            float impulse;
            float oldImpulse;

            C = p->m_upperAngle - p->m_angle;
            Cdot = wA - wB;
            impulse = -p->m_axialMass * (Cdot + b2MaxFloat(C, 0.0f) * data->step.inv_dt);
            oldImpulse = p->m_upperImpulse;
            p->m_upperImpulse = b2MaxFloat(p->m_upperImpulse + impulse, 0.0f);
            impulse = p->m_upperImpulse - oldImpulse;

            wA += iA * impulse;
            wB -= iB * impulse;
        }
    }

    // Solve point-to-point constraint
    {
        b2Vec2 v;
        b2Vec2 v1, v2;

        b2Vec2 Cdot;
        b2Vec2 impulse;

        b2Vec2CrossProductKL(v1, wB, p->m_rB);
        b2Vec2CrossProductKL(v2, wA, p->m_rA);
        b2Vec2Add(v, vB, v1);
        b2Vec2Sub(v, v, vA);
        b2Vec2Sub(Cdot, v, v2);
        b2Vec2Negate(v, Cdot);
        b2Mat22Solve(impulse, p->m_K, v);

        p->m_impulse[0] += impulse[0];
        p->m_impulse[1] += impulse[1];

        b2Vec2Scale(v, impulse, mA);
        b2Vec2Sub(vA, vA, v);
        wA -= iA * b2Vec2CrossProduct(p->m_rA, impulse);

        b2Vec2Scale(v, impulse, mB);
        b2Vec2Add(vB, vB, v);
        wB += iB * b2Vec2CrossProduct(p->m_rB, impulse);
    }

    b2Vec2Assign(data->velocities[p->m_indexA].v, vA);
    data->velocities[p->m_indexA].w = wA;
    b2Vec2Assign(data->velocities[p->m_indexB].v, vB);
    data->velocities[p->m_indexB].w = wB;
}

B2_API
int
b2JointRevoluteSolvePositionConstraints(
    struct b2JointRevolute* p,
    const struct b2SolverData* data)
{
    b2Vec2 cA;
    float aA;
    b2Vec2 cB;
    float aB;

    b2Rot qA, qB;

    float angularError;
    float positionError;

    int fixedRotation;

    b2Vec2Assign(cA, data->positions[p->m_indexA].c);
    aA = data->positions[p->m_indexA].a;
    b2Vec2Assign(cB, data->positions[p->m_indexB].c);
    aB = data->positions[p->m_indexB].a;

    b2RotFromAngle(qA, aA);
    b2RotFromAngle(qB, aB);

    angularError = 0.0f;
    positionError = 0.0f;

    fixedRotation = (p->m_invIA + p->m_invIB == 0.0f);

    // Solve angular limit constraint
    if (p->m_enableLimit && fixedRotation == b2False)
    {
        float angle;
        float C;

        float limitImpulse;

        angle = aB - aA - p->m_referenceAngle;
        C = 0.0f;

        if (b2AbsFloat(p->m_upperAngle - p->m_lowerAngle) < 2.0f * b2_angularSlop)
        {
            // Prevent large angular corrections
            C = b2ClampFloat(angle - p->m_lowerAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
        }
        else if (angle <= p->m_lowerAngle)
        {
            // Prevent large angular corrections and allow some slop.
            C = b2ClampFloat(angle - p->m_lowerAngle + b2_angularSlop, -b2_maxAngularCorrection, 0.0f);
        }
        else if (angle >= p->m_upperAngle)
        {
            // Prevent large angular corrections and allow some slop.
            C = b2ClampFloat(angle - p->m_upperAngle - b2_angularSlop, 0.0f, b2_maxAngularCorrection);
        }

        limitImpulse = -p->m_axialMass * C;
        aA -= p->m_invIA * limitImpulse;
        aB += p->m_invIB * limitImpulse;
        angularError = b2AbsFloat(C);
    }

    // Solve point-to-point constraint.
    {
        b2Vec2 v;

        b2Vec2 rA;
        b2Vec2 rB;

        b2Vec2 C;

        float mA, mB;
        float iA, iB;

        b2Mat22 K;

        b2Vec2 impulse;

        b2RotFromAngle(qA, aA);
        b2RotFromAngle(qB, aB);

        b2Vec2Sub(v, p->m_localAnchorA, p->m_localCenterA);
        b2RotMulVec2(rA, qA, v);
        b2Vec2Sub(v, p->m_localAnchorB, p->m_localCenterB);
        b2RotMulVec2(rB, qB, v);

        b2Vec2Add(v, cB, rB);
        b2Vec2Sub(v, v, cA);
        b2Vec2Sub(C, v, rA);
        positionError = b2Vec2Length(C);

        mA = p->m_invMassA; mB = p->m_invMassB;
        iA = p->m_invIA   ; iB = p->m_invIB;

        K[0][0] = mA + mB + iA * rA[1] * rA[1] + iB * rB[1] * rB[1];
        K[0][1] = -iA * rA[0] * rA[1] - iB * rB[0] * rB[1];
        K[1][0] = K[0][1];
        K[1][1] = mA + mB + iA * rA[0] * rA[0] + iB * rB[0] * rB[0];

        b2Mat22Solve(impulse, K, C);
        b2Vec2Negate(impulse, impulse);

        b2Vec2Scale(v, impulse, mA);
        b2Vec2Sub(cA, cA, v);
        aA -= iA * b2Vec2CrossProduct(rA, impulse);

        b2Vec2Scale(v, impulse, mB);
        b2Vec2Add(cB, cB, v);
        aB += iB * b2Vec2CrossProduct(rB, impulse);
    }

    b2Vec2Assign(data->positions[p->m_indexA].c, cA);
    data->positions[p->m_indexA].a = aA;
    b2Vec2Assign(data->positions[p->m_indexB].c, cB);
    data->positions[p->m_indexB].a = aB;

    return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
}

B2_API
float
b2JointRevoluteGetJointAngle(
    const struct b2JointRevolute* p)
{
    struct b2Body* bA = p->m_bodyA;
    struct b2Body* bB = p->m_bodyB;
    return bB->m_sweep.a - bA->m_sweep.a - p->m_referenceAngle;
}

B2_API
float
b2JointRevoluteGetJointSpeed(
    const struct b2JointRevolute* p)
{

    struct b2Body* bA = p->m_bodyA;
    struct b2Body* bB = p->m_bodyB;
    return bB->m_angularVelocity - bA->m_angularVelocity;
}

B2_API
void
b2JointRevoluteEnableLimit(
    struct b2JointRevolute* p,
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
b2JointRevoluteSetLimits(
    struct b2JointRevolute* p,
    float lower,
    float upper)
{
    b2Assert(lower <= upper);

    if (lower != p->m_lowerAngle || upper != p->m_upperAngle)
    {
        b2BodySetAwake(p->m_bodyA, b2True);
        b2BodySetAwake(p->m_bodyB, b2True);
        p->m_lowerImpulse = 0.0f;
        p->m_upperImpulse = 0.0f;
        p->m_lowerAngle = lower;
        p->m_upperAngle = upper;
    }
}

B2_API
void
b2JointRevoluteEnableMotor(
    struct b2JointRevolute* p,
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
b2JointRevoluteSetMotorSpeed(
    struct b2JointRevolute* p,
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
b2JointRevoluteSetMaxMotorTorque(
    struct b2JointRevolute* p,
    float torque)
{
    if (torque != p->m_maxMotorTorque)
    {
        b2BodySetAwake(p->m_bodyA, b2True);
        b2BodySetAwake(p->m_bodyB, b2True);
        p->m_maxMotorTorque = torque;
    }
}
