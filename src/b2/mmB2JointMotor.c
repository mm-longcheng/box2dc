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

#include "mmB2JointMotor.h"
#include "mmB2Common.h"
#include "mmB2Body.h"
#include "mmB2Draw.h"
#include "mmB2TimeStep.h"

#include <assert.h>
#include <string.h>

B2_API const struct b2MetaAllocator b2MetaAllocatorJointMotor =
{
    "b2JointMotor",
    sizeof(struct b2JointMotor),
    &b2JointMotorPrepare,
    &b2JointMotorDiscard,
};

B2_API const struct b2JointMeta b2JointMotorMeta =
{
    &b2JointMotorGetAnchorA,
    &b2JointMotorGetAnchorB,
    &b2JointMotorGetReactionForce,
    &b2JointMotorGetReactionTorque,
    &b2JointMotorDump,
    &b2JointMotorShiftOrigin,
    &b2JointMotorDraw,
    &b2JointMotorInitVelocityConstraints,
    &b2JointMotorSolveVelocityConstraints,
    &b2JointMotorSolvePositionConstraints,
};

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)
//
// r1 = offset - c1
// r2 = -c2

// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

B2_API
void
b2JointMotorDefReset(
    struct b2JointMotorDef* p)
{
    b2JointDefReset((struct b2JointDef*)p);
    p->type = b2JointTypeMotor;
    b2Vec2SetZero(p->linearOffset);
    p->angularOffset = 0.0f;
    p->maxForce = 1.0f;
    p->maxTorque = 1.0f;
    p->correctionFactor = 0.3f;
}

B2_API
void
b2JointMotorDefInitialize(
    struct b2JointMotorDef* p,
    struct b2Body* bodyA,
    struct b2Body* bodyB)
{
    b2Vec2ConstRef xB;
    float angleA;
    float angleB;

    p->bodyA = bodyA;
    p->bodyB = bodyB;
    xB = b2BodyGetPosition(bodyB);
    b2BodyGetLocalPoint(bodyA, xB, p->linearOffset);

    angleA = b2BodyGetAngle(bodyA);
    angleB = b2BodyGetAngle(bodyB);
    p->angularOffset = angleB - angleA;
}

B2_API
void
b2JointMotorPrepare(
    struct b2JointMotor* p,
    const struct b2JointMotorDef* def)
{
    b2JointFromDef((struct b2Joint*)p, (struct b2JointDef*)def);
    p->Meta = &b2JointMotorMeta;

    b2Vec2Assign(p->m_linearOffset, def->linearOffset);
    p->m_angularOffset = def->angularOffset;

    b2Vec2SetZero(p->m_linearImpulse);
    p->m_angularImpulse = 0.0f;

    p->m_maxForce = def->maxForce;
    p->m_maxTorque = def->maxTorque;
    p->m_correctionFactor = def->correctionFactor;
}

B2_API
void
b2JointMotorDiscard(
    struct b2JointMotor* p)
{
    memset(p, 0, sizeof(struct b2JointMotor));
}

B2_API
void
b2JointMotorGetAnchorA(
    const struct b2JointMotor* p,
    b2Vec2 anchor)
{
    b2Vec2Assign(anchor, b2BodyGetPosition(p->m_bodyA));
}

B2_API
void
b2JointMotorGetAnchorB(
    const struct b2JointMotor* p,
    b2Vec2 anchor)
{
    b2Vec2Assign(anchor, b2BodyGetPosition(p->m_bodyB));
}

B2_API
void
b2JointMotorGetReactionForce(
    const struct b2JointMotor* p,
    float inv_dt,
    b2Vec2 force)
{
    b2Vec2Scale(force, p->m_linearImpulse, inv_dt);
}

B2_API
float
b2JointMotorGetReactionTorque(
    const struct b2JointMotor* p,
    float inv_dt)
{
    return inv_dt * p->m_angularImpulse;
}

B2_API
void
b2JointMotorDump(
    const struct b2JointMotor* p)
{
    int32 indexA = p->m_bodyA->m_islandIndex;
    int32 indexB = p->m_bodyB->m_islandIndex;

    b2Dump("  b2MotorJointDef jd;\n");
    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
    b2Dump("  jd.collideConnected = bool(%d);\n", p->m_collideConnected);
    b2Dump("  jd.linearOffset.Set(%.9g, %.9g);\n", p->m_linearOffset[0], p->m_linearOffset[1]);
    b2Dump("  jd.angularOffset = %.9g;\n", p->m_angularOffset);
    b2Dump("  jd.maxForce = %.9g;\n", p->m_maxForce);
    b2Dump("  jd.maxTorque = %.9g;\n", p->m_maxTorque);
    b2Dump("  jd.correctionFactor = %.9g;\n", p->m_correctionFactor);
    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", p->m_index);
}

B2_API
void
b2JointMotorShiftOrigin(
    struct b2JointMotor* p,
    const b2Vec2 newOrigin)
{
    B2_NOT_USED(newOrigin);
}

B2_API
void
b2JointMotorDraw(
    const struct b2JointMotor* p,
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
    b2JointMotorGetAnchorA(p, p1);
    b2JointMotorGetAnchorB(p, p2);

    b2DrawSegment(draw, x1, p1, color);
    b2DrawSegment(draw, p1, p2, color);
    b2DrawSegment(draw, x2, p2, color);
}

B2_API
void
b2JointMotorInitVelocityConstraints(
    struct b2JointMotor* p,
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

    float mA, mB;
    float iA, iB;

    b2Mat22 K;

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

    // Compute the effective mass matrix.
    b2Vec2Sub(v, p->m_linearOffset, p->m_localCenterA);
    b2RotMulVec2(p->m_rA, qA, v);
    b2Vec2Negate(v, p->m_localCenterB);
    b2RotMulVec2(p->m_rB, qB, v);

    // J = [-I -r1_skew I r2_skew]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
    //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
    //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

    mA = p->m_invMassA, mB = p->m_invMassB;
    iA = p->m_invIA   , iB = p->m_invIB;

    // Upper 2 by 2 of K for point to point
    K[0][0] = mA + mB + iA * p->m_rA[1] * p->m_rA[1] + iB * p->m_rB[1] * p->m_rB[1];
    K[0][1] = -iA * p->m_rA[0] * p->m_rA[1] - iB * p->m_rB[0] * p->m_rB[1];
    K[1][0] = K[0][1];
    K[1][1] = mA + mB + iA * p->m_rA[0] * p->m_rA[0] + iB * p->m_rB[0] * p->m_rB[0];

    b2Mat22GetInverse(p->m_linearMass, K);

    p->m_angularMass = iA + iB;
    if (p->m_angularMass > 0.0f)
    {
        p->m_angularMass = 1.0f / p->m_angularMass;
    }

    b2Vec2Add(v1, cB, p->m_rB);
    b2Vec2Add(v2, cA, p->m_rA);
    b2Vec2Sub(p->m_linearError, v1, v2);
    p->m_angularError = aB - aA - p->m_angularOffset;

    if (data->step.warmStarting)
    {
        b2Vec2 P;

        // Scale impulses to support a variable time step.
        b2Vec2Scale(p->m_linearImpulse, p->m_linearImpulse, data->step.dtRatio);
        p->m_angularImpulse *= data->step.dtRatio;

        P[0] = p->m_linearImpulse[0];
        P[1] = p->m_linearImpulse[1];

        b2Vec2Scale(v, P, mA);
        b2Vec2Sub(vA, vA, v);
        wA -= iA * (b2Vec2CrossProduct(p->m_rA, P) + p->m_angularImpulse);
        b2Vec2Scale(v, P, mB);
        b2Vec2Add(vB, vB, v);
        wB += iB * (b2Vec2CrossProduct(p->m_rB, P) + p->m_angularImpulse);
    }
    else
    {
        b2Vec2SetZero(p->m_linearImpulse);
        p->m_angularImpulse = 0.0f;
    }

    b2Vec2Assign(data->velocities[p->m_indexA].v, vA);
    data->velocities[p->m_indexA].w = wA;
    b2Vec2Assign(data->velocities[p->m_indexB].v, vB);
    data->velocities[p->m_indexB].w = wB;
}

B2_API
void
b2JointMotorSolveVelocityConstraints(
    struct b2JointMotor* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;
    b2Vec2 v1, v2, v3;

    b2Vec2 vA;
    float wA;
    b2Vec2 vB;
    float wB;

    float mA, mB;
    float iA, iB;

    float h;
    float inv_h;

    b2Vec2Assign(vA, data->velocities[p->m_indexA].v);
    wA = data->velocities[p->m_indexA].w;
    b2Vec2Assign(vB, data->velocities[p->m_indexB].v);
    wB = data->velocities[p->m_indexB].w;

    mA = p->m_invMassA, mB = p->m_invMassB;
    iA = p->m_invIA   , iB = p->m_invIB;

    h = data->step.dt;
    inv_h = data->step.inv_dt;

    // Solve angular friction
    {
        float Cdot;
        float impulse;

        float oldImpulse;
        float maxImpulse;

        Cdot = wB - wA + inv_h * p->m_correctionFactor * p->m_angularError;
        impulse = -p->m_angularMass * Cdot;

        oldImpulse = p->m_angularImpulse;
        maxImpulse = h * p->m_maxTorque;
        p->m_angularImpulse = b2ClampFloat(p->m_angularImpulse + impulse, -maxImpulse, maxImpulse);
        impulse = p->m_angularImpulse - oldImpulse;

        wA -= iA * impulse;
        wB += iB * impulse;
    }

    // Solve linear friction
    {
        b2Vec2 Cdot;

        b2Vec2 impulse;
        b2Vec2 oldImpulse;

        float maxImpulse;

        b2Vec2CrossProductKL(v1, wB, p->m_rB);
        b2Vec2CrossProductKL(v2, wA, p->m_rA);
        b2Vec2Scale(v3, p->m_linearError, inv_h * p->m_correctionFactor);
        b2Vec2Add(v1, vB, v1);
        b2Vec2Add(v2, vA, v2);
        b2Vec2Sub(v, v1, v2);
        b2Vec2Add(Cdot, v, v3);

        b2Mat22MulVec2(impulse, p->m_linearMass, Cdot);
        b2Vec2Negate(impulse, impulse);
        b2Vec2Assign(oldImpulse, p->m_linearImpulse);
        b2Vec2Add(p->m_linearImpulse, p->m_linearImpulse, impulse);

        maxImpulse = h * p->m_maxForce;

        if (b2Vec2SquaredLength(p->m_linearImpulse) > maxImpulse * maxImpulse)
        {
            b2Vec2Normalize(p->m_linearImpulse, p->m_linearImpulse);
            b2Vec2Scale(p->m_linearImpulse, p->m_linearImpulse, maxImpulse);
        }

        b2Vec2Sub(impulse, p->m_linearImpulse, oldImpulse);

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
b2JointMotorSolvePositionConstraints(
    struct b2JointMotor* p,
    const struct b2SolverData* data)
{
    B2_NOT_USED(data);

    return b2True;
}

B2_API
void
b2JointMotorSetLinearOffset(
    struct b2JointMotor* p,
    const b2Vec2 linearOffset)
{
    if (linearOffset[0] != p->m_linearOffset[0] ||
        linearOffset[1] != p->m_linearOffset[1])
    {
        b2BodySetAwake(p->m_bodyA, b2True);
        b2BodySetAwake(p->m_bodyB, b2True);
        b2Vec2Assign(p->m_linearOffset, linearOffset);
    }
}

B2_API
void
b2JointMotorSetAngularOffset(
    struct b2JointMotor* p,
    float angularOffset)
{
    if (angularOffset != p->m_angularOffset)
    {
        b2BodySetAwake(p->m_bodyA, b2True);
        b2BodySetAwake(p->m_bodyB, b2True);
        p->m_angularOffset = angularOffset;
    }
}

B2_API
void
b2JointMotorSetMaxForce(
    struct b2JointMotor* p,
    float force)
{
    b2Assert(b2IsValid(force) && force >= 0.0f);
    p->m_maxForce = force;
}

B2_API
void
b2JointMotorSetMaxTorque(
    struct b2JointMotor* p,
    float torque)
{
    b2Assert(b2IsValid(torque) && torque >= 0.0f);
    p->m_maxTorque = torque;
}

B2_API
void
b2JointMotorSetCorrectionFactor(
    struct b2JointMotor* p,
    float factor)
{
    b2Assert(b2IsValid(factor) && 0.0f <= factor && factor <= 1.0f);
    p->m_correctionFactor = factor;
}
