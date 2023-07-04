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

#include "mmB2JointWeld.h"
#include "mmB2Common.h"
#include "mmB2Body.h"
#include "mmB2Draw.h"
#include "mmB2TimeStep.h"

#include <assert.h>
#include <string.h>

B2_API const struct b2MetaAllocator b2MetaAllocatorJointWeld =
{
    "b2JointWeld",
    sizeof(struct b2JointWeld),
    &b2JointWeldPrepare,
    &b2JointWeldDiscard,
};

B2_API const struct b2JointMeta b2JointWeldMeta =
{
    &b2JointWeldGetAnchorA,
    &b2JointWeldGetAnchorB,
    &b2JointWeldGetReactionForce,
    &b2JointWeldGetReactionTorque,
    &b2JointWeldDump,
    &b2JointWeldShiftOrigin,
    &b2JointWeldDraw,
    &b2JointWeldInitVelocityConstraints,
    &b2JointWeldSolveVelocityConstraints,
    &b2JointWeldSolvePositionConstraints,
};

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

B2_API
void
b2JointWeldDefReset(
    struct b2JointWeldDef* p)
{
    b2JointDefReset((struct b2JointDef*)p);
    p->type = b2JointTypeWeld;
    b2Vec2Make(p->localAnchorA, 0.0f, 0.0f);
    b2Vec2Make(p->localAnchorB, 0.0f, 0.0f);
    p->referenceAngle = 0.0f;
    p->stiffness = 0.0f;
    p->damping = 0.0f;
}

B2_API
void
b2JointWeldDefInitialize(
    struct b2JointWeldDef* p,
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
b2JointWeldPrepare(
    struct b2JointWeld* p,
    const struct b2JointWeldDef* def)
{
    b2JointFromDef((struct b2Joint*)p, (struct b2JointDef*)def);
    p->Meta = &b2JointWeldMeta;

    b2Vec2Assign(p->m_localAnchorA, def->localAnchorA);
    b2Vec2Assign(p->m_localAnchorB, def->localAnchorB);
    p->m_referenceAngle = def->referenceAngle;
    p->m_stiffness = def->stiffness;
    p->m_damping = def->damping;

    b2Vec3SetZero(p->m_impulse);
}

B2_API
void
b2JointWeldDiscard(
    struct b2JointWeld* p)
{
    memset(p, 0, sizeof(struct b2JointWeld));
}

B2_API
void
b2JointWeldGetAnchorA(
    const struct b2JointWeld* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyA, p->m_localAnchorA, anchor);
}

B2_API
void
b2JointWeldGetAnchorB(
    const struct b2JointWeld* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyB, p->m_localAnchorB, anchor);
}

B2_API
void
b2JointWeldGetReactionForce(
    const struct b2JointWeld* p,
    float inv_dt,
    b2Vec2 force)
{
    b2Vec2 P;
    P[0] = p->m_impulse[0];
    P[1] = p->m_impulse[1];
    b2Vec2Scale(force, P, inv_dt);
}

B2_API
float
b2JointWeldGetReactionTorque(
    const struct b2JointWeld* p,
    float inv_dt)
{
    return inv_dt * p->m_impulse[2];
}

B2_API
void
b2JointWeldDump(
    const struct b2JointWeld* p)
{
    int32 indexA = p->m_bodyA->m_islandIndex;
    int32 indexB = p->m_bodyB->m_islandIndex;

    b2Dump("  b2WeldJointDef jd;\n");
    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
    b2Dump("  jd.collideConnected = bool(%d);\n", p->m_collideConnected);
    b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", p->m_localAnchorA[0], p->m_localAnchorA[1]);
    b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", p->m_localAnchorB[0], p->m_localAnchorB[1]);
    b2Dump("  jd.referenceAngle = %.9g;\n", p->m_referenceAngle);
    b2Dump("  jd.stiffness = %.9g;\n", p->m_stiffness);
    b2Dump("  jd.damping = %.9g;\n", p->m_damping);
    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", p->m_index);
}

B2_API
void
b2JointWeldShiftOrigin(
    struct b2JointWeld* p,
    const b2Vec2 newOrigin)
{
    B2_NOT_USED(newOrigin);
}

B2_API
void
b2JointWeldDraw(
    const struct b2JointWeld* p,
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
    b2JointWeldGetAnchorA(p, p1);
    b2JointWeldGetAnchorB(p, p2);

    b2DrawSegment(draw, x1, p1, color);
    b2DrawSegment(draw, p1, p2, color);
    b2DrawSegment(draw, x2, p2, color);
}

B2_API
void
b2JointWeldInitVelocityConstraints(
    struct b2JointWeld* p,
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

    b2Mat33 K;

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
    //     [ 0       -1 0       1]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
    //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
    //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

    mA = p->m_invMassA, mB = p->m_invMassB;
    iA = p->m_invIA   , iB = p->m_invIB;

    K[0][0] = mA + mB + p->m_rA[1] * p->m_rA[1] * iA + p->m_rB[1] * p->m_rB[1] * iB;
    K[1][0] = -p->m_rA[1] * p->m_rA[0] * iA - p->m_rB[1] * p->m_rB[0] * iB;
    K[2][0] = -p->m_rA[1] * iA - p->m_rB[1] * iB;
    K[0][1] = K[1][0];
    K[1][1] = mA + mB + p->m_rA[0] * p->m_rA[0] * iA + p->m_rB[0] * p->m_rB[0] * iB;
    K[2][1] = p->m_rA[0] * iA + p->m_rB[0] * iB;
    K[0][2] = K[2][0];
    K[1][2] = K[2][1];
    K[2][2] = iA + iB;

    if (p->m_stiffness > 0.0f)
    {
        float invM;
        float C;
        float d;
        float k;
        float h;

        b2Mat33GetInverse22(p->m_mass, K);

        invM = iA + iB;

        C = aB - aA - p->m_referenceAngle;

        // Damping coefficient
        d = p->m_damping;

        // Spring stiffness
        k = p->m_stiffness;

        // magic formulas
        h = data->step.dt;
        p->m_gamma = h * (d + h * k);
        p->m_gamma = p->m_gamma != 0.0f ? 1.0f / p->m_gamma : 0.0f;
        p->m_bias = C * h * k * p->m_gamma;

        invM += p->m_gamma;
        p->m_mass[2][2] = invM != 0.0f ? 1.0f / invM : 0.0f;
    }
    else if (K[2][2] == 0.0f)
    {
        b2Mat33GetInverse22(p->m_mass, K);
        p->m_gamma = 0.0f;
        p->m_bias = 0.0f;
    }
    else
    {
        b2Mat33GetSymInverse33(p->m_mass, K);
        p->m_gamma = 0.0f;
        p->m_bias = 0.0f;
    }

    if (data->step.warmStarting)
    {
        b2Vec2 P;

        // Scale impulses to support a variable time step.
        b2Vec3Scale(p->m_impulse, p->m_impulse, data->step.dtRatio);

        P[0] = p->m_impulse[0];
        P[1] = p->m_impulse[1];

        b2Vec2Scale(v, P, mA);
        b2Vec2Sub(vA, vA, v);
        wA -= iA * (b2Vec2CrossProduct(p->m_rA, P) + p->m_impulse[2]);

        b2Vec2Scale(v, P, mB);
        b2Vec2Add(vB, vB, v);
        wB += iB * (b2Vec2CrossProduct(p->m_rB, P) + p->m_impulse[2]);
    }
    else
    {
        b2Vec3SetZero(p->m_impulse);
    }

    b2Vec2Assign(data->velocities[p->m_indexA].v, vA);
    data->velocities[p->m_indexA].w = wA;
    b2Vec2Assign(data->velocities[p->m_indexB].v, vB);
    data->velocities[p->m_indexB].w = wB;
}

B2_API
void
b2JointWeldSolveVelocityConstraints(
    struct b2JointWeld* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;
    b2Vec2 v1, v2;

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

    if (p->m_stiffness > 0.0f)
    {
        float Cdot2;

        float impulse2;

        b2Vec2 Cdot1;

        b2Vec2 impulse1;

        b2Vec2 P;

        Cdot2 = wB - wA;

        impulse2 = -p->m_mass[2][2] * (Cdot2 + p->m_bias + p->m_gamma * p->m_impulse[2]);
        p->m_impulse[2] += impulse2;

        wA -= iA * impulse2;
        wB += iB * impulse2;

        b2Vec2CrossProductKL(v1, wB, p->m_rB);
        b2Vec2CrossProductKL(v2, wA, p->m_rA);
        b2Vec2Add(v1, vB, v1);
        b2Vec2Add(v2, vA, v2);
        b2Vec2Sub(Cdot1, v1, v2);

        b2Mat33MulVec2(impulse1, p->m_mass, Cdot1);
        b2Vec2Negate(impulse1, impulse1);
        p->m_impulse[0] += impulse1[0];
        p->m_impulse[1] += impulse1[1];

        b2Vec2Assign(P, impulse1);

        b2Vec2Scale(v, P, mA);
        b2Vec2Sub(vA, vA, v);
        wA -= iA * b2Vec2CrossProduct(p->m_rA, P);

        b2Vec2Scale(v, P, mB);
        b2Vec2Add(vB, vB, v);
        wB += iB * b2Vec2CrossProduct(p->m_rB, P);
    }
    else
    {
        b2Vec2 Cdot1;
        float Cdot2;
        b2Vec3 Cdot;

        b2Vec3 impulse;

        b2Vec2 P;

        b2Vec2CrossProductKL(v1, wB, p->m_rB);
        b2Vec2CrossProductKL(v2, wA, p->m_rA);
        b2Vec2Add(v1, vB, v1);
        b2Vec2Add(v2, vA, v2);
        b2Vec2Sub(Cdot1, v1, v2);
        Cdot2 = wB - wA;
        Cdot[0] = Cdot1[0];
        Cdot[1] = Cdot1[1];
        Cdot[2] = Cdot2;

        b2Mat33MulVec3(impulse, p->m_mass, Cdot);
        b2Vec3Negate(impulse, impulse);
        b2Vec3Add(p->m_impulse, p->m_impulse, impulse);

        P[0] = impulse[0];
        P[1] = impulse[1];

        b2Vec2Scale(v, P, mA);
        b2Vec2Sub(vA, vA, v);
        wA -= iA * (b2Vec2CrossProduct(p->m_rA, P) + impulse[2]);

        b2Vec2Scale(v, P, mB);
        b2Vec2Add(vB, vB, v);
        wB += iB * (b2Vec2CrossProduct(p->m_rB, P) + impulse[2]);
    }

    b2Vec2Assign(data->velocities[p->m_indexA].v, vA);
    data->velocities[p->m_indexA].w = wA;
    b2Vec2Assign(data->velocities[p->m_indexB].v, vB);
    data->velocities[p->m_indexB].w = wB;
}

B2_API
int
b2JointWeldSolvePositionConstraints(
    struct b2JointWeld* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;

    b2Vec2 cA;
    float aA;
    b2Vec2 cB;
    float aB;

    b2Rot qA, qB;

    float mA, mB;
    float iA, iB;

    b2Vec2 rA;
    b2Vec2 rB;

    float positionError, angularError;

    b2Mat33 K;

    b2Vec2Assign(cA, data->positions[p->m_indexA].c);
    aA = data->positions[p->m_indexA].a;
    b2Vec2Assign(cB, data->positions[p->m_indexB].c);
    aB = data->positions[p->m_indexB].a;

    b2RotFromAngle(qA, aA);
    b2RotFromAngle(qB, aB);

    mA = p->m_invMassA, mB = p->m_invMassB;
    iA = p->m_invIA   , iB = p->m_invIB;

    b2Vec2Sub(v, p->m_localAnchorA, p->m_localCenterA);
    b2RotMulVec2(rA, qA, v);
    b2Vec2Sub(v, p->m_localAnchorB, p->m_localCenterB);
    b2RotMulVec2(rB, qB, v);

    K[0][0] = mA + mB + rA[1] * rA[1] * iA + rB[1] * rB[1] * iB;
    K[1][0] = -rA[1] * rA[0] * iA - rB[1] * rB[0] * iB;
    K[2][0] = -rA[1] * iA - rB[1] * iB;
    K[0][1] = K[1][0];
    K[1][1] = mA + mB + rA[0] * rA[0] * iA + rB[0] * rB[0] * iB;
    K[2][1] = rA[0] * iA + rB[0] * iB;
    K[0][2] = K[2][0];
    K[1][2] = K[2][1];
    K[2][2] = iA + iB;

    if (p->m_stiffness > 0.0f)
    {
        b2Vec2 C1;
        b2Vec2 P;

        b2Vec2Add(v, cB, rB);
        b2Vec2Sub(v, v, cA);
        b2Vec2Sub(C1, v, rA);

        positionError = b2Vec2Length(C1);
        angularError = 0.0f;

        b2Mat33Solve22(P, K, C1);
        b2Vec2Negate(P, P);

        b2Vec2Scale(v, P, mA);
        b2Vec2Sub(cA, cA, v);
        aA -= iA * b2Vec2CrossProduct(rA, P);

        b2Vec2Scale(v, P, mB);
        b2Vec2Add(cB, cB, v);
        aB += iB * b2Vec2CrossProduct(rB, P);
    }
    else
    {
        b2Vec2 C1;
        float C2;

        b2Vec3 C;

        b2Vec3 impulse;
        b2Vec2 P;

        b2Vec2Add(v, cB, rB);
        b2Vec2Sub(v, v, cA);
        b2Vec2Sub(C1, v, rA);

        C2 = aB - aA - p->m_referenceAngle;

        positionError = b2Vec2Length(C1);
        angularError = b2AbsFloat(C2);

        C[0] = C1[0];
        C[1] = C1[1];
        C[2] = C2;

        if (K[2][2] > 0.0f)
        {
            b2Mat33Solve33(impulse, K, C);
            b2Vec3Negate(impulse, impulse);
        }
        else
        {
            b2Vec2 impulse2;
            b2Mat33Solve22(impulse2, K, C1);
            b2Vec2Negate(impulse2, impulse2);
            b2Vec3Make(impulse, impulse2[0], impulse2[1], 0.0f);
        }

        P[0] = impulse[0];
        P[1] = impulse[1];

        b2Vec2Scale(v, P, mA);
        b2Vec2Sub(cA, cA, v);
        aA -= iA * (b2Vec2CrossProduct(rA, P) + impulse[2]);

        b2Vec2Scale(v, P, mB);
        b2Vec2Add(cB, cB, v);
        aB += iB * (b2Vec2CrossProduct(rB, P) + impulse[2]);
    }

    b2Vec2Assign(data->positions[p->m_indexA].c, cA);
    data->positions[p->m_indexA].a = aA;
    b2Vec2Assign(data->positions[p->m_indexB].c, cB);
    data->positions[p->m_indexB].a = aB;

    return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
}
