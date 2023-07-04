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

#include "mmB2JointPulley.h"
#include "mmB2Common.h"
#include "mmB2Body.h"
#include "mmB2Draw.h"
#include "mmB2TimeStep.h"

#include <assert.h>
#include <string.h>

B2_API const struct b2MetaAllocator b2MetaAllocatorJointPulley =
{
    "b2JointPulley",
    sizeof(struct b2JointPulley),
    &b2JointPulleyPrepare,
    &b2JointPulleyDiscard,
};

B2_API const struct b2JointMeta b2JointPulleyMeta =
{
    &b2JointPulleyGetAnchorA,
    &b2JointPulleyGetAnchorB,
    &b2JointPulleyGetReactionForce,
    &b2JointPulleyGetReactionTorque,
    &b2JointPulleyDump,
    &b2JointPulleyShiftOrigin,
    &b2JointPulleyDraw,
    &b2JointPulleyInitVelocityConstraints,
    &b2JointPulleySolveVelocityConstraints,
    &b2JointPulleySolvePositionConstraints,
};

B2_API const float b2_minPulleyLength = 2.0f;

// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// C = C0 - (length1 + ratio * length2)
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

B2_API
void
b2JointPulleyDefReset(
    struct b2JointPulleyDef* p)
{
    b2JointDefReset((struct b2JointDef*)p);
    p->type = b2JointTypePulley;
    b2Vec2Make(p->groundAnchorA, -1.0f, 1.0f);
    b2Vec2Make(p->groundAnchorB, +1.0f, 1.0f);
    b2Vec2Make(p->localAnchorA, -1.0f, 0.0f);
    b2Vec2Make(p->localAnchorB, +1.0f, 0.0f);
    p->lengthA = 0.0f;
    p->lengthB = 0.0f;
    p->ratio = 1.0f;
    p->collideConnected = b2True;
}

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
    float ratio)
{
    b2Vec2 dA;
    b2Vec2 dB;

    p->bodyA = bodyA;
    p->bodyB = bodyB;
    b2Vec2Assign(p->groundAnchorA, groundAnchorA);
    b2Vec2Assign(p->groundAnchorB, groundAnchorB);
    b2BodyGetLocalPoint(bodyA, anchorA, p->localAnchorA);
    b2BodyGetLocalPoint(bodyB, anchorB, p->localAnchorB);
    b2Vec2Sub(dA, anchorA, groundAnchorA);
    p->lengthA = b2Vec2Length(dA);
    b2Vec2Sub(dB, anchorB, groundAnchorB);
    p->lengthB = b2Vec2Length(dB);
    p->ratio = ratio;
    b2Assert(ratio > b2_epsilon);
}

B2_API
void
b2JointPulleyPrepare(
    struct b2JointPulley* p,
    const struct b2JointPulleyDef* def)
{
    b2JointFromDef((struct b2Joint*)p, (struct b2JointDef*)def);
    p->Meta = &b2JointPulleyMeta;

    b2Vec2Assign(p->m_groundAnchorA, def->groundAnchorA);
    b2Vec2Assign(p->m_groundAnchorB, def->groundAnchorB);
    b2Vec2Assign(p->m_localAnchorA, def->localAnchorA);
    b2Vec2Assign(p->m_localAnchorB, def->localAnchorB);

    p->m_lengthA = def->lengthA;
    p->m_lengthB = def->lengthB;

    b2Assert(def->ratio != 0.0f);
    p->m_ratio = def->ratio;

    p->m_constant = def->lengthA + p->m_ratio * def->lengthB;

    p->m_impulse = 0.0f;
}

B2_API
void
b2JointPulleyDiscard(
    struct b2JointPulley* p)
{
    memset(p, 0, sizeof(struct b2JointPulley));
}

B2_API
void
b2JointPulleyGetAnchorA(
    const struct b2JointPulley* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyA, p->m_localAnchorA, anchor);
}

B2_API
void
b2JointPulleyGetAnchorB(
    const struct b2JointPulley* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyB, p->m_localAnchorB, anchor);
}

B2_API
void
b2JointPulleyGetReactionForce(
    const struct b2JointPulley* p,
    float inv_dt,
    b2Vec2 force)
{
    b2Vec2 P;
    b2Vec2Scale(P, p->m_uB, p->m_impulse);
    b2Vec2Scale(force, P, inv_dt);
}

B2_API
float
b2JointPulleyGetReactionTorque(
    const struct b2JointPulley* p,
    float inv_dt)
{
    B2_NOT_USED(inv_dt);
    return 0.0f;
}

B2_API
void
b2JointPulleyDump(
    const struct b2JointPulley* p)
{
    int32 indexA = p->m_bodyA->m_islandIndex;
    int32 indexB = p->m_bodyB->m_islandIndex;

    b2Dump("  b2PulleyJointDef jd;\n");
    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
    b2Dump("  jd.collideConnected = bool(%d);\n", p->m_collideConnected);
    b2Dump("  jd.groundAnchorA.Set(%.9g, %.9g);\n", p->m_groundAnchorA[0], p->m_groundAnchorA[1]);
    b2Dump("  jd.groundAnchorB.Set(%.9g, %.9g);\n", p->m_groundAnchorB[0], p->m_groundAnchorB[1]);
    b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", p->m_localAnchorA[0], p->m_localAnchorA[1]);
    b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", p->m_localAnchorB[0], p->m_localAnchorB[1]);
    b2Dump("  jd.lengthA = %.9g;\n", p->m_lengthA);
    b2Dump("  jd.lengthB = %.9g;\n", p->m_lengthB);
    b2Dump("  jd.ratio = %.9g;\n", p->m_ratio);
    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", p->m_index);
}

B2_API
void
b2JointPulleyShiftOrigin(
    struct b2JointPulley* p,
    const b2Vec2 newOrigin)
{
    b2Vec2Sub(p->m_groundAnchorA, p->m_groundAnchorA, newOrigin);
    b2Vec2Sub(p->m_groundAnchorB, p->m_groundAnchorB, newOrigin);
}

B2_API
void
b2JointPulleyDraw(
    const struct b2JointPulley* p,
    struct b2Draw* draw)
{
    static const b2Color color = { 0.5f, 0.8f, 0.8f, 1.0f };

    b2TransformConstRef xf1;
    b2TransformConstRef xf2;
    b2Vec2 x1;
    b2Vec2 x2;
    b2Vec2 p1;
    b2Vec2 p2;

    b2Vec2ConstRef s1;
    b2Vec2ConstRef s2;

    xf1 = b2BodyGetTransform(p->m_bodyA);
    xf2 = b2BodyGetTransform(p->m_bodyB);
    b2Vec2Assign(x1, xf1[0]);
    b2Vec2Assign(x2, xf2[0]);
    b2JointPulleyGetAnchorA(p, p1);
    b2JointPulleyGetAnchorB(p, p2);

    s1 = b2JointPulleyGetGroundAnchorA(p);
    s2 = b2JointPulleyGetGroundAnchorB(p);
    b2DrawSegment(draw, s1, p1, color);
    b2DrawSegment(draw, s2, p2, color);
    b2DrawSegment(draw, s1, s2, color);
}

B2_API
void
b2JointPulleyInitVelocityConstraints(
    struct b2JointPulley* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;

    b2Vec2 cA;
    float aA;
    b2Vec2 vA;
    float wA;

    b2Vec2 cB;
    float aB;
    b2Vec2 vB;
    float wB;

    b2Rot qA, qB;

    float lengthA;
    float lengthB;

    float ruA;
    float ruB;

    float mA;
    float mB;

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

    b2Vec2Sub(v, p->m_localAnchorA, p->m_localCenterA);
    b2RotMulVec2(p->m_rA, qA, v);
    b2Vec2Sub(v, p->m_localAnchorB, p->m_localCenterB);
    b2RotMulVec2(p->m_rB, qB, v);

    // Get the pulley axes.
    b2Vec2Add(v, cA, p->m_rA);
    b2Vec2Sub(p->m_uA, v, p->m_groundAnchorA);
    b2Vec2Add(v, cB, p->m_rB);
    b2Vec2Sub(p->m_uB, v, p->m_groundAnchorB);

    lengthA = b2Vec2Length(p->m_uA);
    lengthB = b2Vec2Length(p->m_uB);

    if (lengthA > 10.0f * b2_linearSlop)
    {
        b2Vec2Scale(p->m_uA, p->m_uA, 1.0f / lengthA);
    }
    else
    {
        b2Vec2SetZero(p->m_uA);
    }

    if (lengthB > 10.0f * b2_linearSlop)
    {
        b2Vec2Scale(p->m_uB, p->m_uB, 1.0f / lengthB);
    }
    else
    {
        b2Vec2SetZero(p->m_uB);
    }

    // Compute effective mass.
    ruA = b2Vec2CrossProduct(p->m_rA, p->m_uA);
    ruB = b2Vec2CrossProduct(p->m_rB, p->m_uB);

    mA = p->m_invMassA + p->m_invIA * ruA * ruA;
    mB = p->m_invMassB + p->m_invIB * ruB * ruB;

    p->m_mass = mA + p->m_ratio * p->m_ratio * mB;

    if (p->m_mass > 0.0f)
    {
        p->m_mass = 1.0f / p->m_mass;
    }

    if (data->step.warmStarting)
    {
        b2Vec2 PA;
        b2Vec2 PB;

        // Scale impulses to support variable time steps.
        p->m_impulse *= data->step.dtRatio;

        // Warm starting.
        b2Vec2Scale(PA, p->m_uA, -p->m_impulse);
        b2Vec2Scale(PB, p->m_uB, -p->m_ratio * p->m_impulse);

        b2Vec2Scale(v, PA, p->m_invMassA);
        b2Vec2Add(vA, vA, v);
        wA += p->m_invIA * b2Vec2CrossProduct(p->m_rA, PA);
        b2Vec2Scale(v, PB, p->m_invMassB);
        b2Vec2Add(vB, vB, v);
        wB += p->m_invIB * b2Vec2CrossProduct(p->m_rB, PB);
    }
    else
    {
        p->m_impulse = 0.0f;
    }

    b2Vec2Assign(data->velocities[p->m_indexA].v, vA);
    data->velocities[p->m_indexA].w = wA;
    b2Vec2Assign(data->velocities[p->m_indexB].v, vB);
    data->velocities[p->m_indexB].w = wB;
}

B2_API
void
b2JointPulleySolveVelocityConstraints(
    struct b2JointPulley* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;

    b2Vec2 vA;
    float wA;
    b2Vec2 vB;
    float wB;

    b2Vec2 vpA;
    b2Vec2 vpB;

    float Cdot;
    float impulse;

    b2Vec2 PA;
    b2Vec2 PB;

    b2Vec2Assign(vA, data->velocities[p->m_indexA].v);
    wA = data->velocities[p->m_indexA].w;
    b2Vec2Assign(vB, data->velocities[p->m_indexB].v);
    wB = data->velocities[p->m_indexB].w;

    b2Vec2CrossProductKL(v, wA, p->m_rA);
    b2Vec2Add(vpA, vA, v);
    b2Vec2CrossProductKL(v, wB, p->m_rB);
    b2Vec2Add(vpB, vB, v);

    Cdot = -b2Vec2DotProduct(p->m_uA, vpA) - p->m_ratio * b2Vec2DotProduct(p->m_uB, vpB);
    impulse = -p->m_mass * Cdot;
    p->m_impulse += impulse;

    b2Vec2Scale(PA, p->m_uA, -impulse);
    b2Vec2Scale(PB, p->m_uB, -p->m_ratio * impulse);

    b2Vec2Scale(v, PA, p->m_invMassA);
    b2Vec2Add(vA, vA, v);
    wA += p->m_invIA * b2Vec2CrossProduct(p->m_rA, PA);
    b2Vec2Scale(v, PB, p->m_invMassB);
    b2Vec2Add(vB, vB, v);
    wB += p->m_invIB * b2Vec2CrossProduct(p->m_rB, PB);

    b2Vec2Assign(data->velocities[p->m_indexA].v, vA);
    data->velocities[p->m_indexA].w = wA;
    b2Vec2Assign(data->velocities[p->m_indexB].v, vB);
    data->velocities[p->m_indexB].w = wB;
}

B2_API
int
b2JointPulleySolvePositionConstraints(
    struct b2JointPulley* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;

    b2Vec2 cA;
    float aA;
    b2Vec2 cB;
    float aB;

    b2Rot qA, qB;

    b2Vec2 rA;
    b2Vec2 rB;

    b2Vec2 uA;
    b2Vec2 uB;

    float lengthA;
    float lengthB;

    float ruA;
    float ruB;

    float mA;
    float mB;

    float mass;

    float C;
    float linearError;

    float impulse;

    b2Vec2 PA;
    b2Vec2 PB;

    b2Vec2Assign(cA, data->positions[p->m_indexA].c);
    aA = data->positions[p->m_indexA].a;
    b2Vec2Assign(cB, data->positions[p->m_indexB].c);
    aB = data->positions[p->m_indexB].a;

    b2RotFromAngle(qA, aA);
    b2RotFromAngle(qB, aB);

    b2Vec2Sub(v, p->m_localAnchorA, p->m_localCenterA);
    b2RotMulVec2(rA, qA, v);
    b2Vec2Sub(v, p->m_localAnchorB, p->m_localCenterB);
    b2RotMulVec2(rB, qB, v);

    // Get the pulley axes.
    b2Vec2Add(v, cA, rA);
    b2Vec2Sub(uA, v, p->m_groundAnchorA);
    b2Vec2Add(v, cB, rB);
    b2Vec2Sub(uB, v, p->m_groundAnchorB);

    lengthA = b2Vec2Length(uA);
    lengthB = b2Vec2Length(uB);

    if (lengthA > 10.0f * b2_linearSlop)
    {
        b2Vec2Scale(uA, uA, 1.0f / lengthA);
    }
    else
    {
        b2Vec2SetZero(uA);
    }

    if (lengthB > 10.0f * b2_linearSlop)
    {
        b2Vec2Scale(uB, uB, 1.0f / lengthB);
    }
    else
    {
        b2Vec2SetZero(uB);
    }

    // Compute effective mass.
    ruA = b2Vec2CrossProduct(rA, uA);
    ruB = b2Vec2CrossProduct(rB, uB);

    mA = p->m_invMassA + p->m_invIA * ruA * ruA;
    mB = p->m_invMassB + p->m_invIB * ruB * ruB;

    mass = mA + p->m_ratio * p->m_ratio * mB;

    if (mass > 0.0f)
    {
        mass = 1.0f / mass;
    }

    C = p->m_constant - lengthA - p->m_ratio * lengthB;
    linearError = b2AbsFloat(C);

    impulse = -mass * C;

    b2Vec2Scale(PA, uA, -impulse);
    b2Vec2Scale(PB, uB, -p->m_ratio * impulse);

    b2Vec2Scale(v, PA, p->m_invMassA);
    b2Vec2Add(cA, cA, v);
    aA += p->m_invIA * b2Vec2CrossProduct(rA, PA);
    b2Vec2Scale(v, PB, p->m_invMassB);
    b2Vec2Add(cB, cB, v);
    aB += p->m_invIB * b2Vec2CrossProduct(rB, PB);

    b2Vec2Assign(data->positions[p->m_indexA].c, cA);
    data->positions[p->m_indexA].a = aA;
    b2Vec2Assign(data->positions[p->m_indexB].c, cB);
    data->positions[p->m_indexB].a = aB;

    return linearError < b2_linearSlop;
}

B2_API
float
b2JointPulleyGetCurrentLengthA(
    const struct b2JointPulley* p)
{
    b2Vec2 w;
    b2Vec2ConstRef s;
    b2Vec2 d;

    b2BodyGetWorldPoint(p->m_bodyA, p->m_localAnchorA, w);
    s = p->m_groundAnchorA;
    b2Vec2Sub(d, w, s);
    return b2Vec2Length(d);
}

B2_API
float
b2JointPulleyGetCurrentLengthB(
    const struct b2JointPulley* p)
{
    b2Vec2 w;
    b2Vec2ConstRef s;
    b2Vec2 d;

    b2BodyGetWorldPoint(p->m_bodyB, p->m_localAnchorB, w);
    s = p->m_groundAnchorB;
    b2Vec2Sub(d, w, s);
    return b2Vec2Length(d);
}
