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

#include "mmB2JointDistance.h"
#include "mmB2Common.h"
#include "mmB2Body.h"
#include "mmB2Draw.h"
#include "mmB2TimeStep.h"

#include <assert.h>
#include <string.h>

B2_API const struct b2MetaAllocator b2MetaAllocatorJointDistance =
{
    "b2JointDistance",
    sizeof(struct b2JointDistance),
    &b2JointDistancePrepare,
    &b2JointDistanceDiscard,
};

B2_API const struct b2JointMeta b2JointDistanceMeta =
{
    &b2JointDistanceGetAnchorA,
    &b2JointDistanceGetAnchorB,
    &b2JointDistanceGetReactionForce,
    &b2JointDistanceGetReactionTorque,
    &b2JointDistanceDump,
    &b2JointDistanceShiftOrigin,
    &b2JointDistanceDraw,
    &b2JointDistanceInitVelocityConstraints,
    &b2JointDistanceSolveVelocityConstraints,
    &b2JointDistanceSolvePositionConstraints,
};

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k * 

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

B2_API
void
b2JointDistanceDefReset(
    struct b2JointDistanceDef* p)
{
    b2JointDefReset((struct b2JointDef*)p);
    p->type = b2JointTypeDistance;
    b2Vec2Make(p->localAnchorA, 0.0f, 0.0f);
    b2Vec2Make(p->localAnchorB, 0.0f, 0.0f);
    p->length = 1.0f;
    p->minLength = 0.0f;
    p->maxLength = FLT_MAX;
    p->stiffness = 0.0f;
    p->damping = 0.0f;
}

B2_API
void
b2JointDistanceDefInitialize(
    struct b2JointDistanceDef* p,
    struct b2Body* bodyA,
    struct b2Body* bodyB,
    const b2Vec2 anchorA,
    const b2Vec2 anchorB)
{
    b2Vec2 d;
    p->bodyA = bodyA;
    p->bodyB = bodyB;
    b2BodyGetLocalPoint(bodyA, anchorA, p->localAnchorA);
    b2BodyGetLocalPoint(bodyB, anchorB, p->localAnchorB);
    b2Vec2Sub(d, anchorB, anchorA);
    p->length = b2MaxFloat(b2Vec2Length(d), b2_linearSlop);
    p->minLength = p->length;
    p->maxLength = p->length;
}

B2_API
void
b2JointDistancePrepare(
    struct b2JointDistance* p,
    const struct b2JointDistanceDef* def)
{
    b2JointFromDef((struct b2Joint*)p, (struct b2JointDef*)def);
    p->Meta = &b2JointDistanceMeta;

    b2Vec2Assign(p->m_localAnchorA, def->localAnchorA);
    b2Vec2Assign(p->m_localAnchorB, def->localAnchorB);
    p->m_length = b2MaxFloat(def->length, b2_linearSlop);
    p->m_minLength = b2MaxFloat(def->minLength, b2_linearSlop);
    p->m_maxLength = b2MaxFloat(def->maxLength, p->m_minLength);
    p->m_stiffness = def->stiffness;
    p->m_damping = def->damping;

    p->m_gamma = 0.0f;
    p->m_bias = 0.0f;
    p->m_impulse = 0.0f;
    p->m_lowerImpulse = 0.0f;
    p->m_upperImpulse = 0.0f;
    p->m_currentLength = 0.0f;
}

B2_API
void
b2JointDistanceDiscard(
    struct b2JointDistance* p)
{
    memset(p, 0, sizeof(struct b2JointDistance));
}

B2_API
void
b2JointDistanceGetAnchorA(
    const struct b2JointDistance* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyA, p->m_localAnchorA, anchor);
}

B2_API
void
b2JointDistanceGetAnchorB(
    const struct b2JointDistance* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyB, p->m_localAnchorB, anchor);
}

/// Get the reaction force given the inverse time step.
/// Unit is N.
B2_API
void
b2JointDistanceGetReactionForce(
    const struct b2JointDistance* p,
    float inv_dt,
    b2Vec2 force)
{
    b2Vec2Scale(force, p->m_u, inv_dt * (p->m_impulse + p->m_lowerImpulse - p->m_upperImpulse));
}

/// Get the reaction torque given the inverse time step.
/// Unit is N*m. This is always zero for a distance joint.
B2_API
float
b2JointDistanceGetReactionTorque(
    const struct b2JointDistance* p,
    float inv_dt)
{
    B2_NOT_USED(inv_dt);
    return 0.0f;
}

/// Dump joint to dmLog
B2_API
void
b2JointDistanceDump(
    const struct b2JointDistance* p)
{
    int32 indexA = p->m_bodyA->m_islandIndex;
    int32 indexB = p->m_bodyB->m_islandIndex;

    b2Dump("  b2DistanceJointDef jd;\n");
    b2Dump("  jd.bodyA = bodies[%d];\n", indexA);
    b2Dump("  jd.bodyB = bodies[%d];\n", indexB);
    b2Dump("  jd.collideConnected = bool(%d);\n", p->m_collideConnected);
    b2Dump("  jd.localAnchorA.Set(%.9g, %.9g);\n", p->m_localAnchorA[0], p->m_localAnchorA[1]);
    b2Dump("  jd.localAnchorB.Set(%.9g, %.9g);\n", p->m_localAnchorB[0], p->m_localAnchorB[1]);
    b2Dump("  jd.length = %.9g;\n", p->m_length);
    b2Dump("  jd.minLength = %.9g;\n", p->m_minLength);
    b2Dump("  jd.maxLength = %.9g;\n", p->m_maxLength);
    b2Dump("  jd.stiffness = %.9g;\n", p->m_stiffness);
    b2Dump("  jd.damping = %.9g;\n", p->m_damping);
    b2Dump("  joints[%d] = m_world->CreateJoint(&jd);\n", p->m_index);
}

B2_API
void
b2JointDistanceShiftOrigin(
    struct b2JointDistance* p,
    const b2Vec2 newOrigin)
{
    B2_NOT_USED(newOrigin);
}

B2_API
void
b2JointDistanceDraw(
    const struct b2JointDistance* p,
    struct b2Draw* draw)
{
    static const b2Color c1 = { 0.7f, 0.7f, 0.7f, 1.0f };
    static const b2Color c2 = { 0.3f, 0.9f, 0.3f, 1.0f };
    static const b2Color c3 = { 0.9f, 0.3f, 0.3f, 1.0f };
    static const b2Color c4 = { 0.4f, 0.4f, 0.4f, 1.0f };

    b2TransformConstRef xfA;
    b2TransformConstRef xfB;

    b2Vec2 v;

    b2Vec2 pA;
    b2Vec2 pB;

    b2Vec2 axis;

    b2Vec2 pRest;

    xfA = b2BodyGetTransform(p->m_bodyA);
    xfB = b2BodyGetTransform(p->m_bodyB);
    b2TransformMulVec2(pA, xfA, p->m_localAnchorA);
    b2TransformMulVec2(pB, xfB, p->m_localAnchorB);

    b2Vec2Sub(axis, pB, pA);
    b2Vec2Normalize(axis, axis);

    b2DrawSegment(draw, pA, pB, c4);

    b2Vec2Scale(v, axis, p->m_length);
    b2Vec2Add(pRest, pA, v);
    b2DrawPoint(draw, pRest, 8.0f, c1);

    if (p->m_minLength != p->m_maxLength)
    {
        if (p->m_minLength > b2_linearSlop)
        {
            b2Vec2 pMin;
            b2Vec2Scale(v, axis, p->m_minLength);
            b2Vec2Add(pMin, pA, v);
            b2DrawPoint(draw, pMin, 4.0f, c2);
        }

        if (p->m_maxLength < FLT_MAX)
        {
            b2Vec2 pMax;
            b2Vec2Scale(v, axis, p->m_maxLength);
            b2Vec2Add(pMax, pA, v);
            b2DrawPoint(draw, pMax, 4.0f, c3);
        }
    }
}

B2_API
void
b2JointDistanceInitVelocityConstraints(
    struct b2JointDistance* p,
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

    float crAu;
    float crBu;
    float invMass;

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
    b2Vec2Add(v, cB, p->m_rB);
    b2Vec2Sub(v, v, cA);
    b2Vec2Sub(p->m_u, v, p->m_rA);

    // Handle singularity.
    p->m_currentLength = b2Vec2Length(p->m_u);
    if (p->m_currentLength > b2_linearSlop)
    {
        b2Vec2Scale(p->m_u, p->m_u, 1.0f / p->m_currentLength);
    }
    else
    {
        b2Vec2Make(p->m_u, 0.0f, 0.0f);
        p->m_mass = 0.0f;
        p->m_impulse = 0.0f;
        p->m_lowerImpulse = 0.0f;
        p->m_upperImpulse = 0.0f;
    }

    crAu = b2Vec2CrossProduct(p->m_rA, p->m_u);
    crBu = b2Vec2CrossProduct(p->m_rB, p->m_u);
    invMass = p->m_invMassA + p->m_invIA * crAu * crAu + p->m_invMassB + p->m_invIB * crBu * crBu;
    p->m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

    if (p->m_stiffness > 0.0f && p->m_minLength < p->m_maxLength)
    {
        float C;

        float d;
        float k;

        float h;

        // soft
        C = p->m_currentLength - p->m_length;

        d = p->m_damping;
        k = p->m_stiffness;

        // magic formulas
        h = data->step.dt;

        // gamma = 1 / (h * (d + h * k))
        // the extra factor of h in the denominator is since the lambda is an impulse, not a force
        p->m_gamma = h * (d + h * k);
        p->m_gamma = p->m_gamma != 0.0f ? 1.0f / p->m_gamma : 0.0f;
        p->m_bias = C * h * k * p->m_gamma;

        invMass += p->m_gamma;
        p->m_softMass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
    }
    else
    {
        // rigid
        p->m_gamma = 0.0f;
        p->m_bias = 0.0f;
        p->m_softMass = p->m_mass;
    }

    if (data->step.warmStarting)
    {
        b2Vec2 P;

        // Scale the impulse to support a variable time step.
        p->m_impulse *= data->step.dtRatio;
        p->m_lowerImpulse *= data->step.dtRatio;
        p->m_upperImpulse *= data->step.dtRatio;

        b2Vec2Scale(P, p->m_u, p->m_impulse + p->m_lowerImpulse - p->m_upperImpulse);
        b2Vec2Scale(v, P, p->m_invMassA);
        b2Vec2Sub(vA, vA, v);
        wA -= p->m_invIA * b2Vec2CrossProduct(p->m_rA, P);
        b2Vec2Scale(v, P, p->m_invMassB);
        b2Vec2Add(vB, vB, v);
        wB += p->m_invIB * b2Vec2CrossProduct(p->m_rB, P);
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
b2JointDistanceSolveVelocityConstraints(
    struct b2JointDistance* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;

    b2Vec2 vA;
    float wA;
    b2Vec2 vB;
    float wB;

    b2Vec2Assign(vA, data->velocities[p->m_indexA].v);
    wA = data->velocities[p->m_indexA].w;
    b2Vec2Assign(vB, data->velocities[p->m_indexB].v);
    wB = data->velocities[p->m_indexB].w;

    if (p->m_minLength < p->m_maxLength)
    {
        if (p->m_stiffness > 0.0f)
        {
            b2Vec2 vpA;
            b2Vec2 vpB;
            float Cdot;
            float impulse;
            b2Vec2 P;
            // Cdot = dot(u, v + cross(w, r))
            b2Vec2CrossProductKL(v, wA, p->m_rA);
            b2Vec2Add(vpA, vA, v);
            b2Vec2CrossProductKL(v, wB, p->m_rB);
            b2Vec2Add(vpB, vB, v);
            b2Vec2Sub(v, vpB, vpA);
            Cdot = b2Vec2DotProduct(p->m_u, v);

            impulse = -p->m_softMass * (Cdot + p->m_bias + p->m_gamma * p->m_impulse);
            p->m_impulse += impulse;

            b2Vec2Scale(P, p->m_u, impulse);
            b2Vec2Scale(v, P, p->m_invMassA);
            b2Vec2Sub(vA, vA, v);
            wA -= p->m_invIA * b2Vec2CrossProduct(p->m_rA, P);
            b2Vec2Scale(v, P, p->m_invMassB);
            b2Vec2Add(vB, vB, v);
            wB += p->m_invIB * b2Vec2CrossProduct(p->m_rB, P);
        }

        // lower
        {
            float C;
            float bias;

            b2Vec2 vpA;
            b2Vec2 vpB;
            float Cdot;

            float impulse;
            float oldImpulse;

            b2Vec2 P;

            C = p->m_currentLength - p->m_minLength;
            bias = b2MaxFloat(0.0f, C) * data->step.inv_dt;

            b2Vec2CrossProductKL(v, wA, p->m_rA);
            b2Vec2Add(vpA, vA, v);
            b2Vec2CrossProductKL(v, wB, p->m_rB);
            b2Vec2Add(vpB, vB, v);
            b2Vec2Sub(v, vpB, vpA);
            Cdot = b2Vec2DotProduct(p->m_u, v);

            impulse = -p->m_mass * (Cdot + bias);
            oldImpulse = p->m_lowerImpulse;
            p->m_lowerImpulse = b2MaxFloat(0.0f, p->m_lowerImpulse + impulse);
            impulse = p->m_lowerImpulse - oldImpulse;
            b2Vec2Scale(P, p->m_u, impulse);

            b2Vec2Scale(v, P, p->m_invMassA);
            b2Vec2Sub(vA, vA, v);
            wA -= p->m_invIA * b2Vec2CrossProduct(p->m_rA, P);
            b2Vec2Scale(v, P, p->m_invMassB);
            b2Vec2Add(vB, vB, v);
            wB += p->m_invIB * b2Vec2CrossProduct(p->m_rB, P);
        }

        // upper
        {
            float C;
            float bias;

            b2Vec2 vpA;
            b2Vec2 vpB;
            float Cdot;

            float impulse;
            float oldImpulse;

            b2Vec2 P;

            C = p->m_maxLength - p->m_currentLength;
            bias = b2MaxFloat(0.0f, C) * data->step.inv_dt;

            b2Vec2CrossProductKL(v, wA, p->m_rA);
            b2Vec2Add(vpA, vA, v);
            b2Vec2CrossProductKL(v, wB, p->m_rB);
            b2Vec2Add(vpB, vB, v);
            b2Vec2Sub(v, vpA, vpB);
            Cdot = b2Vec2DotProduct(p->m_u, v);

            impulse = -p->m_mass * (Cdot + bias);
            oldImpulse = p->m_upperImpulse;
            p->m_upperImpulse = b2MaxFloat(0.0f, p->m_upperImpulse + impulse);
            impulse = p->m_upperImpulse - oldImpulse;
            b2Vec2Scale(P, p->m_u, -impulse);

            b2Vec2Scale(v, P, p->m_invMassA);
            b2Vec2Sub(vA, vA, v);
            wA -= p->m_invIA * b2Vec2CrossProduct(p->m_rA, P);
            b2Vec2Scale(v, P, p->m_invMassB);
            b2Vec2Add(vB, vB, v);
            wB += p->m_invIB * b2Vec2CrossProduct(p->m_rB, P);
        }
    }
    else
    {
        b2Vec2 vpA;
        b2Vec2 vpB;
        float Cdot;

        float impulse;

        b2Vec2 P;

        // Equal limits

        // Cdot = dot(u, v + cross(w, r))
        b2Vec2CrossProductKL(v, wA, p->m_rA);
        b2Vec2Add(vpA, vA, v);
        b2Vec2CrossProductKL(v, wB, p->m_rB);
        b2Vec2Add(vpB, vB, v);
        b2Vec2Sub(v, vpB, vpA);
        Cdot = b2Vec2DotProduct(p->m_u, v);

        impulse = -p->m_mass * Cdot;
        p->m_impulse += impulse;

        b2Vec2Scale(P, p->m_u, impulse);
        b2Vec2Scale(v, P, p->m_invMassA);
        b2Vec2Sub(vA, vA, v);
        wA -= p->m_invIA * b2Vec2CrossProduct(p->m_rA, P);
        b2Vec2Scale(v, P, p->m_invMassB);
        b2Vec2Add(vB, vB, v);
        wB += p->m_invIB * b2Vec2CrossProduct(p->m_rB, P);
    }

    b2Vec2Assign(data->velocities[p->m_indexA].v, vA);
    data->velocities[p->m_indexA].w = wA;
    b2Vec2Assign(data->velocities[p->m_indexB].v, vB);
    data->velocities[p->m_indexB].w = wB;
}

B2_API
int
b2JointDistanceSolvePositionConstraints(
    struct b2JointDistance* p,
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
    b2Vec2 u;

    float length;
    float C;

    float impulse;
    b2Vec2 P;

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
    b2Vec2Add(v, cB, rB);
    b2Vec2Sub(v, v, cA);
    b2Vec2Sub(u, v, rA);

    length = b2Vec2Normalize(u, u);
    if (p->m_minLength == p->m_maxLength)
    {
        C = length - p->m_minLength;
    }
    else if (length < p->m_minLength)
    {
        C = length - p->m_minLength;
    }
    else if (p->m_maxLength < length)
    {
        C = length - p->m_maxLength;
    }
    else
    {
        return b2True;
    }

    impulse = -p->m_mass * C;
    b2Vec2Scale(P, u, impulse);

    b2Vec2Scale(v, P, p->m_invMassA);
    b2Vec2Sub(cA, cA, v);
    aA -= p->m_invIA * b2Vec2CrossProduct(rA, P);
    b2Vec2Scale(v, P, p->m_invMassB);
    b2Vec2Add(cB, cB, v);
    aB += p->m_invIB * b2Vec2CrossProduct(rB, P);

    b2Vec2Assign(data->positions[p->m_indexA].c, cA);
    data->positions[p->m_indexA].a = aA;
    b2Vec2Assign(data->positions[p->m_indexB].c, cB);
    data->positions[p->m_indexB].a = aB;

    return b2AbsFloat(C) < b2_linearSlop;
}

B2_API
float
b2JointDistanceSetLength(
    struct b2JointDistance* p,
    float length)
{
    p->m_impulse = 0.0f;
    p->m_length = b2MaxFloat(b2_linearSlop, length);
    return p->m_length;
}

B2_API
float
b2JointDistanceSetMinLength(
    struct b2JointDistance* p,
    float minLength)
{
    p->m_lowerImpulse = 0.0f;
    p->m_minLength = b2ClampFloat(minLength, b2_linearSlop, p->m_maxLength);
    return p->m_minLength;
}

B2_API
float
b2JointDistanceSetMaxLength(
    struct b2JointDistance* p,
    float maxLength)
{
    p->m_upperImpulse = 0.0f;
    p->m_maxLength = b2MaxFloat(maxLength, p->m_minLength);
    return p->m_maxLength;
}

B2_API
float
b2JointDistanceGetCurrentLength(
    const struct b2JointDistance* p)
{
    b2Vec2 pA;
    b2Vec2 pB;
    b2Vec2 d;
    float length;

    b2BodyGetWorldPoint(p->m_bodyA, p->m_localAnchorA, pA);
    b2BodyGetWorldPoint(p->m_bodyB, p->m_localAnchorB, pB);
    b2Vec2Sub(d, pB, pA);
    length = b2Vec2Length(d);
    return length;
}
