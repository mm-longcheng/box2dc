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

#include "mmB2JointMouse.h"
#include "mmB2Common.h"
#include "mmB2Body.h"
#include "mmB2Draw.h"
#include "mmB2TimeStep.h"

#include <assert.h>
#include <string.h>

B2_API const struct b2MetaAllocator b2MetaAllocatorJointMouse =
{
    "b2JointMouse",
    sizeof(struct b2JointMouse),
    &b2JointMousePrepare,
    &b2JointMouseDiscard,
};

B2_API const struct b2JointMeta b2JointMouseMeta =
{
    &b2JointMouseGetAnchorA,
    &b2JointMouseGetAnchorB,
    &b2JointMouseGetReactionForce,
    &b2JointMouseGetReactionTorque,
    &b2JointMouseDump,
    &b2JointMouseShiftOrigin,
    &b2JointMouseDraw,
    &b2JointMouseInitVelocityConstraints,
    &b2JointMouseSolveVelocityConstraints,
    &b2JointMouseSolvePositionConstraints,
};

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

B2_API
void
b2JointMouseDefReset(
    struct b2JointMouseDef* p)
{
    b2JointDefReset((struct b2JointDef*)p);
    p->type = b2JointTypeMouse;
    b2Vec2Make(p->target, 0.0f, 0.0f);
    p->maxForce = 0.0f;
    p->stiffness = 0.0f;
    p->damping = 0.0f;
}

B2_API
void
b2JointMousePrepare(
    struct b2JointMouse* p,
    const struct b2JointMouseDef* def)
{
    b2JointFromDef((struct b2Joint*)p, (struct b2JointDef*)def);
    p->Meta = &b2JointMouseMeta;

    b2Vec2Assign(p->m_targetA, def->target);
    b2TransformMulTVec2(p->m_localAnchorB, b2BodyGetTransform(p->m_bodyB), p->m_targetA);
    p->m_maxForce = def->maxForce;
    p->m_stiffness = def->stiffness;
    p->m_damping = def->damping;

    b2Vec2SetZero(p->m_impulse);
    p->m_beta = 0.0f;
    p->m_gamma = 0.0f;
}

B2_API
void
b2JointMouseDiscard(
    struct b2JointMouse* p)
{
    memset(p, 0, sizeof(struct b2JointMouse));
}

B2_API
void
b2JointMouseGetAnchorA(
    const struct b2JointMouse* p,
    b2Vec2 anchor)
{
    b2Vec2Assign(anchor, p->m_targetA);
}

B2_API
void
b2JointMouseGetAnchorB(
    const struct b2JointMouse* p,
    b2Vec2 anchor)
{
    b2BodyGetWorldPoint(p->m_bodyB, p->m_localAnchorB, anchor);
}

B2_API
void
b2JointMouseGetReactionForce(
    const struct b2JointMouse* p,
    float inv_dt,
    b2Vec2 force)
{
    b2Vec2Scale(force, p->m_impulse, inv_dt);
}

B2_API
float
b2JointMouseGetReactionTorque(
    const struct b2JointMouse* p,
    float inv_dt)
{
    return inv_dt * 0.0f;
}

B2_API
void
b2JointMouseDump(
    const struct b2JointMouse* p)
{
    b2Log("Mouse joint dumping is not supported.\n");
}

B2_API
void
b2JointMouseShiftOrigin(
    struct b2JointMouse* p,
    const b2Vec2 newOrigin)
{
    b2Vec2Sub(p->m_targetA, p->m_targetA, newOrigin);
}

B2_API
void
b2JointMouseDraw(
    const struct b2JointMouse* p,
    struct b2Draw* draw)
{
    static const b2Color c1 = { 0.0f, 1.0f, 0.0f, 1.0f };
    static const b2Color c2 = { 0.8f, 0.8f, 0.8f, 1.0f };

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
    b2JointMouseGetAnchorA(p, p1);
    b2JointMouseGetAnchorB(p, p2);

    b2DrawPoint(draw, p1, 4.0f, c1);
    b2DrawPoint(draw, p2, 4.0f, c1);
    b2DrawSegment(draw, p1, p2, c2);
}

B2_API
void
b2JointMouseInitVelocityConstraints(
    struct b2JointMouse* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;

    b2Vec2 cB;
    float aB;
    b2Vec2 vB;
    float wB;

    b2Rot qB;

    float d;
    float k;

    float h;

    b2Mat22 K;

    p->m_indexB = p->m_bodyB->m_islandIndex;
    b2Vec2Assign(p->m_localCenterB, p->m_bodyB->m_sweep.localCenter);
    p->m_invMassB = p->m_bodyB->m_invMass;
    p->m_invIB = p->m_bodyB->m_invI;

    b2Vec2Assign(cB, data->positions[p->m_indexB].c);
    aB = data->positions[p->m_indexB].a;
    b2Vec2Assign(vB, data->velocities[p->m_indexB].v);
    wB = data->velocities[p->m_indexB].w;

    b2RotFromAngle(qB, aB);

    d = p->m_damping;
    k = p->m_stiffness;

    // magic formulas
    // gamma has units of inverse mass.
    // beta has units of inverse time.
    h = data->step.dt;
    p->m_gamma = h * (d + h * k);
    if (p->m_gamma != 0.0f)
    {
        p->m_gamma = 1.0f / p->m_gamma;
    }
    p->m_beta = h * k * p->m_gamma;

    // Compute the effective mass matrix.
    b2Vec2Sub(v, p->m_localAnchorB, p->m_localCenterB);
    b2RotMulVec2(p->m_rB, qB, v);

    // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
    //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
    //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
    K[0][0] = p->m_invMassB + p->m_invIB * p->m_rB[1] * p->m_rB[1] + p->m_gamma;
    K[0][1] = -p->m_invIB * p->m_rB[0] * p->m_rB[1];
    K[1][0] = K[0][1];
    K[1][1] = p->m_invMassB + p->m_invIB * p->m_rB[0] * p->m_rB[0] + p->m_gamma;

    b2Mat22GetInverse(p->m_mass, K);

    b2Vec2Add(v, cB, p->m_rB);
    b2Vec2Sub(p->m_C, v, p->m_targetA);
    b2Vec2Scale(p->m_C, p->m_C, p->m_beta);

    // Cheat with some damping
    wB *= b2MaxFloat(0.0f, 1.0f - 0.02f * (60.0f * data->step.dt));

    if (data->step.warmStarting)
    {
        b2Vec2Scale(p->m_impulse, p->m_impulse, data->step.dtRatio);
        b2Vec2Scale(v, p->m_impulse, p->m_invMassB);
        b2Vec2Add(vB, vB, v);
        wB += p->m_invIB * b2Vec2CrossProduct(p->m_rB, p->m_impulse);
    }
    else
    {
        b2Vec3SetZero(p->m_impulse);
    }

    b2Vec2Assign(data->velocities[p->m_indexB].v, vB);
    data->velocities[p->m_indexB].w = wB;
}

B2_API
void
b2JointMouseSolveVelocityConstraints(
    struct b2JointMouse* p,
    const struct b2SolverData* data)
{
    b2Vec2 v;
    b2Vec2 v1, v2;

    b2Vec2 vB;
    float wB;

    b2Vec2 Cdot;
    b2Vec2 impulse;

    b2Vec2 oldImpulse;
    float maxImpulse;

    b2Vec2Assign(vB, data->velocities[p->m_indexB].v);
    wB = data->velocities[p->m_indexB].w;

    // Cdot = v + cross(w, r)
    b2Vec2CrossProductKL(v, wB, p->m_rB);
    b2Vec2Add(Cdot, vB, v);

    b2Vec2Add(v1, Cdot, p->m_C);
    b2Vec2Scale(v2, p->m_impulse, p->m_gamma);
    b2Vec2Add(v, v1, v2);
    b2Vec2Negate(v, v);
    b2Mat22MulVec2(impulse, p->m_mass, v);

    b2Vec2Assign(oldImpulse, p->m_impulse);
    b2Vec2Add(p->m_impulse, p->m_impulse, impulse);
    maxImpulse = data->step.dt * p->m_maxForce;
    if (b2Vec2SquaredLength(p->m_impulse) > maxImpulse * maxImpulse)
    {
        b2Vec2Scale(p->m_impulse, p->m_impulse, maxImpulse / b2Vec2Length(p->m_impulse));
    }
    b2Vec2Sub(impulse, p->m_impulse, oldImpulse);

    b2Vec2Scale(v, impulse, p->m_invMassB);
    b2Vec2Add(vB, vB, v);
    wB += p->m_invIB * b2Vec2CrossProduct(p->m_rB, impulse);

    b2Vec2Assign(data->velocities[p->m_indexB].v, vB);
    data->velocities[p->m_indexB].w = wB;
}

B2_API
int
b2JointMouseSolvePositionConstraints(
    struct b2JointMouse* p,
    const struct b2SolverData* data)
{
    B2_NOT_USED(data);
    return b2True;
}

B2_API
void
b2JointMouseSetTarget(
    struct b2JointMouse* p,
    const b2Vec2 target)
{
    if (!b2Vec2Equals(target, p->m_targetA))
    {
        b2BodySetAwake(p->m_bodyB, b2True);
        b2Vec2Assign(p->m_targetA, target);
    }
}
