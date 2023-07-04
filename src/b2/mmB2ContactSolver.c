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

#include "mmB2ContactSolver.h"
#include "mmB2Contact.h"
#include "mmB2Fixture.h"
#include "mmB2Shape.h"
#include "mmB2Body.h"
#include "mmB2StackAllocator.h"

#include <assert.h>

// Solver debugging is normally disabled because the block solver sometimes has to deal with a poorly conditioned effective mass matrix.
#define B2_DEBUG_SOLVER 0

B2_API int g_blockSolve = b2True;

struct b2ContactPositionConstraint
{
    b2Vec2 localPoints[b2_maxManifoldPoints];
    b2Vec2 localNormal;
    b2Vec2 localPoint;
    int32 indexA;
    int32 indexB;
    float invMassA, invMassB;
    b2Vec2 localCenterA, localCenterB;
    float invIA, invIB;
    enum b2ManifoldType type;
    float radiusA, radiusB;
    int32 pointCount;
};

struct b2PositionSolverManifold
{
    b2Vec2 normal;
    b2Vec2 point;
    float separation;
};

static
void 
b2PositionSolverManifoldInitialize(
    struct b2PositionSolverManifold* p,
    struct b2ContactPositionConstraint* pc, 
    const b2Transform xfA, 
    const b2Transform xfB, 
    int32 index)
{
    b2Assert(pc->pointCount > 0);

    switch (pc->type)
    {
    case b2ManifoldTypeCircles:
    {
        b2Vec2 v;
        b2Vec2 pointA;
        b2Vec2 pointB;

        b2TransformMulVec2(pointA, xfA, pc->localPoint);
        b2TransformMulVec2(pointB, xfB, pc->localPoints[0]);
        b2Vec2Sub(p->normal, pointB, pointA);
        b2Vec2Normalize(p->normal, p->normal);
        b2Vec2Add(v, pointA, pointB);
        b2Vec2Scale(p->point, v, 0.5f);
        b2Vec2Sub(v, pointB, pointA);
        p->separation = b2Vec2DotProduct(v, p->normal) - pc->radiusA - pc->radiusB;
    }
    break;

    case b2ManifoldTypeFaceA:
    {
        b2Vec2 v;
        b2Vec2 planePoint;
        b2Vec2 clipPoint;

        b2RotMulVec2(p->normal, xfA[1], pc->localNormal);
        b2TransformMulVec2(planePoint, xfA, pc->localPoint);

        b2TransformMulVec2(clipPoint, xfB, pc->localPoints[index]);
        b2Vec2Sub(v, clipPoint, planePoint);
        p->separation = b2Vec2DotProduct(v, p->normal) - pc->radiusA - pc->radiusB;
        b2Vec2Assign(p->point, clipPoint);
    }
    break;

    case b2ManifoldTypeFaceB:
    {
        b2Vec2 v;
        b2Vec2 planePoint;
        b2Vec2 clipPoint;

        b2RotMulVec2(p->normal, xfB[1], pc->localNormal);
        b2TransformMulVec2(planePoint, xfB, pc->localPoint);

        b2TransformMulVec2(clipPoint, xfA, pc->localPoints[index]);
        b2Vec2Sub(v, clipPoint, planePoint);
        p->separation = b2Vec2DotProduct(v, p->normal) - pc->radiusA - pc->radiusB;
        b2Vec2Assign(p->point, clipPoint);

        // Ensure normal points from A to B
        b2Vec2Negate(p->normal, p->normal);
    }
    break;
    }
}

B2_API
void
b2ContactSolverPrepare(
    struct b2ContactSolver* p,
    struct b2ContactSolverDef* def)
{
    int32 i, j;

    p->m_step = def->step;
    p->m_allocator = def->allocator;
    p->m_count = def->count;
    p->m_positionConstraints = (struct b2ContactPositionConstraint*)b2StackAllocatorAllocate(p->m_allocator, p->m_count * sizeof(struct b2ContactPositionConstraint));
    p->m_velocityConstraints = (struct b2ContactVelocityConstraint*)b2StackAllocatorAllocate(p->m_allocator, p->m_count * sizeof(struct b2ContactVelocityConstraint));
    p->m_positions = def->positions;
    p->m_velocities = def->velocities;
    p->m_contacts = def->contacts;

    // Initialize position independent portions of the constraints.
    for (i = 0; i < p->m_count; ++i)
    {
        struct b2Contact* contact;

        struct b2Fixture* fixtureA;
        struct b2Fixture* fixtureB;
        struct b2Shape* shapeA;
        struct b2Shape* shapeB;
        float radiusA;
        float radiusB;
        struct b2Body* bodyA;
        struct b2Body* bodyB;
        struct b2Manifold* manifold;

        int32 pointCount;

        struct b2ContactVelocityConstraint* vc;

        struct b2ContactPositionConstraint* pc;

        contact = p->m_contacts[i];

        fixtureA = contact->m_fixtureA;
        fixtureB = contact->m_fixtureB;
        shapeA = b2FixtureGetShapeRef(fixtureA);
        shapeB = b2FixtureGetShapeRef(fixtureB);
        radiusA = shapeA->m_radius;
        radiusB = shapeB->m_radius;
        bodyA = b2FixtureGetBodyRef(fixtureA);
        bodyB = b2FixtureGetBodyRef(fixtureB);
        manifold = b2ContactGetManifoldRef(contact);

        pointCount = manifold->pointCount;
        b2Assert(pointCount > 0);

        vc = p->m_velocityConstraints + i;
        vc->friction = contact->m_friction;
        vc->restitution = contact->m_restitution;
        vc->threshold = contact->m_restitutionThreshold;
        vc->tangentSpeed = contact->m_tangentSpeed;
        vc->indexA = bodyA->m_islandIndex;
        vc->indexB = bodyB->m_islandIndex;
        vc->invMassA = bodyA->m_invMass;
        vc->invMassB = bodyB->m_invMass;
        vc->invIA = bodyA->m_invI;
        vc->invIB = bodyB->m_invI;
        vc->contactIndex = i;
        vc->pointCount = pointCount;
        b2Mat22SetZero(vc->K);
        b2Mat22SetZero(vc->normalMass);

        pc = p->m_positionConstraints + i;
        pc->indexA = bodyA->m_islandIndex;
        pc->indexB = bodyB->m_islandIndex;
        pc->invMassA = bodyA->m_invMass;
        pc->invMassB = bodyB->m_invMass;
        b2Vec2Assign(pc->localCenterA, bodyA->m_sweep.localCenter);
        b2Vec2Assign(pc->localCenterB, bodyB->m_sweep.localCenter);
        pc->invIA = bodyA->m_invI;
        pc->invIB = bodyB->m_invI;
        b2Vec2Assign(pc->localNormal, manifold->localNormal);
        b2Vec2Assign(pc->localPoint, manifold->localPoint);
        pc->pointCount = pointCount;
        pc->radiusA = radiusA;
        pc->radiusB = radiusB;
        pc->type = manifold->type;

        for (j = 0; j < pointCount; ++j)
        {
            struct b2ManifoldPoint* cp;
            struct b2VelocityConstraintPoint* vcp;

            cp = manifold->points + j;
            vcp = vc->points + j;

            if (p->m_step.warmStarting)
            {
                vcp->normalImpulse = p->m_step.dtRatio * cp->normalImpulse;
                vcp->tangentImpulse = p->m_step.dtRatio * cp->tangentImpulse;
            }
            else
            {
                vcp->normalImpulse = 0.0f;
                vcp->tangentImpulse = 0.0f;
            }

            b2Vec2SetZero(vcp->rA);
            b2Vec2SetZero(vcp->rB);
            vcp->normalMass = 0.0f;
            vcp->tangentMass = 0.0f;
            vcp->velocityBias = 0.0f;

            b2Vec2Assign(pc->localPoints[j], cp->localPoint);
        }
    }
}

B2_API
void
b2ContactSolverDiscard(
    struct b2ContactSolver* p)
{
    b2StackAllocatorFree(p->m_allocator, p->m_velocityConstraints);
    b2StackAllocatorFree(p->m_allocator, p->m_positionConstraints);
}

// Initialize position dependent portions of the velocity constraints.
B2_API
void
b2ContactSolverInitializeVelocityConstraints(
    struct b2ContactSolver* p)
{
    int32 i, j;

    for (i = 0; i < p->m_count; ++i)
    {
        b2Vec2 v;
        b2Vec2 v1, v2;

        struct b2ContactVelocityConstraint* vc;
        struct b2ContactPositionConstraint* pc;

        float radiusA;
        float radiusB;
        struct b2Manifold* manifold;

        int32 indexA;
        int32 indexB;

        float mA;
        float mB;
        float iA;
        float iB;
        b2Vec2ConstRef localCenterA;
        b2Vec2ConstRef localCenterB;

        b2Vec2 cA;
        float aA;
        b2Vec2 vA;
        float wA;

        b2Vec2 cB;
        float aB;
        b2Vec2 vB;
        float wB;

        b2Transform xfA, xfB;

        struct b2WorldManifold worldManifold;

        int32 pointCount;

        vc = p->m_velocityConstraints + i;
        pc = p->m_positionConstraints + i;

        radiusA = pc->radiusA;
        radiusB = pc->radiusB;
        manifold = b2ContactGetManifoldRef(p->m_contacts[vc->contactIndex]);

        indexA = vc->indexA;
        indexB = vc->indexB;

        mA = vc->invMassA;
        mB = vc->invMassB;
        iA = vc->invIA;
        iB = vc->invIB;
        localCenterA = pc->localCenterA;
        localCenterB = pc->localCenterB;

        b2Vec2Assign(cA, p->m_positions[indexA].c);
        aA = p->m_positions[indexA].a;
        b2Vec2Assign(vA, p->m_velocities[indexA].v);
        wA = p->m_velocities[indexA].w;

        b2Vec2Assign(cB, p->m_positions[indexB].c);
        aB = p->m_positions[indexB].a;
        b2Vec2Assign(vB, p->m_velocities[indexB].v);
        wB = p->m_velocities[indexB].w;

        b2Assert(manifold->pointCount > 0);

        b2RotFromAngle(xfA[1], aA);
        b2RotFromAngle(xfB[1], aB);
        b2RotMulVec2(v, xfA[1], localCenterA);
        b2Vec2Sub(xfA[0], cA, v);
        b2RotMulVec2(v, xfB[1], localCenterB);
        b2Vec2Sub(xfB[0], cB, v);

        b2WorldManifoldInitialize(&worldManifold, manifold, xfA, radiusA, xfB, radiusB);

        b2Vec2Assign(vc->normal, worldManifold.normal);

        pointCount = vc->pointCount;
        for (j = 0; j < pointCount; ++j)
        {
            struct b2VelocityConstraintPoint* vcp;

            float rnA;
            float rnB;

            float kNormal;

            b2Vec2 tangent;

            float rtA;
            float rtB;

            float kTangent;

            float vRel;

            vcp = vc->points + j;

            b2Vec2Sub(vcp->rA, worldManifold.points[j], cA);
            b2Vec2Sub(vcp->rB, worldManifold.points[j], cB);

            rnA = b2Vec2CrossProduct(vcp->rA, vc->normal);
            rnB = b2Vec2CrossProduct(vcp->rB, vc->normal);

            kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

            vcp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

            b2Vec2CrossProductKR(tangent, vc->normal, 1.0f);

            rtA = b2Vec2CrossProduct(vcp->rA, tangent);
            rtB = b2Vec2CrossProduct(vcp->rB, tangent);

            kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

            vcp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

            // Setup a velocity bias for restitution.
            vcp->velocityBias = 0.0f;
            b2Vec2CrossProductKL(v1, wB, vcp->rB);
            b2Vec2CrossProductKL(v2, wA, vcp->rA);
            b2Vec2Add(v, vB, v1);
            b2Vec2Sub(v, v, vA);
            b2Vec2Sub(v, v, v2);
            vRel = b2Vec2DotProduct(vc->normal, v);
            if (vRel < -vc->threshold)
            {
                vcp->velocityBias = -vc->restitution * vRel;
            }
        }

        // If we have two points, then prepare the block solver.
        if (vc->pointCount == 2 && g_blockSolve)
        {
            struct b2VelocityConstraintPoint* vcp1 = vc->points + 0;
            struct b2VelocityConstraintPoint* vcp2 = vc->points + 1;

            float rn1A = b2Vec2CrossProduct(vcp1->rA, vc->normal);
            float rn1B = b2Vec2CrossProduct(vcp1->rB, vc->normal);
            float rn2A = b2Vec2CrossProduct(vcp2->rA, vc->normal);
            float rn2B = b2Vec2CrossProduct(vcp2->rB, vc->normal);

            float k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
            float k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
            float k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

            // Ensure a reasonable condition number.
            const float k_maxConditionNumber = 1000.0f;
            if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
            {
                // K is safe to invert.
                b2Vec2Make(vc->K[0], k11, k12);
                b2Vec2Make(vc->K[1], k12, k22);
                b2Mat22GetInverse(vc->normalMass, vc->K);
            }
            else
            {
                // The constraints are redundant, just use one.
                // TODO_ERIN use deepest?
                vc->pointCount = 1;
            }
        }
    }
}

B2_API
void
b2ContactSolverWarmStart(
    struct b2ContactSolver* p)
{
    int32 i, j;

    // Warm start.
    for (i = 0; i < p->m_count; ++i)
    {
        b2Vec2 v;
        b2Vec2 v1, v2;

        struct b2ContactVelocityConstraint* vc;

        int32 indexA;
        int32 indexB;
        float mA;
        float iA;
        float mB;
        float iB;
        int32 pointCount;

        b2Vec2 vA;
        float wA;
        b2Vec2 vB;
        float wB;

        b2Vec2 normal;
        b2Vec2 tangent;

        vc = p->m_velocityConstraints + i;

        indexA = vc->indexA;
        indexB = vc->indexB;
        mA = vc->invMassA;
        iA = vc->invIA;
        mB = vc->invMassB;
        iB = vc->invIB;
        pointCount = vc->pointCount;

        b2Vec2Assign(vA, p->m_velocities[indexA].v);
        wA = p->m_velocities[indexA].w;
        b2Vec2Assign(vB, p->m_velocities[indexB].v);
        wB = p->m_velocities[indexB].w;

        b2Vec2Assign(normal, vc->normal);
        b2Vec2CrossProductKR(tangent, normal, 1.0f);

        for (j = 0; j < pointCount; ++j)
        {
            struct b2VelocityConstraintPoint* vcp;
            b2Vec2 P;

            vcp = vc->points + j;
            b2Vec2Scale(v1, normal, vcp->normalImpulse);
            b2Vec2Scale(v2, tangent, vcp->tangentImpulse);
            b2Vec2Add(P, v1, v2);
            wA -= iA * b2Vec2CrossProduct(vcp->rA, P);
            b2Vec2Scale(v, P, mA);
            b2Vec2Sub(vA, vA, v);
            wB += iB * b2Vec2CrossProduct(vcp->rB, P);
            b2Vec2Scale(v, P, mB);
            b2Vec2Add(vB, vB, v);
        }

        b2Vec2Assign(p->m_velocities[indexA].v, vA);
        p->m_velocities[indexA].w = wA;
        b2Vec2Assign(p->m_velocities[indexB].v, vB);
        p->m_velocities[indexB].w = wB;
    }
}

B2_API
void
b2ContactSolverSolveVelocityConstraints(
    struct b2ContactSolver* p)
{
    int32 i, j;

    for (i = 0; i < p->m_count; ++i)
    {
        b2Vec2 v;
        b2Vec2 v1, v2;

        struct b2ContactVelocityConstraint* vc;

        int32 indexA;
        int32 indexB;
        float mA;
        float iA;
        float mB;
        float iB;
        int32 pointCount;

        b2Vec2 vA;
        float wA;
        b2Vec2 vB;
        float wB;

        b2Vec2 normal;
        b2Vec2 tangent;
        float friction;

        vc = p->m_velocityConstraints + i;

        indexA = vc->indexA;
        indexB = vc->indexB;
        mA = vc->invMassA;
        iA = vc->invIA;
        mB = vc->invMassB;
        iB = vc->invIB;
        pointCount = vc->pointCount;

        b2Vec2Assign(vA, p->m_velocities[indexA].v);
        wA = p->m_velocities[indexA].w;
        b2Vec2Assign(vB, p->m_velocities[indexB].v);
        wB = p->m_velocities[indexB].w;

        b2Vec2Assign(normal, vc->normal);
        b2Vec2CrossProductKR(tangent, normal, 1.0f);
        friction = vc->friction;

        b2Assert(pointCount == 1 || pointCount == 2);

        // Solve tangent constraints first because non-penetration is more important
        // than friction.
        for (j = 0; j < pointCount; ++j)
        {
            struct b2VelocityConstraintPoint* vcp;

            b2Vec2 dv;

            float vt;
            float lambda;

            float maxFriction;
            float newImpulse;

            b2Vec2 P;

            vcp = vc->points + j;

            // Relative velocity at contact
            b2Vec2CrossProductKL(v1, wB, vcp->rB);
            b2Vec2CrossProductKL(v2, wA, vcp->rA);
            b2Vec2Add(v, vB, v1);
            b2Vec2Sub(v, v, vA);
            b2Vec2Sub(dv, v, v2);

            // Compute tangent force
            vt = b2Vec2DotProduct(dv, tangent) - vc->tangentSpeed;
            lambda = vcp->tangentMass * (-vt);

            // b2Clamp the accumulated force
            maxFriction = friction * vcp->normalImpulse;
            newImpulse = b2ClampFloat(vcp->tangentImpulse + lambda, -maxFriction, maxFriction);
            lambda = newImpulse - vcp->tangentImpulse;
            vcp->tangentImpulse = newImpulse;

            // Apply contact impulse
            b2Vec2Scale(P, tangent, lambda);

            b2Vec2Scale(v, P, mA);
            b2Vec2Sub(vA, vA, v);
            wA -= iA * b2Vec2CrossProduct(vcp->rA, P);

            b2Vec2Scale(v, P, mB);
            b2Vec2Add(vB, vB, v);
            wB += iB * b2Vec2CrossProduct(vcp->rB, P);
        }

        // Solve normal constraints
        if (pointCount == 1 || g_blockSolve == b2False)
        {
            for (j = 0; j < pointCount; ++j)
            {
                struct b2VelocityConstraintPoint* vcp;

                b2Vec2 dv;

                float vn;
                float lambda;

                float newImpulse;

                b2Vec2 P;

                vcp = vc->points + j;

                // Relative velocity at contact
                b2Vec2CrossProductKL(v1, wB, vcp->rB);
                b2Vec2CrossProductKL(v2, wA, vcp->rA);
                b2Vec2Add(v, vB, v1);
                b2Vec2Sub(v, v, vA);
                b2Vec2Sub(dv, v, v2);

                // Compute normal impulse
                vn = b2Vec2DotProduct(dv, normal);
                lambda = -vcp->normalMass * (vn - vcp->velocityBias);

                // b2Clamp the accumulated impulse
                newImpulse = b2MaxFloat(vcp->normalImpulse + lambda, 0.0f);
                lambda = newImpulse - vcp->normalImpulse;
                vcp->normalImpulse = newImpulse;

                // Apply contact impulse
                b2Vec2Scale(P, normal, lambda);

                b2Vec2Scale(v, P, mA);
                b2Vec2Sub(vA, vA, v);
                wA -= iA * b2Vec2CrossProduct(vcp->rA, P);

                b2Vec2Scale(v, P, mB);
                b2Vec2Add(vB, vB, v);
                wB += iB * b2Vec2CrossProduct(vcp->rB, P);
            }
        }
        else
        {
            // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
            // Build the mini LCP for this contact patch
            //
            // vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
            //
            // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
            // b = vn0 - velocityBias
            //
            // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
            // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
            // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
            // solution that satisfies the problem is chosen.
            // 
            // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
            // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
            //
            // Substitute:
            // 
            // x = a + d
            // 
            // a := old total impulse
            // x := new total impulse
            // d := incremental impulse 
            //
            // For the current iteration we extend the formula for the incremental impulse
            // to compute the new total impulse:
            //
            // vn = A * d + b
            //    = A * (x - a) + b
            //    = A * x + b - A * a
            //    = A * x + b'
            // b' = b - A * a;

            struct b2VelocityConstraintPoint* cp1;
            struct b2VelocityConstraintPoint* cp2;

            b2Vec2 a;

            b2Vec2 dv1;
            b2Vec2 dv2;

            float vn1;
            float vn2;

            b2Vec2 b;

            const float k_errorTol = 1e-3f;
            B2_NOT_USED(k_errorTol);

            cp1 = vc->points + 0;
            cp2 = vc->points + 1;

            a[0] = cp1->normalImpulse;
            a[1] = cp2->normalImpulse;
            b2Assert(a[0] >= 0.0f && a[1] >= 0.0f);

            // Relative velocity at contact
            b2Vec2CrossProductKL(v1, wB, cp1->rB);
            b2Vec2CrossProductKL(v2, wA, cp1->rA);
            b2Vec2Add(v, vB, v1);
            b2Vec2Sub(v, v, vA);
            b2Vec2Sub(dv1, v, v2);
            b2Vec2CrossProductKL(v1, wB, cp2->rB);
            b2Vec2CrossProductKL(v2, wA, cp2->rA);
            b2Vec2Add(v, vB, v1);
            b2Vec2Sub(v, v, vA);
            b2Vec2Sub(dv2, v, v2);

            // Compute normal velocity
            vn1 = b2Vec2DotProduct(dv1, normal);
            vn2 = b2Vec2DotProduct(dv2, normal);

            b[0] = vn1 - cp1->velocityBias;
            b[1] = vn2 - cp2->velocityBias;

            // Compute b'
            b2Mat22MulVec2(v, vc->K, a);
            b2Vec2Sub(b, b, v);

            for (;;)
            {
                //
                // Case 1: vn = 0
                //
                // 0 = A * x + b'
                //
                // Solve for x:
                //
                // x = - inv(A) * b'
                //

                b2Vec2 x;

                b2Mat22MulVec2(x, vc->normalMass, b);
                b2Vec2Negate(x, x);

                if (x[0] >= 0.0f && x[1] >= 0.0f)
                {
                    b2Vec2 d;

                    b2Vec2 P1;
                    b2Vec2 P2;
                    b2Vec2 P;

                    // Get the incremental impulse
                    b2Vec2Sub(d, x, a);

                    // Apply incremental impulse
                    b2Vec2Scale(P1, normal, d[0]);
                    b2Vec2Scale(P2, normal, d[1]);
                    b2Vec2Add(P, P1, P2);

                    b2Vec2Scale(v, P, mA);
                    b2Vec2Sub(vA, vA, v);
                    wA -= iA * (b2Vec2CrossProduct(cp1->rA, P1) + b2Vec2CrossProduct(cp2->rA, P2));

                    b2Vec2Scale(v, P, mB);
                    b2Vec2Add(vB, vB, v);
                    wB += iB * (b2Vec2CrossProduct(cp1->rB, P1) + b2Vec2CrossProduct(cp2->rB, P2));

                    // Accumulate
                    cp1->normalImpulse = x[0];
                    cp2->normalImpulse = x[1];

#if B2_DEBUG_SOLVER == 1
                    // Postconditions
                    b2Vec2CrossProductKL(v1, wB, cp1->rB);
                    b2Vec2CrossProductKL(v2, wA, cp1->rA);
                    b2Vec2Add(v, vB, v1);
                    b2Vec2Sub(v, v, vA);
                    b2Vec2Sub(dv1, v, v2);
                    b2Vec2CrossProductKL(v1, wB, cp2->rB);
                    b2Vec2CrossProductKL(v2, wA, cp2->rA);
                    b2Vec2Add(v, vB, v1);
                    b2Vec2Sub(v, v, vA);
                    b2Vec2Sub(dv2, v, v2);

                    // Compute normal velocity
                    vn1 = b2Vec2DotProduct(dv1, normal);
                    vn2 = b2Vec2DotProduct(dv2, normal);

                    b2Assert(b2AbsFloat(vn1 - cp1->velocityBias) < k_errorTol);
                    b2Assert(b2AbsFloat(vn2 - cp2->velocityBias) < k_errorTol);
#endif
                    break;
                }

                //
                // Case 2: vn1 = 0 and x2 = 0
                //
                //   0 = a11 * x1 + a12 * 0 + b1' 
                // vn2 = a21 * x1 + a22 * 0 + b2'
                //
                x[0] = -cp1->normalMass * b[0];
                x[1] = 0.0f;
                vn1 = 0.0f;
                vn2 = vc->K[0][1] * x[0] + b[1];
                if (x[0] >= 0.0f && vn2 >= 0.0f)
                {
                    b2Vec2 d;

                    b2Vec2 P1;
                    b2Vec2 P2;
                    b2Vec2 P;

                    // Get the incremental impulse
                    b2Vec2Sub(d, x, a);

                    // Apply incremental impulse
                    b2Vec2Scale(P1, normal, d[0]);
                    b2Vec2Scale(P2, normal, d[1]);
                    b2Vec2Add(P, P1, P2);

                    b2Vec2Scale(v, P, mA);
                    b2Vec2Sub(vA, vA, v);
                    wA -= iA * (b2Vec2CrossProduct(cp1->rA, P1) + b2Vec2CrossProduct(cp2->rA, P2));

                    b2Vec2Scale(v, P, mB);
                    b2Vec2Add(vB, vB, v);
                    wB += iB * (b2Vec2CrossProduct(cp1->rB, P1) + b2Vec2CrossProduct(cp2->rB, P2));

                    // Accumulate
                    cp1->normalImpulse = x[0];
                    cp2->normalImpulse = x[1];

#if B2_DEBUG_SOLVER == 1
                    // Postconditions
                    b2Vec2CrossProductKL(v1, wB, cp1->rB);
                    b2Vec2CrossProductKL(v2, wA, cp1->rA);
                    b2Vec2Add(v, vB, v1);
                    b2Vec2Sub(v, v, vA);
                    b2Vec2Sub(dv1, v, v2);

                    // Compute normal velocity
                    vn1 = b2Vec2DotProduct(dv1, normal);

                    b2Assert(b2AbsFloat(vn1 - cp1->velocityBias) < k_errorTol);
#endif
                    break;
                }


                //
                // Case 3: vn2 = 0 and x1 = 0
                //
                // vn1 = a11 * 0 + a12 * x2 + b1' 
                //   0 = a21 * 0 + a22 * x2 + b2'
                //
                x[0] = 0.0f;
                x[1] = -cp2->normalMass * b[1];
                vn1 = vc->K[1][0] * x[1] + b[0];
                vn2 = 0.0f;

                if (x[1] >= 0.0f && vn1 >= 0.0f)
                {
                    b2Vec2 d;

                    b2Vec2 P1;
                    b2Vec2 P2;
                    b2Vec2 P;

                    // Resubstitute for the incremental impulse
                    b2Vec2Sub(d, x, a);

                    // Apply incremental impulse
                    b2Vec2Scale(P1, normal, d[0]);
                    b2Vec2Scale(P2, normal, d[1]);
                    b2Vec2Add(P, P1, P2);

                    b2Vec2Scale(v, P, mA);
                    b2Vec2Sub(vA, vA, v);
                    wA -= iA * (b2Vec2CrossProduct(cp1->rA, P1) + b2Vec2CrossProduct(cp2->rA, P2));

                    b2Vec2Scale(v, P, mB);
                    b2Vec2Add(vB, vB, v);
                    wB += iB * (b2Vec2CrossProduct(cp1->rB, P1) + b2Vec2CrossProduct(cp2->rB, P2));

                    // Accumulate
                    cp1->normalImpulse = x[0];
                    cp2->normalImpulse = x[1];

#if B2_DEBUG_SOLVER == 1
                    // Postconditions
                    b2Vec2CrossProductKL(v1, wB, cp2->rB);
                    b2Vec2CrossProductKL(v2, wA, cp2->rA);
                    b2Vec2Add(v, vB, v1);
                    b2Vec2Sub(v, v, vA);
                    b2Vec2Sub(dv2, v, v2);

                    // Compute normal velocity
                    vn2 = b2Vec2DotProduct(dv2, normal);

                    b2Assert(b2AbsFloat(vn2 - cp2->velocityBias) < k_errorTol);
#endif
                    break;
                }

                //
                // Case 4: x1 = 0 and x2 = 0
                // 
                // vn1 = b1
                // vn2 = b2;
                x[0] = 0.0f;
                x[1] = 0.0f;
                vn1 = b[0];
                vn2 = b[1];

                if (vn1 >= 0.0f && vn2 >= 0.0f)
                {
                    b2Vec2 d;

                    b2Vec2 P1;
                    b2Vec2 P2;
                    b2Vec2 P;

                    // Resubstitute for the incremental impulse
                    b2Vec2Sub(d, x, a);

                    // Apply incremental impulse
                    b2Vec2Scale(P1, normal, d[0]);
                    b2Vec2Scale(P2, normal, d[1]);
                    b2Vec2Add(P, P1, P2);

                    b2Vec2Scale(v, P, mA);
                    b2Vec2Sub(vA, vA, v);
                    wA -= iA * (b2Vec2CrossProduct(cp1->rA, P1) + b2Vec2CrossProduct(cp2->rA, P2));

                    b2Vec2Scale(v, P, mB);
                    b2Vec2Add(vB, vB, v);
                    wB += iB * (b2Vec2CrossProduct(cp1->rB, P1) + b2Vec2CrossProduct(cp2->rB, P2));

                    // Accumulate
                    cp1->normalImpulse = x[0];
                    cp2->normalImpulse = x[1];

                    break;
                }

                // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                break;
            }
        }

        b2Vec2Assign(p->m_velocities[indexA].v, vA);
        p->m_velocities[indexA].w = wA;
        b2Vec2Assign(p->m_velocities[indexB].v, vB);
        p->m_velocities[indexB].w = wB;
    }
}

B2_API
void
b2ContactSolverStoreImpulses(
    struct b2ContactSolver* p)
{
    int32 i, j;
    for (i = 0; i < p->m_count; ++i)
    {
        struct b2ContactVelocityConstraint* vc;
        struct b2Manifold* manifold;

        vc = p->m_velocityConstraints + i;
        manifold = b2ContactGetManifoldRef(p->m_contacts[vc->contactIndex]);

        for (j = 0; j < vc->pointCount; ++j)
        {
            manifold->points[j].normalImpulse = vc->points[j].normalImpulse;
            manifold->points[j].tangentImpulse = vc->points[j].tangentImpulse;
        }
    }
}

// Sequential solver.
B2_API
int
b2ContactSolverSolvePositionConstraints(
    struct b2ContactSolver* p)
{
    int32 i, j;

    float minSeparation = 0.0f;

    for (i = 0; i < p->m_count; ++i)
    {
        b2Vec2 v;

        struct b2ContactPositionConstraint* pc;

        int32 indexA;
        int32 indexB;
        b2Vec2 localCenterA;
        float mA;
        float iA;
        b2Vec2 localCenterB;
        float mB;
        float iB;
        int32 pointCount;

        b2Vec2 cA;
        float aA;

        b2Vec2 cB;
        float aB;

        pc = p->m_positionConstraints + i;

        indexA = pc->indexA;
        indexB = pc->indexB;
        b2Vec2Assign(localCenterA, pc->localCenterA);
        mA = pc->invMassA;
        iA = pc->invIA;
        b2Vec2Assign(localCenterB, pc->localCenterB);
        mB = pc->invMassB;
        iB = pc->invIB;
        pointCount = pc->pointCount;

        b2Vec2Assign(cA, p->m_positions[indexA].c);
        aA = p->m_positions[indexA].a;

        b2Vec2Assign(cB, p->m_positions[indexB].c);
        aB = p->m_positions[indexB].a;

        // Solve normal constraints
        for (j = 0; j < pointCount; ++j)
        {
            b2Transform xfA, xfB;

            struct b2PositionSolverManifold psm;

            b2Vec2 normal;

            b2Vec2 point;
            float separation;

            b2Vec2 rA;
            b2Vec2 rB;

            float C;

            float rnA;
            float rnB;
            float K;

            float impulse;

            b2Vec2 P;

            b2RotFromAngle(xfA[1], aA);
            b2RotFromAngle(xfB[1], aB);
            b2RotMulVec2(v, xfA[1], localCenterA);
            b2Vec2Sub(xfA[0], cA, v);
            b2RotMulVec2(v, xfB[1], localCenterB);
            b2Vec2Sub(xfB[0], cB, v);

            b2PositionSolverManifoldInitialize(&psm, pc, xfA, xfB, j);
            b2Vec2Assign(normal, psm.normal);

            b2Vec2Assign(point, psm.point);
            separation = psm.separation;

            b2Vec2Sub(rA, point, cA);
            b2Vec2Sub(rB, point, cB);

            // Track max constraint error.
            minSeparation = b2MinFloat(minSeparation, separation);

            // Prevent large corrections and allow slop.
            C = b2ClampFloat(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

            // Compute the effective mass.
            rnA = b2Vec2CrossProduct(rA, normal);
            rnB = b2Vec2CrossProduct(rB, normal);
            K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

            // Compute normal impulse
            impulse = K > 0.0f ? -C / K : 0.0f;

            b2Vec2Scale(P, normal, impulse);

            b2Vec2Scale(v, P, mA);
            b2Vec2Sub(cA, cA, v);
            aA -= iA * b2Vec2CrossProduct(rA, P);

            b2Vec2Scale(v, P, mB);
            b2Vec2Add(cB, cB, v);
            aB += iB * b2Vec2CrossProduct(rB, P);
        }

        b2Vec2Assign(p->m_positions[indexA].c, cA);
        p->m_positions[indexA].a = aA;

        b2Vec2Assign(p->m_positions[indexB].c, cB);
        p->m_positions[indexB].a = aB;
    }

    // We can't expect minSpeparation >= -b2_linearSlop because we don't
    // push the separation above -b2_linearSlop.
    return minSeparation >= -3.0f * b2_linearSlop;
}

// Sequential position solver for position constraints.
B2_API
int
b2ContactSolverSolveTOIPositionConstraints(
    struct b2ContactSolver* p,
    int32 toiIndexA,
    int32 toiIndexB)
{
    int32 i, j;

    float minSeparation = 0.0f;

    for (i = 0; i < p->m_count; ++i)
    {
        b2Vec2 v;

        struct b2ContactPositionConstraint* pc;

        int32 indexA;
        int32 indexB;
        b2Vec2 localCenterA;
        b2Vec2 localCenterB;
        int32 pointCount;

        float mA;
        float iA;

        float mB;
        float iB;;

        b2Vec2 cA;
        float aA;

        b2Vec2 cB;
        float aB;

        pc = p->m_positionConstraints + i;

        indexA = pc->indexA;
        indexB = pc->indexB;
        b2Vec2Assign(localCenterA, pc->localCenterA);
        b2Vec2Assign(localCenterB, pc->localCenterB);
        pointCount = pc->pointCount;

        mA = 0.0f;
        iA = 0.0f;
        if (indexA == toiIndexA || indexA == toiIndexB)
        {
            mA = pc->invMassA;
            iA = pc->invIA;
        }

        mB = 0.0f;
        iB = 0.0f;
        if (indexB == toiIndexA || indexB == toiIndexB)
        {
            mB = pc->invMassB;
            iB = pc->invIB;
        }

        b2Vec2Assign(cA, p->m_positions[indexA].c);
        aA = p->m_positions[indexA].a;

        b2Vec2Assign(cB, p->m_positions[indexB].c);
        aB = p->m_positions[indexB].a;

        // Solve normal constraints
        for (j = 0; j < pointCount; ++j)
        {
            b2Transform xfA, xfB;

            struct b2PositionSolverManifold psm;

            b2Vec2 normal;

            b2Vec2 point;
            float separation;

            b2Vec2 rA;
            b2Vec2 rB;

            float C;

            float rnA;
            float rnB;
            float K;

            float impulse;

            b2Vec2 P;

            b2RotFromAngle(xfA[1], aA);
            b2RotFromAngle(xfB[1], aB);
            b2RotMulVec2(v, xfA[1], localCenterA);
            b2Vec2Sub(xfA[0], cA, v);
            b2RotMulVec2(v, xfB[1], localCenterB);
            b2Vec2Sub(xfB[0], cB, v);

            b2PositionSolverManifoldInitialize(&psm, pc, xfA, xfB, j);
            b2Vec2Assign(normal, psm.normal);

            b2Vec2Assign(point, psm.point);
            separation = psm.separation;

            b2Vec2Sub(rA, point, cA);
            b2Vec2Sub(rB, point, cB);

            // Track max constraint error.
            minSeparation = b2MinFloat(minSeparation, separation);

            // Prevent large corrections and allow slop.
            C = b2ClampFloat(b2_toiBaumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

            // Compute the effective mass.
            rnA = b2Vec2CrossProduct(rA, normal);
            rnB = b2Vec2CrossProduct(rB, normal);
            K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

            // Compute normal impulse
            impulse = K > 0.0f ? -C / K : 0.0f;

            b2Vec2Scale(P, normal, impulse);

            b2Vec2Scale(v, P, mA);
            b2Vec2Sub(cA, cA, v);
            aA -= iA * b2Vec2CrossProduct(rA, P);

            b2Vec2Scale(v, P, mB);
            b2Vec2Add(cB, cB, v);
            aB += iB * b2Vec2CrossProduct(rB, P);
        }

        b2Vec2Assign(p->m_positions[indexA].c, cA);
        p->m_positions[indexA].a = aA;

        b2Vec2Assign(p->m_positions[indexB].c, cB);
        p->m_positions[indexB].a = aB;
    }

    // We can't expect minSpeparation >= -b2_linearSlop because we don't
    // push the separation above -b2_linearSlop.
    return minSeparation >= -1.5f * b2_linearSlop;
}

