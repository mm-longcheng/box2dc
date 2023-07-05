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

#include "mmB2Island.h"
#include "mmB2Common.h"
#include "mmB2StackAllocator.h"
#include "mmB2Body.h"
#include "mmB2Joint.h"
#include "mmB2TimeStep.h"
#include "mmB2ContactSolver.h"
#include "mmB2Distance.h"
#include "mmB2WorldCallbacks.h"
#include "mmB2Timer.h"

#include <assert.h>
#include <stddef.h>

/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than b2_linearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

/*
Cache Performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.

The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/

/*
2D Rotation

R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
    [q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*/

B2_API
void
b2IslandPrepare(
    struct b2Island* p,
    int32 bodyCapacity,
    int32 contactCapacity,
    int32 jointCapacity,
    struct b2StackAllocator* allocator,
    struct b2ContactListener* listener)
{
    p->m_bodyCapacity = bodyCapacity;
    p->m_contactCapacity = contactCapacity;
    p->m_jointCapacity = jointCapacity;
    p->m_bodyCount = 0;
    p->m_contactCount = 0;
    p->m_jointCount = 0;

    p->m_allocator = allocator;
    p->m_listener = listener;

    p->m_bodies = (struct b2Body**)b2StackAllocatorAllocate(p->m_allocator, bodyCapacity * sizeof(struct b2Body*));
    p->m_contacts = (struct b2Contact**)b2StackAllocatorAllocate(p->m_allocator, contactCapacity * sizeof(struct b2Contact*));
    p->m_joints = (struct b2Joint**)b2StackAllocatorAllocate(p->m_allocator, jointCapacity * sizeof(struct b2Joint*));

    p->m_velocities = (struct b2Velocity*)b2StackAllocatorAllocate(p->m_allocator, p->m_bodyCapacity * sizeof(struct b2Velocity));
    p->m_positions = (struct b2Position*)b2StackAllocatorAllocate(p->m_allocator, p->m_bodyCapacity * sizeof(struct b2Position));
}

B2_API
void
b2IslandDiscard(
    struct b2Island* p)
{
    // Warning: the order should reverse the constructor order.
    b2StackAllocatorFree(p->m_allocator, p->m_positions);
    b2StackAllocatorFree(p->m_allocator, p->m_velocities);
    b2StackAllocatorFree(p->m_allocator, p->m_joints);
    b2StackAllocatorFree(p->m_allocator, p->m_contacts);
    b2StackAllocatorFree(p->m_allocator, p->m_bodies);
}

B2_API
void
b2IslandClear(
    struct b2Island* p)
{
    p->m_bodyCount = 0;
    p->m_contactCount = 0;
    p->m_jointCount = 0;
}

B2_API
void
b2IslandSolve(
    struct b2Island* p,
    struct b2Profile* profile,
    const struct b2TimeStep* step,
    const b2Vec2 gravity,
    int allowSleep)
{
    int32 i, j;

    b2Vec2 t;

    struct b2Timer timer;

    float h;

    struct b2SolverData solverData;

    struct b2ContactSolverDef contactSolverDef;
    struct b2ContactSolver contactSolver;

    int positionSolved;

    h = step->dt;

    b2TimerMake(&timer);

    // Integrate velocities and apply damping. Initialize the body state.
    for (i = 0; i < p->m_bodyCount; ++i)
    {
        struct b2Body* b;

        b2Vec2 c;
        float a;
        b2Vec2 v;
        float w;

        b = p->m_bodies[i];

        b2Vec2Assign(c, b->m_sweep.c);
        a = b->m_sweep.a;
        b2Vec2Assign(v, b->m_linearVelocity);
        w = b->m_angularVelocity;

        // Store positions for continuous collision.
        b2Vec2Assign(b->m_sweep.c0, b->m_sweep.c);
        b->m_sweep.a0 = b->m_sweep.a;

        if (b->m_type == b2BodyTypeDynamic)
        {
            // Integrate velocities.
            b2Vec2Scale(t, gravity, b->m_gravityScale * b->m_mass);
            b2Vec2Add(t, t, b->m_force);
            b2Vec2Scale(t, t, h * b->m_invMass);
            b2Vec2Add(v, v, t);
            w += h * b->m_invI * b->m_torque;

            // Apply damping.
            // ODE: dv/dt + c * v = 0
            // Solution: v(t) = v0 * exp(-c * t)
            // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
            // v2 = exp(-c * dt) * v1
            // Pade approximation:
            // v2 = v1 * 1 / (1 + c * dt)
            b2Vec2Scale(v, v, 1.0f / (1.0f + h * b->m_linearDamping));
            w *= 1.0f / (1.0f + h * b->m_angularDamping);
        }

        b2Vec2Assign(p->m_positions[i].c, c);
        p->m_positions[i].a = a;
        b2Vec2Assign(p->m_velocities[i].v, v);
        p->m_velocities[i].w = w;
    }

    b2TimerReset(&timer);

    // Solver data
    solverData.step = *step;
    solverData.positions = p->m_positions;
    solverData.velocities = p->m_velocities;

    // Initialize velocity constraints.
    contactSolverDef.step = *step;
    contactSolverDef.contacts = p->m_contacts;
    contactSolverDef.count = p->m_contactCount;
    contactSolverDef.positions = p->m_positions;
    contactSolverDef.velocities = p->m_velocities;
    contactSolverDef.allocator = p->m_allocator;

    b2ContactSolverPrepare(&contactSolver, &contactSolverDef);
    b2ContactSolverInitializeVelocityConstraints(&contactSolver);

    if (step->warmStarting)
    {
        b2ContactSolverWarmStart(&contactSolver);
    }

    for (i = 0; i < p->m_jointCount; ++i)
    {
        b2JointInitVelocityConstraints(p->m_joints[i], &solverData);
    }

    profile->solveInit = b2TimerGetMilliseconds(&timer);

    // Solve velocity constraints
    b2TimerReset(&timer);
    for (i = 0; i < step->velocityIterations; ++i)
    {
        for (j = 0; j < p->m_jointCount; ++j)
        {
            b2JointSolveVelocityConstraints(p->m_joints[j], &solverData);
        }

        b2ContactSolverSolveVelocityConstraints(&contactSolver);
    }

    // Store impulses for warm starting
    b2ContactSolverStoreImpulses(&contactSolver);
    profile->solveVelocity = b2TimerGetMilliseconds(&timer);

    // Integrate positions
    for (i = 0; i < p->m_bodyCount; ++i)
    {
        b2Vec2 c;
        float a;
        b2Vec2 v;
        float w;

        b2Vec2 translation;

        float rotation;

        b2Vec2Assign(c, p->m_positions[i].c);
        a = p->m_positions[i].a;
        b2Vec2Assign(v, p->m_velocities[i].v);
        w = p->m_velocities[i].w;

        // Check for large velocities
        b2Vec2Scale(translation, v, h);
        if (b2Vec2DotProduct(translation, translation) > b2_maxTranslationSquared)
        {
            float ratio = b2_maxTranslation / b2Vec2Length(translation);
            b2Vec2Scale(v, v, ratio);
        }

        rotation = h * w;
        if (rotation * rotation > b2_maxRotationSquared)
        {
            float ratio = b2_maxRotation / b2AbsFloat(rotation);
            w *= ratio;
        }

        // Integrate
        b2Vec2Scale(t, v, h);
        b2Vec2Add(c, c, t);
        a += h * w;

        b2Vec2Assign(p->m_positions[i].c, c);
        p->m_positions[i].a = a;
        b2Vec2Assign(p->m_velocities[i].v, v);
        p->m_velocities[i].w = w;
    }

    // Solve position constraints
    b2TimerReset(&timer);
    positionSolved = b2False;
    for (i = 0; i < step->positionIterations; ++i)
    {
        int contactsOkay;
        int jointsOkay;

        contactsOkay = b2ContactSolverSolvePositionConstraints(&contactSolver);

        jointsOkay = b2True;
        for (j = 0; j < p->m_jointCount; ++j)
        {
            int jointOkay = b2JointSolvePositionConstraints(p->m_joints[j], &solverData);
            jointsOkay = jointsOkay && jointOkay;
        }

        if (contactsOkay && jointsOkay)
        {
            // Exit early if the position errors are small.
            positionSolved = b2True;
            break;
        }
    }

    // Copy state buffers back to the bodies
    for (i = 0; i < p->m_bodyCount; ++i)
    {
        struct b2Body* body;
        body = p->m_bodies[i];
        b2Vec2Assign(body->m_sweep.c, p->m_positions[i].c);
        body->m_sweep.a = p->m_positions[i].a;
        b2Vec2Assign(body->m_linearVelocity, p->m_velocities[i].v);
        body->m_angularVelocity = p->m_velocities[i].w;
        b2BodySynchronizeTransform(body);
    }

    profile->solvePosition = b2TimerGetMilliseconds(&timer);

    b2IslandReport(p, contactSolver.m_velocityConstraints);

    if (allowSleep)
    {
        float minSleepTime = b2_maxFloat;

        const float linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
        const float angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;

        for (i = 0; i < p->m_bodyCount; ++i)
        {
            struct b2Body* b;

            b = p->m_bodies[i];
            if (b2BodyGetType(b) == b2BodyTypeStatic)
            {
                continue;
            }

            if ((b->m_flags & b2BodyFlagAutoSleep) == 0 ||
                b->m_angularVelocity * b->m_angularVelocity > angTolSqr ||
                b2Vec2DotProduct(b->m_linearVelocity, b->m_linearVelocity) > linTolSqr)
            {
                b->m_sleepTime = 0.0f;
                minSleepTime = 0.0f;
            }
            else
            {
                b->m_sleepTime += h;
                minSleepTime = b2MinFloat(minSleepTime, b->m_sleepTime);
            }
        }

        if (minSleepTime >= b2_timeToSleep && positionSolved)
        {
            for (i = 0; i < p->m_bodyCount; ++i)
            {
                struct b2Body* b;
                b = p->m_bodies[i];
                b2BodySetAwake(b, b2False);
            }
        }
    }

    b2ContactSolverDiscard(&contactSolver);
}

B2_API
void
b2IslandSolveTOI(
    struct b2Island* p,
    const struct b2TimeStep* subStep,
    int32 toiIndexA,
    int32 toiIndexB)
{
    int32 i;

    struct b2ContactSolverDef contactSolverDef;
    struct b2ContactSolver contactSolver;

    float h;

    b2Vec2 t;

    b2Assert(toiIndexA < p->m_bodyCount);
    b2Assert(toiIndexB < p->m_bodyCount);

    // Initialize the body state.
    for (i = 0; i < p->m_bodyCount; ++i)
    {
        struct b2Body* b;
        b = p->m_bodies[i];
        b2Vec2Assign(p->m_positions[i].c, b->m_sweep.c);
        p->m_positions[i].a = b->m_sweep.a;
        b2Vec2Assign(p->m_velocities[i].v, b->m_linearVelocity);
        p->m_velocities[i].w = b->m_angularVelocity;
    }

    contactSolverDef.contacts = p->m_contacts;
    contactSolverDef.count = p->m_contactCount;
    contactSolverDef.allocator = p->m_allocator;
    contactSolverDef.step = *subStep;
    contactSolverDef.positions = p->m_positions;
    contactSolverDef.velocities = p->m_velocities;

    b2ContactSolverPrepare(&contactSolver, &contactSolverDef);

    // Solve position constraints.
    for (i = 0; i < subStep->positionIterations; ++i)
    {
        int contactsOkay = b2ContactSolverSolveTOIPositionConstraints(&contactSolver, toiIndexA, toiIndexB);
        if (contactsOkay)
        {
            break;
        }
    }

#if 0
    // Is the new position really safe?
    for (i = 0; i < p->m_contactCount; ++i)
    {
        struct b2Contact* c;
        struct b2Fixture* fA;
        struct b2Fixture* fB;

        struct b2Body* bA;
        struct b2Body* bB;

        int32 indexA;
        int32 indexB;

        struct b2DistanceInput input;

        struct b2DistanceOutput output;
        struct b2SimplexCache cache;

        b2DistanceInputReset(&input);

        c = p->m_contacts[i];
        fA = b2ContactGetFixtureA(c);
        fB = b2ContactGetFixtureB(c);

        bA = b2FixtureGetBody(fA);
        bB = b2FixtureGetBody(fB);

        indexA = b2ContactGetChildIndexA(c);
        indexB = b2ContactGetChildIndexB(c);

        b2DistanceProxySetShape(&input.proxyA, b2FixtureGetShape(fA), indexA);
        b2DistanceProxySetShape(&input.proxyB, b2FixtureGetShape(fB), indexB);
        b2TransformAssign(input.transformA, b2BodyGetTransform(bA));
        b2TransformAssign(input.transformB, b2BodyGetTransform(bB));
        input.useRadii = b2False;

        cache.count = 0;
        b2Distance(&output, &cache, &input);

        if (output.distance == 0 || cache.count == 3)
        {
            cache.count += 0;
        }
    }
#endif

    // Leap of faith to new safe state.
    b2Vec2Assign(p->m_bodies[toiIndexA]->m_sweep.c0, p->m_positions[toiIndexA].c);
    p->m_bodies[toiIndexA]->m_sweep.a0 = p->m_positions[toiIndexA].a;
    b2Vec2Assign(p->m_bodies[toiIndexB]->m_sweep.c0, p->m_positions[toiIndexB].c);
    p->m_bodies[toiIndexB]->m_sweep.a0 = p->m_positions[toiIndexB].a;

    // No warm starting is needed for TOI events because warm
    // starting impulses were applied in the discrete solver.
    b2ContactSolverInitializeVelocityConstraints(&contactSolver);

    // Solve velocity constraints.
    for (i = 0; i < subStep->velocityIterations; ++i)
    {
        b2ContactSolverSolveVelocityConstraints(&contactSolver);
    }

    // Don't store the TOI contact forces for warm starting
    // because they can be quite large.

    h = subStep->dt;

    // Integrate positions
    for (i = 0; i < p->m_bodyCount; ++i)
    {
        b2Vec2 c;
        float a;
        b2Vec2 v;
        float w;

        b2Vec2 translation;

        float rotation;

        struct b2Body* body;

        b2Vec2Assign(c, p->m_positions[i].c);
        a = p->m_positions[i].a;
        b2Vec2Assign(v, p->m_velocities[i].v);
        w = p->m_velocities[i].w;

        // Check for large velocities
        b2Vec2Scale(translation, v, h);
        if (b2Vec2DotProduct(translation, translation) > b2_maxTranslationSquared)
        {
            float ratio = b2_maxTranslation / b2Vec2Length(translation);
            b2Vec2Scale(v, v, ratio);
        }

        rotation = h * w;
        if (rotation * rotation > b2_maxRotationSquared)
        {
            float ratio = b2_maxRotation / b2AbsFloat(rotation);
            w *= ratio;
        }

        // Integrate
        b2Vec2Scale(t, v, h);
        b2Vec2Add(c, c, t);
        a += h * w;

        b2Vec2Assign(p->m_positions[i].c, c);
        p->m_positions[i].a = a;
        b2Vec2Assign(p->m_velocities[i].v, v);
        p->m_velocities[i].w = w;

        // Sync bodies
        body = p->m_bodies[i];
        b2Vec2Assign(body->m_sweep.c, c);
        body->m_sweep.a = a;
        b2Vec2Assign(body->m_linearVelocity, v);
        body->m_angularVelocity = w;
        b2BodySynchronizeTransform(body);
    }

    b2IslandReport(p, contactSolver.m_velocityConstraints);

    b2ContactSolverDiscard(&contactSolver);
}

B2_API
void
b2IslandAddBody(
    struct b2Island* p,
    struct b2Body* body)
{
    b2Assert(p->m_bodyCount < p->m_bodyCapacity);
    body->m_islandIndex = p->m_bodyCount;
    p->m_bodies[p->m_bodyCount] = body;
    ++p->m_bodyCount;
}

B2_API
void
b2IslandAddContact(
    struct b2Island* p,
    struct b2Contact* contact)
{
    b2Assert(p->m_contactCount < p->m_contactCapacity);
    p->m_contacts[p->m_contactCount++] = contact;
}

B2_API
void
b2IslandAddJoint(
    struct b2Island* p,
    struct b2Joint* joint)
{
    b2Assert(p->m_jointCount < p->m_jointCapacity);
    p->m_joints[p->m_jointCount++] = joint;
}

B2_API
void
b2IslandReport(
    struct b2Island* p,
    const struct b2ContactVelocityConstraint* constraints)
{
    int32 i, j;

    if (p->m_listener == NULL)
    {
        return;
    }

    for (i = 0; i < p->m_contactCount; ++i)
    {
        struct b2Contact* c;
        const struct b2ContactVelocityConstraint* vc;
        struct b2ContactImpulse impulse;

        c = p->m_contacts[i];

        vc = constraints + i;

        impulse.count = vc->pointCount;
        for (j = 0; j < vc->pointCount; ++j)
        {
            impulse.normalImpulses[j] = vc->points[j].normalImpulse;
            impulse.tangentImpulses[j] = vc->points[j].tangentImpulse;
        }

        b2ContactListenerPostSolve(p->m_listener, c, &impulse);
    }
}
