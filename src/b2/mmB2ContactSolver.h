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

#ifndef __mmB2ContactSolver_h__
#define __mmB2ContactSolver_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Common.h"
#include "b2/mmB2TimeStep.h"

#include "b2/mmB2Prefix.h"

B2_API extern int g_blockSolve;

struct b2VelocityConstraintPoint
{
    b2Vec2 rA;
    b2Vec2 rB;
    float normalImpulse;
    float tangentImpulse;
    float normalMass;
    float tangentMass;
    float velocityBias;
};

struct b2ContactVelocityConstraint
{
    struct b2VelocityConstraintPoint points[b2_maxManifoldPoints];
    b2Vec2 normal;
    b2Mat22 normalMass;
    b2Mat22 K;
    int32 indexA;
    int32 indexB;
    float invMassA, invMassB;
    float invIA, invIB;
    float friction;
    float restitution;
    float threshold;
    float tangentSpeed;
    int32 pointCount;
    int32 contactIndex;
};

struct b2ContactSolverDef
{
    struct b2TimeStep step;
    struct b2Contact** contacts;
    int32 count;
    struct b2Position* positions;
    struct b2Velocity* velocities;
    struct b2StackAllocator* allocator;
};

struct b2ContactSolver
{
    struct b2TimeStep m_step;
    struct b2Position* m_positions;
    struct b2Velocity* m_velocities;
    struct b2StackAllocator* m_allocator;
    struct b2ContactPositionConstraint* m_positionConstraints;
    struct b2ContactVelocityConstraint* m_velocityConstraints;
    struct b2Contact** m_contacts;
    int m_count;
};

B2_API
void
b2ContactSolverPrepare(
    struct b2ContactSolver* p,
    struct b2ContactSolverDef* def);

B2_API
void
b2ContactSolverDiscard(
    struct b2ContactSolver* p);

B2_API
void
b2ContactSolverInitializeVelocityConstraints(
    struct b2ContactSolver* p);

B2_API
void
b2ContactSolverWarmStart(
    struct b2ContactSolver* p);

B2_API
void
b2ContactSolverSolveVelocityConstraints(
    struct b2ContactSolver* p);

B2_API
void
b2ContactSolverStoreImpulses(
    struct b2ContactSolver* p);

B2_API
int
b2ContactSolverSolvePositionConstraints(
    struct b2ContactSolver* p);

B2_API
int
b2ContactSolverSolveTOIPositionConstraints(
    struct b2ContactSolver* p,
    int32 toiIndexA, 
    int32 toiIndexB);

#include "b2/mmB2Suffix.h"

#endif//__mmB2ContactSolver_h__
