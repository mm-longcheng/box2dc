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

#include "mmB2Joint.h"
#include "mmB2Common.h"
#include "mmB2Settings.h"
#include "mmB2Body.h"
#include "mmB2BlockAllocator.h"
#include "mmB2JointTypes.h"
#include "mmB2MetaAllocator.h"

#include <assert.h>
#include <string.h>

B2_API
void
b2JointDefReset(
    struct b2JointDef* p)
{
    p->type = b2JointTypeUnknown;
    p->userData = 0;
    p->bodyA = NULL;
    p->bodyB = NULL;
    p->collideConnected = b2False;
}

B2_API
void
b2LinearStiffness(
    float* stiffness, float* damping,
    float frequencyHertz, float dampingRatio,
    const struct b2Body* bodyA, const struct b2Body* bodyB)
{
    float mass;
    float massA = b2BodyGetMass(bodyA);
    float massB = b2BodyGetMass(bodyB);
    if (massA > 0.0f && massB > 0.0f)
    {
        mass = massA * massB / (massA + massB);
    }
    else if (massA > 0.0f)
    {
        mass = massA;
    }
    else
    {
        mass = massB;
    }

    float omega = 2.0f * b2_pi * frequencyHertz;
    (*stiffness) = mass * omega * omega;
    (*damping) = 2.0f * mass * dampingRatio * omega;
}

B2_API
void
b2AngularStiffness(
    float* stiffness, float* damping,
    float frequencyHertz, float dampingRatio,
    const struct b2Body* bodyA, const struct b2Body* bodyB)
{
    float I;
    float IA = b2BodyGetInertia(bodyA);
    float IB = b2BodyGetInertia(bodyB);
    if (IA > 0.0f && IB > 0.0f)
    {
        I = IA * IB / (IA + IB);
    }
    else if (IA > 0.0f)
    {
        I = IA;
    }
    else
    {
        I = IB;
    }

    float omega = 2.0f * b2_pi * frequencyHertz;
    (*stiffness) = I * omega * omega;
    (*damping) = 2.0f * I * dampingRatio * omega;
}

/// Dump this joint to the log file.
static
void 
b2JointDefaultDump(
    const void* obj) 
{ 
    b2Dump("// Dump is not supported for this joint type.\n"); 
}

/// Shift the origin for any points stored in world coordinates.
static
void 
b2JointDefaultShiftOrigin(
    void* obj, 
    const b2Vec2 newOrigin) 
{
    B2_NOT_USED(newOrigin); 
}

/// Debug draw this joint
static
void 
b2JointDefaultDraw(
    const void* obj, 
    struct b2Draw* draw)
{

}

B2_API const struct b2JointMeta b2JointMetaDefault =
{
    NULL,
    NULL,
    NULL,
    NULL,

    &b2JointDefaultDump,
    &b2JointDefaultShiftOrigin,
    &b2JointDefaultDraw,

    NULL,
    NULL,
    NULL,
};

B2_API
void
b2JointReset(
    struct b2Joint* p)
{
    memset(p, 0, sizeof(struct b2Joint));
    p->Meta = &b2JointMetaDefault;
}

B2_API
void
b2JointFromDef(
    struct b2Joint* p,
    const struct b2JointDef* def)
{
    b2Assert(def->bodyA != def->bodyB);

    p->m_type = def->type;
    p->m_prev = NULL;
    p->m_next = NULL;
    p->m_bodyA = def->bodyA;
    p->m_bodyB = def->bodyB;
    p->m_index = 0;
    p->m_collideConnected = def->collideConnected;
    p->m_islandFlag = b2False;
    p->m_userData = def->userData;

    p->m_edgeA.joint = NULL;
    p->m_edgeA.other = NULL;
    p->m_edgeA.prev = NULL;
    p->m_edgeA.next = NULL;

    p->m_edgeB.joint = NULL;
    p->m_edgeB.other = NULL;
    p->m_edgeB.prev = NULL;
    p->m_edgeB.next = NULL;

    p->Meta = &b2JointMetaDefault;
}

B2_API
struct b2Joint*
b2JointCreate(
    const struct b2JointDef* def, 
    struct b2BlockAllocator* allocator)
{
    typedef
    void
    (*Produce)(
        struct b2Joint* obj,
        const struct b2JointDef* def);

    const struct b2MetaAllocator* meta;
    void* mem;
    struct b2Joint* joint;
    meta = b2MetaAllocatorJoint[def->type];
    mem = b2BlockAllocatorAllocate(allocator, (int32)meta->TypeSize);
    joint = (struct b2Joint*)(mem);
    (*((Produce)meta->Produce))(joint, def);
    return joint;
}

B2_API
void
b2JointDelete(
    struct b2Joint* joint,
    struct b2BlockAllocator* allocator)
{
    typedef
    void
    (*Recycle)(
        struct b2Joint* obj);

    const struct b2MetaAllocator* meta;
    meta = b2MetaAllocatorJoint[joint->m_type];
    (*((Recycle)meta->Recycle))(joint);
    b2BlockAllocatorFree(allocator, joint, (int32)meta->TypeSize);
}

B2_API
int
b2JointIsEnabled(
    const struct b2Joint* p)
{
    return b2BodyIsEnabled(p->m_bodyA) && b2BodyIsEnabled(p->m_bodyB);
}

B2_API
void
b2JointGetAnchorA(
    const struct b2Joint* obj,
    b2Vec2 anchor)
{
    typedef
    void(*GetAnchorA)(
        const void* obj, 
        b2Vec2 anchor);

    (*((GetAnchorA)obj->Meta->GetAnchorA))(
        obj,
        anchor);
}

B2_API
void
b2JointGetAnchorB(
    const struct b2Joint* obj,
    b2Vec2 anchor)
{
    typedef
    void(*GetAnchorB)(
        const void* obj, 
        b2Vec2 anchor);

    (*((GetAnchorB)obj->Meta->GetAnchorB))(
        obj,
        anchor);
}

B2_API
void
b2JointGetReactionForce(
    const struct b2Joint* obj,
    float inv_dt,
    b2Vec2 force)
{
    typedef
    void(*GetReactionForce)(
        const void* obj, 
        float inv_dt, 
        b2Vec2 force);

    (*((GetReactionForce)obj->Meta->GetReactionForce))(
        obj,
        inv_dt,
        force);
}

B2_API
float
b2JointGetReactionTorque(
    const struct b2Joint* obj,
    float inv_dt)
{
    typedef
    float(*GetReactionTorque)(
        const void* obj, 
        float inv_dt);

    return (*((GetReactionTorque)obj->Meta->GetReactionTorque))(
        obj,
        inv_dt);
}

B2_API
void
b2JointDump(
    const struct b2Joint* obj)
{
    typedef
    void(*Dump)(
        const void* obj);

    (*((Dump)obj->Meta->Dump))(
        obj);
}

B2_API
void
b2JointShiftOrigin(
    struct b2Joint* obj,
    const b2Vec2 newOrigin)
{
    typedef
    void(*ShiftOrigin)(
        void* obj, 
        const b2Vec2 newOrigin);

    (*((ShiftOrigin)obj->Meta->ShiftOrigin))(
        obj,
        newOrigin);
}

B2_API
void
b2JointDraw(
    const struct b2Joint* obj,
    struct b2Draw* draw)
{
    typedef
    void(*Draw)(
        const void* obj, 
        struct b2Draw* draw);

    (*((Draw)obj->Meta->Draw))(
        obj,
        draw);
}

B2_API
void
b2JointInitVelocityConstraints(
    struct b2Joint* obj,
    const struct b2SolverData* data)
{
    typedef
    void(*InitVelocityConstraints)(
        void* obj, 
        const struct b2SolverData* data);

    (*((InitVelocityConstraints)obj->Meta->InitVelocityConstraints))(
        obj,
        data);
}

B2_API
void
b2JointSolveVelocityConstraints(
    struct b2Joint* obj,
    const struct b2SolverData* data)
{
    typedef
    void(*SolveVelocityConstraints)(
        void* obj, 
        const struct b2SolverData* data);

    (*((SolveVelocityConstraints)obj->Meta->SolveVelocityConstraints))(
        obj,
        data);
}

B2_API
int
b2JointSolvePositionConstraints(
    struct b2Joint* obj,
    const struct b2SolverData* data)
{
    typedef
    int(*SolvePositionConstraints)(
        void* obj, 
        const struct b2SolverData* data);

    return (*((SolvePositionConstraints)obj->Meta->SolvePositionConstraints))(
        obj,
        data);
}
