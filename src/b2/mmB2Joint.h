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

#ifndef __mmB2Joint_h__
#define __mmB2Joint_h__

#include <stdint.h>

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"

#include "b2/mmB2Prefix.h"

struct b2Body;
struct b2Draw;
struct b2Joint;
struct b2SolverData;
struct b2BlockAllocator;

enum b2JointType
{
    b2JointTypeUnknown,
    b2JointTypeRevolute,
    b2JointTypePrismatic,
    b2JointTypeDistance,
    b2JointTypePulley,
    b2JointTypeMouse,
    b2JointTypeGear,
    b2JointTypeWheel,
    b2JointTypeWeld,
    b2JointTypeFriction,
    b2JointTypeMotor,
};

struct b2Jacobian
{
    b2Vec2 linear;
    float angularA;
    float angularB;
};

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
struct b2JointEdge
{
    struct b2Body* other;			///< provides quick access to the other body attached.
    struct b2Joint* joint;			///< the joint
    struct b2JointEdge* prev;		///< the previous joint edge in the body's joint list
    struct b2JointEdge* next;		///< the next joint edge in the body's joint list
};

/// Joint definitions are used to construct joints.
struct b2JointDef
{
    /// The joint type is set automatically for concrete joint types.
    enum b2JointType type;

    /// Use this to attach application specific data to your joints.
    uintptr_t userData;

    /// The first attached body.
    struct b2Body* bodyA;

    /// The second attached body.
    struct b2Body* bodyB;

    /// Set this flag to true if the attached bodies should collide.
    int collideConnected;
};

/// joint super member.
#define b2JointDefSuper         \
enum b2JointType type;          \
uintptr_t userData;             \
struct b2Body* bodyA;           \
struct b2Body* bodyB;           \
int collideConnected

B2_API
void
b2JointDefReset(
    struct b2JointDef* p);

/// Utility to compute linear stiffness values from frequency and damping ratio
B2_API
void 
b2LinearStiffness(
    float* stiffness, float* damping,
    float frequencyHertz, float dampingRatio,
    const struct b2Body* bodyA, const struct b2Body* bodyB);

/// Utility to compute rotational stiffness values frequency and damping ratio
B2_API
void 
b2AngularStiffness(
    float* stiffness, float* damping,
    float frequencyHertz, float dampingRatio,
    const struct b2Body* bodyA, const struct b2Body* bodyB);

struct b2JointMeta
{
    /// Get the anchor point on bodyA in world coordinates.
    /// void(*GetAnchorA)(
    ///     const void* obj, 
    ///     b2Vec2 anchor);
    void* GetAnchorA;
    /// Get the anchor point on bodyB in world coordinates.
    /// void(*GetAnchorB)(
    ///     const void* obj, 
    ///     b2Vec2 anchor);
    void* GetAnchorB;
    /// Get the reaction force on bodyB at the joint anchor in Newtons.
    /// void(*GetReactionForce)(
    ///     const void* obj, 
    ///     float inv_dt, 
    ///     b2Vec2 force);
    void* GetReactionForce;
    /// Get the reaction torque on bodyB in N*m.
    /// float(*GetReactionTorque)(
    ///     const void* obj, 
    ///     float inv_dt);
    void* GetReactionTorque;

    /// Dump this joint to the log file.
    /// void(*Dump)(
    ///     const void* obj);
    void* Dump;
    /// Shift the origin for any points stored in world coordinates.
    /// void(*ShiftOrigin)(
    ///     void* obj, 
    ///     const b2Vec2 newOrigin);
    void* ShiftOrigin;
    /// Debug draw this joint
    /// void(*Draw)(
    ///     const void* obj, 
    ///     struct b2Draw* draw);
    void* Draw;

    /// void(*InitVelocityConstraints)(
    ///     void* obj, 
    ///     const struct b2SolverData* data);
    void* InitVelocityConstraints;
    /// void(*SolveVelocityConstraints)(
    ///     void* obj, 
    ///     const struct b2SolverData* data);
    void* SolveVelocityConstraints;
    /// This returns true if the position errors are within tolerance.
    /// int(*SolvePositionConstraints)(
    ///     void* obj, 
    ///     const struct b2SolverData* data);
    void* SolvePositionConstraints;
};

B2_API extern const struct b2JointMeta b2JointMetaDefault;

/// void
/// (*Produce)(
///     struct b2Joint* obj,
///     const struct b2JointDef* def);
///
/// void
/// (*Recycle)(
///     struct b2Joint* obj);
/// 
///  struct b2MetaAllocator

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
struct b2Joint
{
    enum b2JointType m_type;
    struct b2Joint* m_prev;
    struct b2Joint* m_next;
    struct b2JointEdge m_edgeA;
    struct b2JointEdge m_edgeB;
    struct b2Body* m_bodyA;
    struct b2Body* m_bodyB;

    int32 m_index;

    int m_islandFlag;
    int m_collideConnected;

    uintptr_t m_userData;

    const struct b2JointMeta* Meta;
};

/// joint super member.
#define b2JointSuper            \
enum b2JointType m_type;        \
struct b2Joint* m_prev;         \
struct b2Joint* m_next;         \
struct b2JointEdge m_edgeA;     \
struct b2JointEdge m_edgeB;     \
struct b2Body* m_bodyA;         \
struct b2Body* m_bodyB;         \
int32 m_index;                  \
int m_islandFlag;               \
int m_collideConnected;         \
uintptr_t m_userData;           \
const struct b2JointMeta* Meta

B2_API
void
b2JointReset(
    struct b2Joint* p);

B2_API
void
b2JointFromDef(
    struct b2Joint* p,
    const struct b2JointDef* def);

B2_API
struct b2Joint*
b2JointCreate(
    const struct b2JointDef* def, 
    struct b2BlockAllocator* allocator);

B2_API
void
b2JointDelete(
    struct b2Joint* joint, 
    struct b2BlockAllocator* allocator);

/// Get the type of the concrete joint.
static
inline
enum b2JointType 
b2JointGetType(
    const struct b2Joint* p)
{
    return p->m_type;
}

/// Get the first body attached to this joint.
static
inline
struct b2Body*
b2JointGetBodyA(
    struct b2Joint* p)
{
    return p->m_bodyA;
}

/// Get the second body attached to this joint.
static
inline
struct b2Body*
b2JointGetBodyB(
    struct b2Joint* p)
{
    return p->m_bodyB;
}

/// Short-cut function to determine if either body is enabled.
B2_API
int
b2JointIsEnabled(
    const struct b2Joint* p);

/// Get collide connected.
/// Note: modifying the collide connect flag won't work correctly because
/// the flag is only checked when fixture AABBs begin to overlap.
static
inline
int
b2JointGetCollideConnected(
    const struct b2Joint* p)
{
    return p->m_collideConnected;
}

/// Get the next joint the world joint list.
static
inline
struct b2Joint*
b2JointGetNextRef(
    struct b2Joint* p)
{
    return p->m_next;
}

static
inline
const struct b2Joint*
b2JointGetNext(
    const struct b2Joint* p)
{
    return p->m_next;
}

/// Get the user data pointer.
static
inline
uintptr_t
b2JointGetUserData(
    const struct b2Joint* p)
{
    return p->m_userData;
}

B2_API
void
b2JointGetAnchorA(
    const struct b2Joint* obj, 
    b2Vec2 anchor);

B2_API
void
b2JointGetAnchorB(
    const struct b2Joint* obj,
    b2Vec2 anchor);

B2_API
void
b2JointGetReactionForce(
    const struct b2Joint* obj, 
    float inv_dt, 
    b2Vec2 force);

B2_API
float
b2JointGetReactionTorque(
    const struct b2Joint* obj, 
    float inv_dt);

B2_API
void
b2JointDump(
    const struct b2Joint* obj);

B2_API
void
b2JointShiftOrigin(
    struct b2Joint* obj, 
    const b2Vec2 newOrigin);

B2_API
void
b2JointDraw(
    const struct b2Joint* obj, 
    struct b2Draw* draw);

B2_API
void
b2JointInitVelocityConstraints(
    struct b2Joint* obj, 
    const struct b2SolverData* data);

B2_API
void
b2JointSolveVelocityConstraints(
    struct b2Joint* obj, 
    const struct b2SolverData* data);

B2_API
int
b2JointSolvePositionConstraints(
    struct b2Joint* obj, 
    const struct b2SolverData* data);

#include "b2/mmB2Suffix.h"

#endif//__mmB2Joint_h__
