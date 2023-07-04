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

#ifndef __mmB2Island_h__
#define __mmB2Island_h__

#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"

#include "b2/mmB2Prefix.h"

struct b2Profile;
struct b2TimeStep;
struct b2ContactVelocityConstraint;

/// This is an internal class.
struct b2Island
{
    struct b2StackAllocator* m_allocator;
    struct b2ContactListener* m_listener;

    struct b2Body** m_bodies;
    struct b2Contact** m_contacts;
    struct b2Joint** m_joints;

    struct b2Position* m_positions;
    struct b2Velocity* m_velocities;

    int32 m_bodyCount;
    int32 m_jointCount;
    int32 m_contactCount;

    int32 m_bodyCapacity;
    int32 m_contactCapacity;
    int32 m_jointCapacity;
};

B2_API
void
b2IslandPrepare(
    struct b2Island* p,
    int32 bodyCapacity, 
    int32 contactCapacity, 
    int32 jointCapacity,
    struct b2StackAllocator* allocator,
    struct b2ContactListener* listener);

B2_API
void
b2IslandDiscard(
    struct b2Island* p);

B2_API
void
b2IslandClear(
    struct b2Island* p);

B2_API
void
b2IslandSolve(
    struct b2Island* p,
    struct b2Profile* profile, 
    const struct b2TimeStep* step, 
    const b2Vec2 gravity, 
    int allowSleep);

B2_API
void
b2IslandSolveTOI(
    struct b2Island* p,
    const struct b2TimeStep* subStep, 
    int32 toiIndexA, 
    int32 toiIndexB);

B2_API
void
b2IslandAddBody(
    struct b2Island* p,
    struct b2Body* body);

B2_API
void
b2IslandAddContact(
    struct b2Island* p,
    struct b2Contact* contact);

B2_API
void
b2IslandAddJoint(
    struct b2Island* p,
    struct b2Joint* joint);

B2_API
void
b2IslandReport(
    struct b2Island* p,
    const struct b2ContactVelocityConstraint* constraints);

#include "b2/mmB2Suffix.h"

#endif//__mmB2Island_h__
