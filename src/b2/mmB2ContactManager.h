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

#ifndef __mmB2ContactManager_h__
#define __mmB2ContactManager_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2BroadPhase.h"

#include "b2/mmB2Prefix.h"

struct b2Fixture;

// Delegate of b2World.
struct b2ContactManager
{
    struct b2BroadPhase m_broadPhase;
    struct b2Contact* m_contactList;
    int32 m_contactCount;
    struct b2ContactFilter* m_contactFilter;
    struct b2ContactListener* m_contactListener;
    struct b2BlockAllocator* m_allocator;
};

B2_API
void
b2ContactManagerInit(
    struct b2ContactManager* p);

B2_API
void
b2ContactManagerDestroy(
    struct b2ContactManager* p);

// Broad-phase callback.
B2_API
void
b2ContactManagerAddPair(
    struct b2ContactManager* p,
    void* proxyUserDataA, 
    void* proxyUserDataB);

B2_API
void
b2ContactManagerFindNewContacts(
    struct b2ContactManager* p);

B2_API
void
b2ContactManagerDelete(
    struct b2ContactManager* p,
    struct b2Contact* c);

B2_API
void
b2ContactManagerCollide(
    struct b2ContactManager* p);

#include "b2/mmB2Suffix.h"

#endif//__mmB2ContactManager_h__
