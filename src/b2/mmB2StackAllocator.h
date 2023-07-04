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

#ifndef __mmB2StackAllocator_h__
#define __mmB2StackAllocator_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"

#include "b2/mmB2Prefix.h"

enum
{
    b2_stackSize       = 100 * 1024, // 100k
    b2_maxStackEntries = 32,
};

struct b2StackEntry
{
    char* data;
    int32 size;
    int usedMalloc;
};

// This is a stack allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will assert
// if you try to interleave multiple allocate/free pairs.
struct b2StackAllocator
{
    char m_data[b2_stackSize];
    int32 m_index;

    int32 m_allocation;
    int32 m_maxAllocation;

    struct b2StackEntry m_entries[b2_maxStackEntries];
    int32 m_entryCount;
};

B2_API
void
b2StackAllocatorInit(
    struct b2StackAllocator* p);

B2_API
void
b2StackAllocatorDestroy(
    struct b2StackAllocator* p);

B2_API
void*
b2StackAllocatorAllocate(
    struct b2StackAllocator* p,
    int32 size);

B2_API
void
b2StackAllocatorFree(
    struct b2StackAllocator* p,
    void* m);

static
inline
int32
b2StackAllocatorGetMaxAllocation(
    const struct b2StackAllocator* p)
{
    return p->m_maxAllocation;
}

#include "b2/mmB2Suffix.h"

#endif//__mmB2StackAllocator_h__
