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

#include "mmB2StackAllocator.h"
#include "mmB2Common.h"
#include "mmB2Math.h"

#include <assert.h>
#include <stddef.h>

B2_API
void
b2StackAllocatorInit(
    struct b2StackAllocator* p)
{
    p->m_index = 0;
    p->m_allocation = 0;
    p->m_maxAllocation = 0;
    p->m_entryCount = 0;
}

B2_API
void
b2StackAllocatorDestroy(
    struct b2StackAllocator* p)
{
    b2Assert(p->m_index == 0);
    b2Assert(p->m_entryCount == 0);
}

B2_API
void*
b2StackAllocatorAllocate(
    struct b2StackAllocator* p,
    int32 size)
{
    struct b2StackEntry* entry;

    b2Assert(p->m_entryCount < b2_maxStackEntries);

    entry = p->m_entries + p->m_entryCount;
    entry->size = size;
    if (p->m_index + size > b2_stackSize)
    {
        entry->data = (char*)b2Alloc(size);
        entry->usedMalloc = b2True;
    }
    else
    {
        entry->data = p->m_data + p->m_index;
        entry->usedMalloc = b2False;
        p->m_index += size;
    }

    p->m_allocation += size;
    p->m_maxAllocation = b2MaxInt32(p->m_maxAllocation, p->m_allocation);
    ++p->m_entryCount;

    return entry->data;
}

B2_API
void
b2StackAllocatorFree(
    struct b2StackAllocator* p,
    void* m)
{
    struct b2StackEntry* entry;

    b2Assert(p->m_entryCount > 0);
    entry = p->m_entries + p->m_entryCount - 1;
    b2Assert(m == entry->data);
    if (entry->usedMalloc)
    {
        b2Free(m);
    }
    else
    {
        p->m_index -= entry->size;
    }
    p->m_allocation -= entry->size;
    --p->m_entryCount;

    m = NULL;
}
