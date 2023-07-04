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

#include "mmB2GrowableStack.h"
#include "mmB2Settings.h"
#include "mmB2Common.h"

#include <assert.h>
#include <string.h>

B2_API
void
b2GrowableStackInit(
    struct b2GrowableStack* p)
{
    p->m_stack = p->m_array;
    p->m_count = 0;
    p->m_capacity = b2GStackN;
}

B2_API
void
b2GrowableStackDestroy(
    struct b2GrowableStack* p)
{
    if (p->m_stack != p->m_array)
    {
        b2Free(p->m_stack);
        p->m_stack = NULL;
    }
}

B2_API
void
b2GrowableStackPush(
    struct b2GrowableStack* p, 
    const b2GStackT element)
{
    if (p->m_count == p->m_capacity)
    {
        b2GStackT* old = p->m_stack;
        p->m_capacity *= 2;
        p->m_stack = (b2GStackT*)b2Alloc(p->m_capacity * sizeof(b2GStackT));
        memcpy(p->m_stack, old, p->m_count * sizeof(b2GStackT));
        if (old != p->m_array)
        {
            b2Free(old);
        }
    }

    p->m_stack[p->m_count] = element;
    ++p->m_count;
}

B2_API
b2GStackT
b2GrowableStackPop(
    struct b2GrowableStack* p)
{
    b2Assert(p->m_count > 0);
    --p->m_count;
    return p->m_stack[p->m_count];
}
