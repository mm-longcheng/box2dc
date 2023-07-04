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

#ifndef __mmB2GrowableStack_h__
#define __mmB2GrowableStack_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"

#include "b2/mmB2Prefix.h"

/// b2GrowableStack<int32, 256>

enum
{
    b2GStackN = 256,
};

typedef int32 b2GStackT;

struct b2GrowableStack
{
    b2GStackT* m_stack;
    b2GStackT m_array[b2GStackN];
    int32 m_count;
    int32 m_capacity;
};

B2_API
void
b2GrowableStackInit(
    struct b2GrowableStack* p);

B2_API
void
b2GrowableStackDestroy(
    struct b2GrowableStack* p);

B2_API
void
b2GrowableStackPush(
    struct b2GrowableStack* p,
    const b2GStackT element);

B2_API
b2GStackT
b2GrowableStackPop(
    struct b2GrowableStack* p);

static
inline
int32
b2GrowableStackGetCount(
    struct b2GrowableStack* p)
{
    return p->m_count;
}

#include "b2/mmB2Suffix.h"

#endif//__mmB2GrowableStack_h__
