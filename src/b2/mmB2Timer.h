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

#ifndef __mmB2Timer_h__
#define __mmB2Timer_h__

#include "b2/mmB2Api.h"

#include "b2/mmB2Prefix.h"

#if defined(_WIN32)
B2_API extern double s_invFrequency;
#endif

/// Timer for profiling. This has platform specific code and may
/// not work on every platform.
struct b2Timer
{
#if defined(_WIN32)
    double m_start;
#elif defined(__linux__) || defined (__APPLE__)
    unsigned long long m_start_sec;
    unsigned long long m_start_usec;
#endif
};

B2_API
void
b2TimerMake(
    struct b2Timer* p);

B2_API
void
b2TimerReset(
    struct b2Timer* p);

B2_API
float
b2TimerGetMilliseconds(
    const struct b2Timer* p);

#include "b2/mmB2Suffix.h"

#endif//__mmB2Timer_h__
