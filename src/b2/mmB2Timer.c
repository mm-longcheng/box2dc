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

#include "mmB2Timer.h"

#if defined(_WIN32)

B2_API double s_invFrequency = 0.0;

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>

B2_API
void
b2TimerMake(
    struct b2Timer* p)
{
    LARGE_INTEGER largeInteger;

    if (s_invFrequency == 0.0)
    {
        QueryPerformanceFrequency(&largeInteger);
        s_invFrequency = (double)(largeInteger.QuadPart);
        if (s_invFrequency > 0.0)
        {
            s_invFrequency = 1000.0 / s_invFrequency;
        }
    }

    QueryPerformanceCounter(&largeInteger);
    p->m_start = (double)(largeInteger.QuadPart);
}

B2_API
void
b2TimerReset(
    struct b2Timer* p)
{
    LARGE_INTEGER largeInteger;
    QueryPerformanceCounter(&largeInteger);
    p->m_start = (double)(largeInteger.QuadPart);
}

B2_API
float
b2TimerGetMilliseconds(
    const struct b2Timer* p)
{
    LARGE_INTEGER largeInteger;
    double count;
    float ms;

    QueryPerformanceCounter(&largeInteger);
    count = (double)(largeInteger.QuadPart);
    ms = (float)(s_invFrequency * (count - p->m_start));
    return ms;
}

#elif defined(__linux__) || defined (__APPLE__)

#include <sys/time.h>

B2_API
void
b2TimerMake(
    struct b2Timer* p)
{
    b2TimerReset(p);
}

B2_API
void
b2TimerReset(
    struct b2Timer* p)
{
    struct timeval t;
    gettimeofday(&t, 0);
    p->m_start_sec = t.tv_sec;
    p->m_start_usec = t.tv_usec;
}

B2_API
float
b2TimerGetMilliseconds(
    const struct b2Timer* p)
{
    struct timeval t;
    gettimeofday(&t, 0);
    time_t start_sec = p->m_start_sec;
    suseconds_t start_usec = (suseconds_t)p->m_start_usec;

    // http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
    if (t.tv_usec < start_usec)
    {
        int nsec = (start_usec - t.tv_usec) / 1000000 + 1;
        start_usec -= 1000000 * nsec;
        start_sec += nsec;
    }

    if (t.tv_usec - start_usec > 1000000)
    {
        int nsec = (t.tv_usec - start_usec) / 1000000;
        start_usec += 1000000 * nsec;
        start_sec -= nsec;
    }
    return 1000.0f * (t.tv_sec - start_sec) + 0.001f * (t.tv_usec - start_usec);
}

#else

B2_API
void
b2TimerMake(
    struct b2Timer* p)
{

}

B2_API
void
b2TimerReset(
    struct b2Timer* p)
{

}

B2_API
float
b2TimerGetMilliseconds(
    const struct b2Timer* p)
{
    return 0.0f;
}

#endif

