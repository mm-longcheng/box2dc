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

#ifndef __mmB2TimeOfImpact_h__
#define __mmB2TimeOfImpact_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Math.h"
#include "b2/mmB2Distance.h"

#include "b2/mmB2Prefix.h"

B2_API extern float b2_toiTime, b2_toiMaxTime;
B2_API extern int32 b2_toiCalls, b2_toiIters, b2_toiMaxIters;
B2_API extern int32 b2_toiRootIters, b2_toiMaxRootIters;

/// Input parameters for b2TimeOfImpact
struct b2TOIInput
{
    struct b2DistanceProxy proxyA;
    struct b2DistanceProxy proxyB;
    struct b2Sweep sweepA;
    struct b2Sweep sweepB;
    float tMax;		// defines sweep interval [0, tMax]
};

B2_API
void
b2TOIInputReset(
    struct b2TOIInput* p);

enum b2TOIOutputState
{
    b2TOIOutputUnknown,
    b2TOIOutputFailed,
    b2TOIOutputOverlapped,
    b2TOIOutputTouching,
    b2TOIOutputSeparated
};

/// Output parameters for b2TimeOfImpact.
struct b2TOIOutput
{
    enum b2TOIOutputState state;
    float t;
};

/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collisions. If you change the time interval, you should call this function
/// again.
/// Note: use b2Distance to compute the contact point and normal at the time of impact.
B2_API 
void 
b2TimeOfImpact(
    struct b2TOIOutput* output, 
    const struct b2TOIInput* input);

#include "b2/mmB2Suffix.h"

#endif//__mmB2TimeOfImpact_h__
