/*
-----------------------------------------------------------------------------
MIT License

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

#include "mmB2JointTypes.h"

#include "mmB2JointRevolute.h"
#include "mmB2JointPrismatic.h"
#include "mmB2JointDistance.h"
#include "mmB2JointPulley.h"
#include "mmB2JointMouse.h"
#include "mmB2JointGear.h"
#include "mmB2JointWheel.h"
#include "mmB2JointWeld.h"
#include "mmB2JointFriction.h"
#include "mmB2JointMotor.h"

B2_API const struct b2MetaAllocator* b2MetaAllocatorJoint[11] =
{
    NULL,
    &b2MetaAllocatorJointRevolute,
    &b2MetaAllocatorJointPrismatic,
    &b2MetaAllocatorJointDistance,
    &b2MetaAllocatorJointPulley,
    &b2MetaAllocatorJointMouse,
    &b2MetaAllocatorJointGear,
    &b2MetaAllocatorJointWheel,
    &b2MetaAllocatorJointWeld,
    &b2MetaAllocatorJointFriction,
    &b2MetaAllocatorJointMotor,
};

