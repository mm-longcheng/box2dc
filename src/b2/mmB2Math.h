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

#ifndef __mmB2Math_h__
#define __mmB2Math_h__

#include <math.h>

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"

#include "b2/mmB2Prefix.h"

typedef float* b2RotRef;
typedef float* b2Vec2Ref;
typedef float* b2Vec3Ref;
typedef float(*b2Mat22Ref)[2];
typedef float(*b2Mat33Ref)[3];
typedef float(*b2TransformRef)[2];

typedef const float* b2RotConstRef;
typedef const float* b2Vec2ConstRef;
typedef const float* b2Vec3ConstRef;
typedef const float(*b2Mat22ConstRef)[2];
typedef const float(*b2Mat33ConstRef)[3];
typedef const float(*b2TransformConstRef)[2];

#define	b2Sqrt(x)	    sqrtf(x)
#define	b2Atan2(y, x)	atan2f(y, x)

/// This function is used to ensure that a floating point number is not a NaN or infinity.
static
inline
int 
b2IsValid(
    float x)
{
    return isfinite(x);
}

static
inline
float
b2MinFloat(
    float a,
    float b)
{
    return (a < b) ? a : b;
}

static
inline
float
b2MaxFloat(
    float a,
    float b)
{
    return (a > b) ? a : b;
}

static
inline
float
b2ClampFloat(
    float v,
    float l,
    float h)
{
    return b2MaxFloat(l, b2MinFloat(v, h));
}

static
inline
int32
b2MinInt32(
    int32 a,
    int32 b)
{
    return (a < b) ? a : b;
}

static
inline
int32
b2MaxInt32(
    int32 a,
    int32 b)
{
    return (a > b) ? a : b;
}

static
inline
int32
b2ClampInt32(
    int32 v,
    int32 l,
    int32 h)
{
    return b2MaxInt32(l, b2MinInt32(v, h));
}

#define b2Swap(T, a, b)   \
{                         \
    T t;                  \
    t = a;                \
    a = b;                \
    b = t;                \
}

#define b2SwapFloat(a, b) b2Swap(float, a, b)
#define b2AbsFloat(a) fabsf(a)
#define b2AbsInt32(a) abs(a)

/// A 2D column vector.
typedef float b2Vec2[2];

B2_API extern const float b2Vec2Zero[2];

B2_API
void
b2Vec2Assign(
    float r[2],
    const float v[2]);

B2_API
int
b2Vec2Equals(
    const float a[2],
    const float b[2]);

B2_API
void
b2Vec2Make(
    float v[2],
    float x,
    float y);

B2_API
void
b2Vec2Add(
    float r[2],
    const float a[2],
    const float b[2]);

B2_API
void
b2Vec2Sub(
    float r[2],
    const float a[2],
    const float b[2]);

B2_API
float
b2Vec2DotProduct(
    const float a[2],
    const float b[2]);

B2_API
float
b2Vec2CrossProduct(
    const float a[2],
    const float b[2]);

B2_API
void
b2Vec2Negate(
    float r[2],
    const float a[2]);

B2_API
float
b2Vec2Length(
    const float a[2]);

B2_API
float
b2Vec2SquaredLength(
    const float a[2]);

B2_API
float
b2Vec2Distance(
    const float a[2],
    const float b[2]);

B2_API
float
b2Vec2SquaredDistance(
    const float a[2],
    const float b[2]);

B2_API
void
b2Vec2Scale(
    float r[2],
    const float v[2],
    const float s);

B2_API
void
b2Vec2Abs(
    float r[2],
    const float a[2]);

B2_API
void
b2Vec2Max(
    float r[2],
    const float a[2],
    const float b[2]);

B2_API
void
b2Vec2Min(
    float r[2],
    const float a[2],
    const float b[2]);

B2_API
void
b2Vec2Clamp(
    float r[2],
    const float v[2],
    const float l[2],
    const float h[2]);

/// Set this vector to all zeros.
B2_API
void
b2Vec2SetZero(
    float v[2]);

/// Does this vector contain finite coordinates?
B2_API
int
b2Vec2IsValid(
    const float v[2]);

B2_API
float
b2Vec2Normalize(
    float r[2], 
    const float v[2]);

/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
B2_API
void
b2Vec2Skew(
    float r[2],
    const float v[2]);

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
B2_API
void
b2Vec2CrossProductKR(
    float r[2],
    const float a[2],
    float s);

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
B2_API
void
b2Vec2CrossProductKL(
    float r[2],
    float s,
    const float a[2]);

/// A 2D column vector with 3 elements.
typedef float b2Vec3[3];

B2_API
void
b2Vec3Make(
    float v[3],
    float x, float y, float z);

B2_API
void
b2Vec3Add(
    float r[3],
    const float a[3],
    const float b[3]);

B2_API
void
b2Vec3Sub(
    float r[3],
    const float a[3],
    const float b[3]);

B2_API
void
b2Vec3Negate(
    float r[3],
    const float a[3]);

B2_API
void
b2Vec3Scale(
    float r[3],
    const float v[3],
    const float s);

/// Set this vector to all zeros.
B2_API
void
b2Vec3SetZero(
    float v[3]);

/// A 2-by-2 matrix. Stored in column-major order.
typedef float b2Mat22[2][2];

/// Set this mat to all zeros.
B2_API
void
b2Mat22SetZero(
    float m[2][2]);

/// r = m * v
B2_API
void
b2Mat22MulVec2(
    float r[2],
    const float m[2][2],
    const float v[2]);

/// Inverse.
B2_API
void
b2Mat22GetInverse(
    float r[2][2],
    const float m[2][2]);

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
B2_API
void
b2Mat22Solve(
    float r[2],
    const float m[2][2],
    const float v[2]);

/// A 3-by-3 matrix. Stored in column-major order.
typedef float b2Mat33[3][3];

/// r = m * v
B2_API
void
b2Mat33MulVec3(
    float r[3],
    const float m[3][3],
    const float v[3]);

/// r = m * v
B2_API
void
b2Mat33MulVec2(
    float r[2],
    const float m[3][3],
    const float v[2]);

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
B2_API
void
b2Mat33Solve33(
    float r[3],
    const float m[3][3],
    const float b[3]);

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases. Solve only the upper
/// 2-by-2 matrix equation.
B2_API
void
b2Mat33Solve22(
    float r[2],
    const float m[3][3],
    const float v[2]);

/// Get the inverse of this matrix as a 2-by-2.
/// Returns the zero matrix if singular.
B2_API
void
b2Mat33GetInverse22(
    float r[3][3],
    const float m[3][3]);

/// Get the symmetric inverse of this matrix as a 3-by-3.
/// Returns the zero matrix if singular.
B2_API
void
b2Mat33GetSymInverse33(
    float r[3][3],
    const float m[3][3]);

/// Rotation (s, c)
typedef float b2Rot[2];

/// Assign
B2_API
void
b2RotAssign(
    float r[2],
    const float q[2]);

/// Initialize from an angle in radians
B2_API
void
b2RotFromAngle(
    float q[2],
    float angle);

/// Set to the identity rotation
B2_API
void
b2RotMakeIdentity(
    float q[2]);

/// Get the angle in radians
B2_API
float
b2RotGetAngle(
    const float q[2]);

/// Get the x-axis
B2_API
void
b2RotGetXAxis(
    const float q[2],
    float v[2]);

/// Get the u-axis
B2_API
void
b2RotGetYAxis(
    const float q[2],
    float v[2]);

/// r = a * b
B2_API
void
b2RotMul(
    float r[2],
    const float a[2],
    const float b[2]);

/// r = Transpose(a) * b
B2_API
void
b2RotMulT(
    float r[2],
    const float a[2],
    const float b[2]);

/// Rotate a vector
B2_API
void
b2RotMulVec2(
    float r[2],
    const float q[2],
    const float v[2]);

/// Inverse rotate a vector
B2_API
void
b2RotMulTVec2(
    float r[2],
    const float q[2],
    const float v[2]);

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
/// b2Transform (
///     p(x, y), 
///     q(s, c))
typedef float b2Transform[2][2];

/// Assign a to t.
B2_API
void
b2TransformAssign(
    float t[2][2],
    const float a[2][2]);

/// Set this to the identity transform.
B2_API
void
b2TransformMakeIdentity(
    float t[2][2]);

/// Set this based on the position and angle.
B2_API
void
b2TransformSet(
    float t[2][2],
    const float position[2],
    float angle);

/// r = a * b
B2_API
void
b2TransformMul(
    float r[2][2],
    const float a[2][2],
    const float b[2][2]);

B2_API
void
b2TransformMulT(
    float r[2][2],
    const float a[2][2],
    const float b[2][2]);

/// r = t * v
B2_API
void
b2TransformMulVec2(
    float r[2],
    const float t[2][2],
    const float v[2]);

B2_API
void
b2TransformMulTVec2(
    float r[2],
    const float t[2][2],
    const float v[2]);

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
struct b2Sweep
{
    b2Vec2 localCenter;	///< local center of mass position
    b2Vec2 c0, c;		///< center world positions
    float a0, a;		///< world angles

    /// Fraction of the current time step in the range [0,1]
    /// c0 and a0 are the positions at alpha0.
    float alpha0;
};

/// Reset b2Sweep.
B2_API
void
b2SweepReset(
    struct b2Sweep* p);

/// Get the interpolated transform at a specific time.
/// @param transform the output transform
/// @param beta is a factor in [0,1], where 0 indicates alpha0.
B2_API
void
b2SweepGetTransform(
    const struct b2Sweep* p,
    b2Transform transform, 
    float beta);

/// Advance the sweep forward, yielding a new initial state.
/// @param alpha the new initial time.
B2_API
void
b2SweepAdvance(
    struct b2Sweep* p,
    float alpha);

/// Normalize the angles.
B2_API
void
b2SweepNormalize(
    struct b2Sweep* p);

#include "b2/mmB2Suffix.h"

#endif//__mmB2Math_h__
