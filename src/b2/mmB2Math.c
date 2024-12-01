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

#include "mmB2Math.h"
#include "mmB2Common.h"

#include <assert.h>

B2_API const float b2Vec2Zero[2] = { 0.0f, 0.0f, };

B2_API
void
b2Vec2Assign(
    float r[2],
    const float v[2])
{
    r[0] = v[0];
    r[1] = v[1];
}

B2_API
int
b2Vec2Equals(
    const float a[2],
    const float b[2])
{
    return
        a[0] == b[0] &&
        a[1] == b[1];
}

B2_API
void
b2Vec2Make(
    float v[2],
    float x,
    float y)
{
    v[0] = x;
    v[1] = y;
}

B2_API
void
b2Vec2Add(
    float r[2],
    const float a[2],
    const float b[2])
{
    // r can be a or b.
    r[0] = a[0] + b[0];
    r[1] = a[1] + b[1];
}

B2_API
void
b2Vec2Sub(
    float r[2],
    const float a[2],
    const float b[2])
{
    // r can be a or b.
    r[0] = a[0] - b[0];
    r[1] = a[1] - b[1];
}

B2_API
float
b2Vec2DotProduct(
    const float a[2],
    const float b[2])
{
    return a[0] * b[0] + a[1] * b[1];
}

B2_API
float
b2Vec2CrossProduct(
    const float a[2],
    const float b[2])
{
    return a[0] * b[1] - a[1] * b[0];
}

B2_API
void
b2Vec2Negate(
    float r[2],
    const float a[2])
{
    // r can be a.
    r[0] = -a[0];
    r[1] = -a[1];
}

B2_API
float
b2Vec2Length(
    const float a[2])
{
    float x = a[0];
    float y = a[1];
    return sqrtf(x * x + y * y);
}

B2_API
float
b2Vec2SquaredLength(
    const float a[2])
{
    float x = a[0];
    float y = a[1];
    return (x * x + y * y);
}

B2_API
float
b2Vec2Distance(
    const float a[2],
    const float b[2])
{
    float x = a[0] - b[0];
    float y = a[1] - b[1];
    return sqrtf(x * x + y * y);
}

B2_API
float
b2Vec2SquaredDistance(
    const float a[2],
    const float b[2])
{
    float x = a[0] - b[0];
    float y = a[1] - b[1];
    return (x * x + y * y);
}

B2_API
void
b2Vec2Scale(
    float r[2],
    const float v[2],
    const float s)
{
    // r can be v.
    r[0] = v[0] * s;
    r[1] = v[1] * s;
}

B2_API
void
b2Vec2Abs(
    float r[2],
    const float a[2])
{
    // r can be a.
    r[0] = fabsf(a[0]);
    r[1] = fabsf(a[1]);
}

B2_API
void
b2Vec2Max(
    float r[2],
    const float a[2],
    const float b[2])
{
    // r can be a or b.
    r[0] = b2MaxFloat(a[0], b[0]);
    r[1] = b2MaxFloat(a[1], b[1]);
}

B2_API
void
b2Vec2Min(
    float r[2],
    const float a[2],
    const float b[2])
{
    // r can be a or b.
    r[0] = b2MinFloat(a[0], b[0]);
    r[1] = b2MinFloat(a[1], b[1]);
}

B2_API
void
b2Vec2Clamp(
    float r[2],
    const float v[2],
    const float l[2],
    const float h[2])
{
    // r can be v.
    r[0] = b2ClampFloat(v[0], l[0], h[0]);
    r[1] = b2ClampFloat(v[1], l[1], h[1]);
}

/// Set this vector to all zeros.
B2_API
void
b2Vec2SetZero(
    float v[2])
{
    v[0] = 0.0f;
    v[1] = 0.0f;
}

/// Does this vector contain finite coordinates?
B2_API
int
b2Vec2IsValid(
    const float v[2])
{
    return b2IsValid(v[0]) && b2IsValid(v[1]);
}

B2_API
float
b2Vec2Normalize(
    float r[2],
    const float v[2])
{
    float l;
    float x = v[0];
    float y = v[1];
    float d = (x * x + y * y);
    if (d < b2_epsilon2)
    {
        l = 0.0f;
        r[0] = 0.0f;
        r[1] = 0.0f;
    }
    else if (1.0f == d)
    {
        l = 1.0f;
        r[0] = x;
        r[1] = y;
    }
    else
    {
        float k;
        l = sqrtf(d);
        k = 1.0f / l;
        r[0] = x * k;
        r[1] = y * k;
    }
    return l;
}

/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
B2_API
void
b2Vec2Skew(
    float r[2],
    const float v[2])
{
    r[0] = -v[1];
    r[1] = +v[0];
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
B2_API
void
b2Vec2CrossProductKR(
    float r[2],
    const float a[2], float s)
{
    r[0] = +s * a[1];
    r[1] = -s * a[0];
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
B2_API
void
b2Vec2CrossProductKL(
    float r[2],
    float s,
    const float a[2])
{
    r[0] = -s * a[1];
    r[1] = +s * a[0];
}

B2_API
void
b2Vec3Make(
    float v[3],
    float x, float y, float z)
{
    v[0] = x;
    v[1] = y;
    v[2] = z;
}

B2_API
void
b2Vec3Add(
    float r[3],
    const float a[3],
    const float b[3])
{
    // r can be a or b.
    r[0] = a[0] + b[0];
    r[1] = a[1] + b[1];
    r[2] = a[2] + b[2];
}

B2_API
void
b2Vec3Sub(
    float r[3],
    const float a[3],
    const float b[3])
{
    // r can be a or b.
    r[0] = a[0] - b[0];
    r[1] = a[1] - b[1];
    r[2] = a[2] - b[2];
}

B2_API
void
b2Vec3Negate(
    float r[3],
    const float a[3])
{
    // r can be a.
    r[0] = -a[0];
    r[1] = -a[1];
    r[2] = -a[2];
}

B2_API
void
b2Vec3Scale(
    float r[3],
    const float v[3],
    const float s)
{
    // r can be v.
    r[0] = v[0] * s;
    r[1] = v[1] * s;
    r[2] = v[2] * s;
}

/// Set this vector to all zeros.
B2_API
void
b2Vec3SetZero(
    float v[3])
{
    v[0] = 0.0f;
    v[1] = 0.0f;
    v[2] = 0.0f;
}

/// Set this mat to all zeros.
B2_API
void
b2Mat22SetZero(
    float m[2][2])
{
    m[0][0] = 0.0f; m[1][0] = 0.0f;
    m[0][1] = 0.0f; m[1][1] = 0.0f;
}

/// r = m * v
B2_API
void
b2Mat22MulVec2(
    float r[2],
    const float m[2][2],
    const float v[2])
{
    float v0 = v[0], v1 = v[1];

    r[0] = m[0][0] * v0 + m[1][0] * v1;
    r[1] = m[0][1] * v0 + m[1][1] * v1;
}

/// Inverse.
B2_API
void
b2Mat22GetInverse(
    float r[2][2],
    const float m[2][2])
{
    float idet;

    float m00 = m[0][0], m01 = m[0][1];
    float m10 = m[1][0], m11 = m[1][1];

    idet = m00 * m11 - m01 * m10;
    idet = (0.0f != idet) ? (1.0f / idet) : 0.0f;

    r[0][0] = +m11 * idet;
    r[0][1] = -m01 * idet;
    r[1][0] = -m10 * idet;
    r[1][1] = +m00 * idet;
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
B2_API
void
b2Mat22Solve(
    float r[2],
    const float m[2][2],
    const float v[2])
{
    // r = Inverse(m) * v
    float idet;

    float v0 = v[0], v1 = v[1];

    float m00 = m[0][0], m01 = m[0][1];
    float m10 = m[1][0], m11 = m[1][1];

    idet = m00 * m11 - m10 * m01;
    idet = (0.0f != idet) ? (1.0f / idet) : 0.0f;

    r[0] = idet * (m11 * v0 - m10 * v1);
    r[1] = idet * (m00 * v1 - m01 * v0);
}

/// r = m * v
B2_API
void
b2Mat33MulVec3(
    float r[3],
    const float m[3][3],
    const float v[3])
{
    float v0 = v[0], v1 = v[1], v2 = v[2];

    r[0] = m[0][0] * v0 + m[1][0] * v1 + m[2][0] * v2;
    r[1] = m[0][1] * v0 + m[1][1] * v1 + m[2][1] * v2;
    r[2] = m[0][2] * v0 + m[1][2] * v1 + m[2][2] * v2;
}

/// r = m * v
B2_API
void
b2Mat33MulVec2(
    float r[2],
    const float m[3][3],
    const float v[2])
{
    float v0 = v[0], v1 = v[1];

    r[0] = m[0][0] * v0 + m[1][0] * v1;
    r[1] = m[0][1] * v0 + m[1][1] * v1;
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases.
B2_API
void
b2Mat33Solve33(
    float r[3],
    const float m[3][3],
    const float v[3])
{
    // r = Inverse(m) * v
    float idet;
    float c0, c1, c2;

    float i[3][3];

    float v0 = v[0], v1 = v[1], v2 = v[2];

    float m00 = m[0][0], m01 = m[0][1], m02 = m[0][2];
    float m10 = m[1][0], m11 = m[1][1], m12 = m[1][2];
    float m20 = m[2][0], m21 = m[2][1], m22 = m[2][2];

    c0 = (+m11 * m22 - m12 * m21);
    c1 = (-m10 * m22 + m20 * m12);
    c2 = (+m10 * m21 - m20 * m11);

    idet = m00 * c0 + m01 * c1 + m02 * c2;
    idet = (0.0f != idet) ? (1.0f / idet) : 0.0f;

    i[0][0] = /*                  */c0 * idet;
    i[0][1] = (-m01 * m22 + m21 * m02) * idet;
    i[0][2] = (+m01 * m12 - m11 * m02) * idet;
    i[1][0] = /*                  */c1 * idet;
    i[1][1] = (+m00 * m22 - m02 * m20) * idet;
    i[1][2] = (-m00 * m12 + m10 * m02) * idet;
    i[2][0] = /*                  */c2 * idet;
    i[2][1] = (-m00 * m21 + m20 * m01) * idet;
    i[2][2] = (+m00 * m11 - m01 * m10) * idet;

    r[0] = i[0][0] * v0 + i[1][0] * v1 + i[2][0] * v2;
    r[1] = i[0][1] * v0 + i[1][1] * v1 + i[2][1] * v2;
    r[2] = i[0][2] * v0 + i[1][2] * v1 + i[2][2] * v2;
}

/// Solve A * x = b, where b is a column vector. This is more efficient
/// than computing the inverse in one-shot cases. Solve only the upper
/// 2-by-2 matrix equation.
B2_API
void
b2Mat33Solve22(
    float r[2],
    const float m[3][3],
    const float v[2])
{
    // r = Inverse(m) * v
    float idet;

    float v0 = v[0], v1 = v[1];

    float m00 = m[0][0], m01 = m[0][1];
    float m10 = m[1][0], m11 = m[1][1];

    idet = m00 * m11 - m10 * m01;
    idet = (0.0f != idet) ? (1.0f / idet) : 0.0f;

    r[0] = idet * (m11 * v0 - m10 * v1);
    r[1] = idet * (m00 * v1 - m01 * v0);
}

/// Get the inverse of this matrix as a 2-by-2.
/// Returns the zero matrix if singular.
B2_API
void
b2Mat33GetInverse22(
    float r[3][3],
    const float m[3][3])
{
    float idet;

    float m00 = m[0][0], m10 = m[1][0];
    float m01 = m[0][1], m11 = m[1][1];

    idet = m00 * m11 - m10 * m01;
    idet = (0.0f != idet) ? (1.0f / idet) : 0.0f;

    r[0][0] = +idet * m11; r[1][0] = -idet * m10; r[0][2] = 0.0f;
    r[0][1] = -idet * m01; r[1][1] = +idet * m00; r[1][2] = 0.0f;
    r[2][0] =        0.0f; r[2][1] =        0.0f; r[2][2] = 0.0f;
}

/// Get the symmetric inverse of this matrix as a 3-by-3.
/// Returns the zero matrix if singular.
B2_API
void
b2Mat33GetSymInverse33(
    float r[3][3],
    const float m[3][3])
{
    float idet;
    float c0, c1, c2;

    float m00 = m[0][0], m01 = m[0][1], m02 = m[0][2];
    float m10 = m[1][0], m11 = m[1][1], m12 = m[1][2];
    float m20 = m[2][0], m21 = m[2][1], m22 = m[2][2];

    c0 = (+m11 * m22 - m12 * m21);
    c1 = (-m10 * m22 + m20 * m12);
    c2 = (+m10 * m21 - m20 * m11);

    idet = m00 * c0 + m01 * c1 + m02 * c2;
    idet = (0.0f != idet) ? (1.0f / idet) : 0.0f;

    r[0][0] = idet * (m11 * m22 - m12 * m12);
    r[0][1] = idet * (m02 * m12 - m01 * m22);
    r[0][2] = idet * (m01 * m12 - m02 * m11);

    r[1][0] = r[0][1];
    r[1][1] = idet * (m00 * m22 - m02 * m02);
    r[1][2] = idet * (m02 * m01 - m00 * m12);

    r[2][0] = r[0][2];
    r[2][1] = r[1][2];
    r[2][2] = idet * (m00 * m11 - m01 * m01);
}

/// Assign
B2_API
void
b2RotAssign(
    float r[2],
    const float q[2])
{
    r[0] = q[0];
    r[1] = q[1];
}

/// Initialize from an angle in radians
B2_API
void
b2RotFromAngle(
    float q[2],
    float angle)
{
    q[0] = sinf(angle);
    q[1] = cosf(angle);
}

/// Set to the identity rotation
B2_API
void
b2RotMakeIdentity(
    float q[2])
{
    q[0] = 0.0f;
    q[1] = 1.0f;
}

/// Get the angle in radians
B2_API
float
b2RotGetAngle(
    const float q[2])
{
    return b2Atan2(q[0], q[1]);
}

/// Get the x-axis
B2_API
void
b2RotGetXAxis(
    const float q[2],
    float v[2])
{
    v[0] = +q[1];
    v[1] = +q[0];
}

/// Get the u-axis
B2_API
void
b2RotGetYAxis(
    const float q[2],
    float v[2])
{
    v[0] = -q[0];
    v[1] = +q[1];
}

/// r = a * b
B2_API
void
b2RotMul(
    float r[2],
    const float a[2],
    const float b[2])
{
    // [ac -as] * [bc -bs] = [ac*bc-as*bs -ac*bs-as*bc]
    // [as  ac]   [bs  bc]   [as*bc+ac*bs -as*bs+ac*bc]
    // s = as * bc + ac * bs
    // c = ac * bc - as * bs
    float a0 = a[0], a1 = a[1];
    float b0 = b[0], b1 = b[1];
    r[0] = a0 * b1 + a1 * b0;
    r[1] = a1 * b1 - a0 * b0;
}

/// r = Transpose(a) * b
B2_API
void
b2RotMulT(
    float r[2],
    const float a[2],
    const float b[2])
{
    // [ ac as] * [bc -bs] = [ac*bc+as*bs -ac*bs+as*bc]
    // [-as ac]   [bs  bc]   [-as*bc+ac*bs as*bs+ac*bc]
    // s = ac * bs - as * bc
    // c = ac * bc + as * bs
    float a0 = a[0], a1 = a[1];
    float b0 = b[0], b1 = b[1];
    r[0] = a1 * b0 - a0 * b1;
    r[1] = a1 * b1 + a0 * b0;
}

/// Rotate a vector
B2_API
void
b2RotMulVec2(
    float r[2],
    const float q[2],
    const float v[2])
{
    float q0 = q[0], q1 = q[1];
    float v0 = v[0], v1 = v[1];

    r[0] = q1 * v0 - q0 * v1;
    r[1] = q0 * v0 + q1 * v1;
}

/// Inverse rotate a vector
B2_API
void
b2RotMulTVec2(
    float r[2],
    const float q[2],
    const float v[2])
{
    float q0 = q[0], q1 = q[1];
    float v0 = v[0], v1 = v[1];

    r[0] = +q1 * v0 + q0 * v1;
    r[1] = -q0 * v0 + q1 * v1;
}

/// Assign a to t.
B2_API
void
b2TransformAssign(
    float t[2][2],
    const float a[2][2])
{
    t[0][0] = a[0][0]; t[1][0] = a[1][0];
    t[0][1] = a[0][1]; t[1][1] = a[1][1];
}

/// Set this to the identity transform.
B2_API
void
b2TransformMakeIdentity(
    float t[2][2])
{
    b2Vec2SetZero(t[0]);
    b2RotMakeIdentity(t[1]);
}

/// Set this based on the position and angle.
B2_API
void
b2TransformSet(
    float t[2][2],
    const float position[2],
    float angle)
{
    b2Vec2Assign(t[0], position);
    b2RotFromAngle(t[1], angle);
}

/// r = a * b
B2_API
void
b2TransformMul(
    float r[2][2],
    const float a[2][2],
    const float b[2][2])
{
    b2RotMul(r[1], a[1], b[1]);
    b2RotMulVec2(r[0], a[1], b[0]);
    b2Vec2Add(r[0], r[0], a[0]);
}

B2_API
void
b2TransformMulT(
    float r[2][2],
    const float a[2][2],
    const float b[2][2])
{
    b2RotMulT(r[1], a[1], b[1]);
    b2Vec2Sub(r[0], b[0], a[0]);
    b2RotMulTVec2(r[0], a[1], r[0]);
}

/// r = t * v
B2_API
void
b2TransformMulVec2(
    float r[2],
    const float t[2][2],
    const float v[2])
{
    float v0 = v[0], v1 = v[1];
    r[0] = (t[1][1] * v0 - t[1][0] * v1) + t[0][0];
    r[1] = (t[1][0] * v0 + t[1][1] * v1) + t[0][1];
}

B2_API
void
b2TransformMulTVec2(
    float r[2],
    const float t[2][2],
    const float v[2])
{
    float px = v[0] - t[0][0];
    float py = v[1] - t[0][1];
    r[0] = (+t[1][1] * px + t[1][0] * py);
    r[1] = (-t[1][0] * px + t[1][1] * py);
}

/// Reset b2Sweep.
B2_API
void
b2SweepReset(
    struct b2Sweep* p)
{
    b2Vec2SetZero(p->localCenter);
    b2Vec2SetZero(p->c0);
    b2Vec2SetZero(p->c);
    p->a0 = 0.0f;
    p->a = 0.0f;
    p->alpha0 = 0.0f;
}

/// Get the interpolated transform at a specific time.
/// @param transform the output transform
/// @param beta is a factor in [0,1], where 0 indicates alpha0.
/// https://fgiesen.wordpress.com/2012/08/15/linear-interpolation-past-present-and-future/
B2_API
void
b2SweepGetTransform(
    const struct b2Sweep* p,
    b2Transform transform,
    float beta)
{
    float angle;
    float v0[2], v1[2];
    b2Vec2Scale(v0, p->c0, (1.0f - beta));
    b2Vec2Scale(v1, p->c, beta);
    b2Vec2Add(transform[0], v0, v1);

    angle = (1.0f - beta) * p->a0 + beta * p->a;
    b2RotFromAngle(transform[1], angle);

    // Shift to origin
    b2RotMulVec2(v0, transform[1], p->localCenter);
    b2Vec2Sub(transform[0], transform[0], v0);
}

/// Advance the sweep forward, yielding a new initial state.
/// @param alpha the new initial time.
B2_API
void
b2SweepAdvance(
    struct b2Sweep* p,
    float alpha)
{
    float beta;
    b2Vec2 v;
    b2Assert(p->alpha0 < 1.0f);
    beta = (alpha - p->alpha0) / (1.0f - p->alpha0);

    b2Vec2Sub(v, p->c, p->c0);
    b2Vec2Scale(v, v, beta);
    b2Vec2Add(p->c0, p->c0, v);

    p->a0 += beta * (p->a - p->a0);

    p->alpha0 = alpha;
}

/// Normalize the angles.
B2_API
void
b2SweepNormalize(
    struct b2Sweep* p)
{
    float d = b2_2pi * floorf(p->a0 / b2_2pi);
    p->a0 -= d;
    p->a -= d;
}
