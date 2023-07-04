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

#include "mmB2Rope.h"
#include "mmB2Common.h"
#include "mmB2Settings.h"
#include "mmB2Draw.h"

#include <assert.h>
#include <stddef.h>

struct b2RopeStretch
{
    int32 i1, i2;
    float invMass1, invMass2;
    float L;
    float lambda;
    float spring;
    float damper;
};

struct b2RopeBend
{
    int32 i1, i2, i3;
    float invMass1, invMass2, invMass3;
    float invEffectiveMass;
    float lambda;
    float L1, L2;
    float alpha1, alpha2;
    float spring;
    float damper;
};

B2_API
void
b2RopeTuningReset(
    struct b2RopeTuning* p)
{
    p->stretchingModel = b2StretchingModelPBD;
    p->bendingModel = b2BendingModelPBDAngle;
    p->damping = 0.0f;
    p->stretchStiffness = 1.0f;
    p->stretchHertz = 1.0f;
    p->stretchDamping = 0.0f;
    p->bendStiffness = 0.5f;
    p->bendHertz = 1.0f;
    p->bendDamping = 0.0f;
    p->isometric = b2False;
    p->fixedEffectiveMass = b2False;
    p->warmStart = b2False;
}

B2_API
void
b2RopeDefReset(
    struct b2RopeDef* p)
{
    b2Vec2SetZero(p->position);
    p->vertices = NULL;
    p->count = 0;
    p->masses = NULL;
    b2Vec2SetZero(p->gravity);

    b2RopeTuningReset(&p->tuning);
}

B2_API
void
b2RopeInit(
    struct b2Rope* p)
{
    b2Vec2SetZero(p->m_position);
    p->m_count = 0;
    p->m_stretchCount = 0;
    p->m_bendCount = 0;
    p->m_stretchConstraints = NULL;
    p->m_bendConstraints = NULL;
    p->m_bindPositions = NULL;
    p->m_ps = NULL;
    p->m_p0s = NULL;
    p->m_vs = NULL;
    p->m_invMasses = NULL;
    b2Vec2SetZero(p->m_gravity);

    b2RopeTuningReset(&p->m_tuning);
}

B2_API
void
b2RopeDestroy(
    struct b2Rope* p)
{
    b2Free(p->m_stretchConstraints);
    b2Free(p->m_bendConstraints);
    b2Free(p->m_bindPositions);
    b2Free(p->m_ps);
    b2Free(p->m_p0s);
    b2Free(p->m_vs);
    b2Free(p->m_invMasses);
}

B2_API
void
b2RopeCreate(
    struct b2Rope* p,
    const struct b2RopeDef* def)
{
    int32 i;

    b2Assert(def->count >= 3);
    b2Vec2Assign(p->m_position, def->position);
    p->m_count = def->count;
    p->m_bindPositions = (b2Vec2*)b2Alloc(p->m_count * sizeof(b2Vec2));
    p->m_ps = (b2Vec2*)b2Alloc(p->m_count * sizeof(b2Vec2));
    p->m_p0s = (b2Vec2*)b2Alloc(p->m_count * sizeof(b2Vec2));
    p->m_vs = (b2Vec2*)b2Alloc(p->m_count * sizeof(b2Vec2));
    p->m_invMasses = (float*)b2Alloc(p->m_count * sizeof(float));

    for (i = 0; i < p->m_count; ++i)
    {
        float m;

        b2Vec2Assign(p->m_bindPositions[i], def->vertices[i]);
        b2Vec2Add(p->m_ps[i], def->vertices[i], p->m_position);
        b2Vec2Add(p->m_p0s[i], def->vertices[i], p->m_position);
        b2Vec2SetZero(p->m_vs[i]);

        m = def->masses[i];
        if (m > 0.0f)
        {
            p->m_invMasses[i] = 1.0f / m;
        }
        else
        {
            p->m_invMasses[i] = 0.0f;
        }
    }

    p->m_stretchCount = p->m_count - 1;
    p->m_bendCount = p->m_count - 2;

    p->m_stretchConstraints = (struct b2RopeStretch*)b2Alloc(p->m_stretchCount * sizeof(struct b2RopeStretch));
    p->m_bendConstraints = (struct b2RopeBend*)b2Alloc(p->m_bendCount * sizeof(struct b2RopeBend));

    for (i = 0; i < p->m_stretchCount; ++i)
    {
        b2Vec2 p1;
        b2Vec2 p2;

        struct b2RopeStretch* c = &p->m_stretchConstraints[i];

        b2Vec2Assign(p1, p->m_ps[i/**/]);
        b2Vec2Assign(p2, p->m_ps[i + 1]);

        c->i1 = i/**/;
        c->i2 = i + 1;
        c->L = b2Vec2Distance(p1, p2);
        c->invMass1 = p->m_invMasses[i/**/];
        c->invMass2 = p->m_invMasses[i + 1];
        c->lambda = 0.0f;
        c->damper = 0.0f;
        c->spring = 0.0f;
    }

    for (i = 0; i < p->m_bendCount; ++i)
    {
        b2Vec2 v;

        b2Vec2 p1;
        b2Vec2 p2;
        b2Vec2 p3;

        b2Vec2 e1;
        b2Vec2 e2;

        float L1sqr;
        float L2sqr;

        b2Vec2 Jd1;
        b2Vec2 Jd2;

        b2Vec2 J1;
        b2Vec2 J2;
        b2Vec2 J3;

        b2Vec2 r;
        float rr;

        struct b2RopeBend* c = &p->m_bendConstraints[i];

        b2Vec2Assign(p1, p->m_ps[i/**/]);
        b2Vec2Assign(p2, p->m_ps[i + 1]);
        b2Vec2Assign(p3, p->m_ps[i + 2]);

        c->i1 = i/**/;
        c->i2 = i + 1;
        c->i3 = i + 2;
        c->invMass1 = p->m_invMasses[i/**/];
        c->invMass2 = p->m_invMasses[i + 1];
        c->invMass3 = p->m_invMasses[i + 2];
        c->invEffectiveMass = 0.0f;
        c->L1 = b2Vec2Distance(p1, p2);
        c->L2 = b2Vec2Distance(p2, p3);
        c->lambda = 0.0f;

        // Pre-compute effective mass (TODO use flattened config)
        b2Vec2Sub(e1, p2, p1);
        b2Vec2Sub(e2, p3, p2);
        L1sqr = b2Vec2SquaredLength(e1);
        L2sqr = b2Vec2SquaredLength(e2);

        if (L1sqr * L2sqr == 0.0f)
        {
            continue;
        }

        b2Vec2Skew(v, e1);
        b2Vec2Scale(Jd1, v, -1.0f / L1sqr);

        b2Vec2Skew(v, e2);
        b2Vec2Scale(Jd2, v, +1.0f / L2sqr);

        b2Vec2Negate(J1, Jd1);
        b2Vec2Sub(J2, Jd1, Jd2);
        b2Vec2Assign(J3, Jd2);

        c->invEffectiveMass = 
            c->invMass1 * b2Vec2DotProduct(J1, J1) + 
            c->invMass2 * b2Vec2DotProduct(J2, J2) + 
            c->invMass3 * b2Vec2DotProduct(J3, J3);

        b2Vec2Sub(r, p3, p1);
        rr = b2Vec2SquaredLength(r);
        if (rr == 0.0f)
        {
            continue;
        }

        // a1 = h2 / (h1 + h2)
        // a2 = h1 / (h1 + h2)
        c->alpha1 = b2Vec2DotProduct(e2, r) / rr;
        c->alpha2 = b2Vec2DotProduct(e1, r) / rr;
    }

    b2Vec2Assign(p->m_gravity, def->gravity);

    b2RopeSetTuning(p, &def->tuning);
}

B2_API
void
b2RopeSetTuning(
    struct b2Rope* p,
    const struct b2RopeTuning* tuning)
{
    int32 i;

    float bendOmega;
    float stretchOmega;

    p->m_tuning = *tuning;

    // Pre-compute spring and damper values based on tuning

    bendOmega = 2.0f * b2_pi * p->m_tuning.bendHertz;

    for (i = 0; i < p->m_bendCount; ++i)
    {
        float L1sqr;
        float L2sqr;

        float J2;
        float sum;

        float mass;

        struct b2RopeBend* c = &p->m_bendConstraints[i];

        L1sqr = c->L1 * c->L1;
        L2sqr = c->L2 * c->L2;

        if (L1sqr * L2sqr == 0.0f)
        {
            c->spring = 0.0f;
            c->damper = 0.0f;
            continue;
        }

        // Flatten the triangle formed by the two edges
        J2 = 1.0f / c->L1 + 1.0f / c->L2;
        sum = c->invMass1 / L1sqr + c->invMass2 * J2 * J2 + c->invMass3 / L2sqr;
        if (sum == 0.0f)
        {
            c->spring = 0.0f;
            c->damper = 0.0f;
            continue;
        }

        mass = 1.0f / sum;

        c->spring = mass * bendOmega * bendOmega;
        c->damper = 2.0f * mass * p->m_tuning.bendDamping * bendOmega;
    }

    stretchOmega = 2.0f * b2_pi * p->m_tuning.stretchHertz;

    for (i = 0; i < p->m_stretchCount; ++i)
    {
        float sum;
        float mass;

        struct b2RopeStretch* c = &p->m_stretchConstraints[i];

        sum = c->invMass1 + c->invMass2;
        if (sum == 0.0f)
        {
            continue;
        }

        mass = 1.0f / sum;

        c->spring = mass * stretchOmega * stretchOmega;
        c->damper = 2.0f * mass * p->m_tuning.stretchDamping * stretchOmega;
    }
}

static
void
b2RopeSolveStretch_PBD(
    struct b2Rope* p)
{
    int32 i;
    const float stiffness = p->m_tuning.stretchStiffness;

    for (i = 0; i < p->m_stretchCount; ++i)
    {
        b2Vec2 v;

        b2Vec2 p1;
        b2Vec2 p2;

        b2Vec2 d;
        float L;
        float sum;

        float s1;
        float s2;

        const struct b2RopeStretch* c = &p->m_stretchConstraints[i];

        b2Vec2Assign(p1, p->m_ps[c->i1]);
        b2Vec2Assign(p2, p->m_ps[c->i2]);

        b2Vec2Sub(d, p2, p1);
        L = b2Vec2Normalize(d, d);

        sum = c->invMass1 + c->invMass2;
        if (sum == 0.0f)
        {
            continue;
        }

        s1 = c->invMass1 / sum;
        s2 = c->invMass2 / sum;

        b2Vec2Scale(v, d, stiffness * s1 * (c->L - L));
        b2Vec2Sub(p1, p1, v);

        b2Vec2Scale(v, d, stiffness * s2 * (c->L - L));
        b2Vec2Add(p2, p2, v);

        b2Vec2Assign(p->m_ps[c->i1], p1);
        b2Vec2Assign(p->m_ps[c->i2], p2);
    }
}

static
void
b2RopeSolveStretch_XPBD(
    struct b2Rope* p,
    float dt)
{
    int32 i;

    b2Assert(dt > 0.0f);

    for (i = 0; i < p->m_stretchCount; ++i)
    {
        b2Vec2 v;

        b2Vec2 p1;
        b2Vec2 p2;

        b2Vec2 dp1;
        b2Vec2 dp2;

        b2Vec2 u;
        float L;

        b2Vec2 J1;
        b2Vec2 J2;

        float sum;

        float alpha;
        float beta;
        float sigma;
        float C;

        float Cdot;

        float B;
        float sum2;

        float impulse;

        struct b2RopeStretch* c = &p->m_stretchConstraints[i];

        b2Vec2Assign(p1, p->m_ps[c->i1]);
        b2Vec2Assign(p2, p->m_ps[c->i2]);

        b2Vec2Sub(dp1, p1, p->m_p0s[c->i1]);
        b2Vec2Sub(dp2, p2, p->m_p0s[c->i2]);

        b2Vec2Sub(u, p2, p1);
        L = b2Vec2Normalize(u, u);

        b2Vec2Negate(J1, u);
        b2Vec2Assign(J2, u);

        sum = c->invMass1 + c->invMass2;
        if (sum == 0.0f)
        {
            continue;
        }

        alpha = 1.0f / (c->spring * dt * dt);	// 1 / kg
        beta = dt * dt * c->damper;				// kg * s
        sigma = alpha * beta / dt;				// non-dimensional
        C = L - c->L;

        // This is using the initial velocities
        Cdot = b2Vec2DotProduct(J1, dp1) + b2Vec2DotProduct(J2, dp2);

        B = C + alpha * c->lambda + sigma * Cdot;
        sum2 = (1.0f + sigma) * sum + alpha;

        impulse = -B / sum2;

        b2Vec2Scale(v, J1, c->invMass1 * impulse);
        b2Vec2Add(p1, p1, v);

        b2Vec2Scale(v, J2, c->invMass2 * impulse);
        b2Vec2Add(p2, p2, v);

        b2Vec2Assign(p->m_ps[c->i1], p1);
        b2Vec2Assign(p->m_ps[c->i2], p2);
        c->lambda += impulse;
    }
}

static
void
b2RopeSolveBend_PBD_Angle(
    struct b2Rope* p)
{
    int32 i;

    const float stiffness = p->m_tuning.bendStiffness;

    for (i = 0; i < p->m_bendCount; ++i)
    {
        b2Vec2 v;

        b2Vec2 p1;
        b2Vec2 p2;
        b2Vec2 p3;

        b2Vec2 d1;
        b2Vec2 d2;
        float a;
        float b;

        float angle;

        float L1sqr, L2sqr;

        b2Vec2 Jd1;
        b2Vec2 Jd2;

        b2Vec2 J1;
        b2Vec2 J2;
        b2Vec2 J3;

        float sum;

        float impulse;

        const struct b2RopeBend* c = &p->m_bendConstraints[i];

        b2Vec2Assign(p1, p->m_ps[c->i1]);
        b2Vec2Assign(p2, p->m_ps[c->i2]);
        b2Vec2Assign(p3, p->m_ps[c->i3]);

        b2Vec2Sub(d1, p2, p1);
        b2Vec2Sub(d2, p3, p2);
        a = b2Vec2CrossProduct(d1, d2);
        b = b2Vec2DotProduct(d1, d2);

        angle = b2Atan2(a, b);

        if (p->m_tuning.isometric)
        {
            L1sqr = c->L1 * c->L1;
            L2sqr = c->L2 * c->L2;
        }
        else
        {
            L1sqr = b2Vec2SquaredLength(d1);
            L2sqr = b2Vec2SquaredLength(d2);
        }

        if (L1sqr * L2sqr == 0.0f)
        {
            continue;
        }

        b2Vec2Skew(v, d1);
        b2Vec2Scale(Jd1, v, -1.0f / L1sqr);

        b2Vec2Skew(v, d2);
        b2Vec2Scale(Jd2, v, +1.0f / L2sqr);

        b2Vec2Negate(J1, Jd1);
        b2Vec2Sub(J2, Jd1, Jd2);
        b2Vec2Assign(J3, Jd2);

        if (p->m_tuning.fixedEffectiveMass)
        {
            sum = c->invEffectiveMass;
        }
        else
        {
            sum = 
                c->invMass1 * b2Vec2DotProduct(J1, J1) + 
                c->invMass2 * b2Vec2DotProduct(J2, J2) + 
                c->invMass3 * b2Vec2DotProduct(J3, J3);
        }

        if (sum == 0.0f)
        {
            sum = c->invEffectiveMass;
        }

        impulse = -stiffness * angle / sum;

        b2Vec2Scale(v, J1, c->invMass1 * impulse);
        b2Vec2Add(p1, p1, v);

        b2Vec2Scale(v, J2, c->invMass2 * impulse);
        b2Vec2Add(p2, p2, v);

        b2Vec2Scale(v, J3, c->invMass3 * impulse);
        b2Vec2Add(p3, p3, v);

        b2Vec2Assign(p->m_ps[c->i1], p1);
        b2Vec2Assign(p->m_ps[c->i2], p2);
        b2Vec2Assign(p->m_ps[c->i3], p3);
    }
}

static
void
b2RopeSolveBend_XPBD_Angle(
    struct b2Rope* p,
    float dt)
{
    int32 i;

    b2Assert(dt > 0.0f);

    for (i = 0; i < p->m_bendCount; ++i)
    {
        b2Vec2 v;

        b2Vec2 p1;
        b2Vec2 p2;
        b2Vec2 p3;

        b2Vec2 dp1;
        b2Vec2 dp2;
        b2Vec2 dp3;

        b2Vec2 d1;
        b2Vec2 d2;

        float L1sqr, L2sqr;

        float a;
        float b;

        float angle;

        b2Vec2 Jd1;
        b2Vec2 Jd2;

        b2Vec2 J1;
        b2Vec2 J2;
        b2Vec2 J3;

        float sum;

        float alpha;
        float beta;
        float sigma;
        float C;

        float Cdot;

        float B;
        float sum2;

        float impulse;

        struct b2RopeBend* c = &p->m_bendConstraints[i];

        b2Vec2Assign(p1, p->m_ps[c->i1]);
        b2Vec2Assign(p2, p->m_ps[c->i2]);
        b2Vec2Assign(p3, p->m_ps[c->i3]);

        b2Vec2Sub(dp1, p1, p->m_p0s[c->i1]);
        b2Vec2Sub(dp2, p2, p->m_p0s[c->i2]);
        b2Vec2Sub(dp3, p3, p->m_p0s[c->i3]);

        b2Vec2Sub(d1, p2, p1);
        b2Vec2Sub(d2, p3, p2);

        if (p->m_tuning.isometric)
        {
            L1sqr = c->L1 * c->L1;
            L2sqr = c->L2 * c->L2;
        }
        else
        {
            L1sqr = b2Vec2SquaredLength(d1);
            L2sqr = b2Vec2SquaredLength(d2);
        }

        if (L1sqr * L2sqr == 0.0f)
        {
            continue;
        }

        a = b2Vec2CrossProduct(d1, d2);
        b = b2Vec2DotProduct(d1, d2);

        angle = b2Atan2(a, b);

        b2Vec2Skew(v, d1);
        b2Vec2Scale(Jd1, v, -1.0f / L1sqr);

        b2Vec2Skew(v, d2);
        b2Vec2Scale(Jd2, v, +1.0f / L2sqr);

        b2Vec2Negate(J1, Jd1);
        b2Vec2Sub(J2, Jd1, Jd2);
        b2Vec2Assign(J3, Jd2);

        if (p->m_tuning.fixedEffectiveMass)
        {
            sum = c->invEffectiveMass;
        }
        else
        {
            sum = 
                c->invMass1 * b2Vec2DotProduct(J1, J1) + 
                c->invMass2 * b2Vec2DotProduct(J2, J2) + 
                c->invMass3 * b2Vec2DotProduct(J3, J3);
        }

        if (sum == 0.0f)
        {
            continue;
        }

        alpha = 1.0f / (c->spring * dt * dt);
        beta = dt * dt * c->damper;
        sigma = alpha * beta / dt;
        C = angle;

        // This is using the initial velocities
        Cdot = b2Vec2DotProduct(J1, dp1) + b2Vec2DotProduct(J2, dp2) + b2Vec2DotProduct(J3, dp3);

        B = C + alpha * c->lambda + sigma * Cdot;
        sum2 = (1.0f + sigma) * sum + alpha;

        impulse = -B / sum2;

        b2Vec2Scale(v, J1, c->invMass1 * impulse);
        b2Vec2Add(p1, p1, v);

        b2Vec2Scale(v, J2, c->invMass2 * impulse);
        b2Vec2Add(p2, p2, v);

        b2Vec2Scale(v, J3, c->invMass3 * impulse);
        b2Vec2Add(p3, p3, v);

        b2Vec2Assign(p->m_ps[c->i1], p1);
        b2Vec2Assign(p->m_ps[c->i2], p2);
        b2Vec2Assign(p->m_ps[c->i3], p3);
        c->lambda += impulse;
    }
}

static
void
b2RopeSolveBend_PBD_Distance(
    struct b2Rope* p)
{
    int32 i;

    const float stiffness = p->m_tuning.bendStiffness;

    for (i = 0; i < p->m_bendCount; ++i)
    {
        b2Vec2 v;

        int32 i1;
        int32 i2;

        b2Vec2 p1;
        b2Vec2 p2;

        b2Vec2 d;
        float L;

        float sum;

        float s1;
        float s2;

        const struct b2RopeBend* c = &p->m_bendConstraints[i];

        i1 = c->i1;
        i2 = c->i3;

        b2Vec2Assign(p1, p->m_ps[i1]);
        b2Vec2Assign(p2, p->m_ps[i2]);

        b2Vec2Sub(d, p2, p1);
        L = b2Vec2Normalize(d, d);

        sum = c->invMass1 + c->invMass3;
        if (sum == 0.0f)
        {
            continue;
        }

        s1 = c->invMass1 / sum;
        s2 = c->invMass3 / sum;

        b2Vec2Scale(v, d, stiffness * s1 * (c->L1 + c->L2 - L));
        b2Vec2Sub(p1, p1, v);

        b2Vec2Scale(v, d, stiffness * s2 * (c->L1 + c->L2 - L));
        b2Vec2Add(p2, p2, v);

        b2Vec2Assign(p->m_ps[i1], p1);
        b2Vec2Assign(p->m_ps[i2], p2);
    }
}

// Constraint based implementation of:
// P. Volino: Simple Linear Bending Stiffness in Particle Systems
static
void
b2RopeSolveBend_PBD_Height(
    struct b2Rope* p)
{
    int32 i;

    const float stiffness = p->m_tuning.bendStiffness;

    for (i = 0; i < p->m_bendCount; ++i)
    {
        b2Vec2 v, v0, v1;

        b2Vec2 p1;
        b2Vec2 p2;
        b2Vec2 p3;

        b2Vec2 d;
        float dLen;

        b2Vec2 dHat;

        b2Vec2 J1;
        b2Vec2 J2;
        b2Vec2 J3;

        float sum;

        float C;
        float mass;
        float impulse;

        const struct b2RopeBend* c = &p->m_bendConstraints[i];

        b2Vec2Assign(p1, p->m_ps[c->i1]);
        b2Vec2Assign(p2, p->m_ps[c->i2]);
        b2Vec2Assign(p3, p->m_ps[c->i3]);

        // Barycentric coordinates are held constant
        b2Vec2Scale(v0, p1, c->alpha1);
        b2Vec2Scale(v1, p3, c->alpha2);
        b2Vec2Add(v, v0, v1);
        b2Vec2Sub(d, v, p2);
        dLen = b2Vec2Length(d);

        if (dLen == 0.0f)
        {
            continue;
        }

        b2Vec2Scale(dHat, d, 1.0f / dLen);

        b2Vec2Scale(J1, dHat, c->alpha1);
        b2Vec2Negate(J2, dHat);
        b2Vec2Scale(J3, dHat, c->alpha2);

        sum = 
            c->invMass1 * c->alpha1 * c->alpha1 + 
            c->invMass2 + 
            c->invMass3 * c->alpha2 * c->alpha2;

        if (sum == 0.0f)
        {
            continue;
        }

        C = dLen;
        mass = 1.0f / sum;
        impulse = -stiffness * mass * C;

        b2Vec2Scale(v, J1, c->invMass1 * impulse);
        b2Vec2Add(p1, p1, v);

        b2Vec2Scale(v, J2, c->invMass2 * impulse);
        b2Vec2Add(p2, p2, v);

        b2Vec2Scale(v, J3, c->invMass3 * impulse);
        b2Vec2Add(p3, p3, v);

        b2Vec2Assign(p->m_ps[c->i1], p1);
        b2Vec2Assign(p->m_ps[c->i2], p2);
        b2Vec2Assign(p->m_ps[c->i3], p3);
    }
}

// M. Kelager: A Triangle Bending Constraint Model for PBD
static
void
b2RopeSolveBend_PBD_Triangle(
    struct b2Rope* p)
{
    int32 i;

    const float stiffness = p->m_tuning.bendStiffness;

    for (i = 0; i < p->m_bendCount; ++i)
    {
        b2Vec2 v;

        b2Vec2 b0;
        b2Vec2 bv;
        b2Vec2 b1;

        float wb0;
        float wbv;
        float wb1;

        float W;
        float invW;

        b2Vec2 d;

        b2Vec2 db0;
        b2Vec2 dbv;
        b2Vec2 db1;

        const struct b2RopeBend* c = &p->m_bendConstraints[i];

        b2Vec2Assign(b0, p->m_ps[c->i1]);
        b2Vec2Assign(bv, p->m_ps[c->i2]);
        b2Vec2Assign(b1, p->m_ps[c->i3]);

        wb0 = c->invMass1;
        wbv = c->invMass2;
        wb1 = c->invMass3;

        W = wb0 + wb1 + 2.0f * wbv;
        invW = stiffness / W;

        b2Vec2Add(v, b0, bv);
        b2Vec2Add(v, v, b1);
        b2Vec2Scale(v, v, 1.0f / 3.0f);
        b2Vec2Sub(d, bv, v);

        b2Vec2Scale(db0, d, +2.0f * wb0 * invW);
        b2Vec2Scale(dbv, d, -4.0f * wbv * invW);
        b2Vec2Scale(db1, d, +2.0f * wb1 * invW);

        b2Vec2Add(b0, b0, db0);
        b2Vec2Add(bv, bv, dbv);
        b2Vec2Add(b1, b1, db1);

        b2Vec2Assign(p->m_ps[c->i1], b0);
        b2Vec2Assign(p->m_ps[c->i2], bv);
        b2Vec2Assign(p->m_ps[c->i3], b1);
    }
}

static
void 
b2RopeApplyBendForces(
    struct b2Rope* p,
    float dt)
{
    int32 i;

    // omega = 2 * pi * hz
    const float omega = 2.0f * b2_pi * p->m_tuning.bendHertz;

    for (i = 0; i < p->m_bendCount; ++i)
    {
        b2Vec2 v;

        b2Vec2 p1;
        b2Vec2 p2;
        b2Vec2 p3;

        b2Vec2 v1;
        b2Vec2 v2;
        b2Vec2 v3;

        b2Vec2 d1;
        b2Vec2 d2;

        float L1sqr, L2sqr;

        float a;
        float b;

        float angle;

        b2Vec2 Jd1;
        b2Vec2 Jd2;

        b2Vec2 J1;
        b2Vec2 J2;
        b2Vec2 J3;

        float sum;

        float mass;

        float spring;
        float damper;

        float C;
        float Cdot;

        float impulse;

        b2Vec2Ref r1;
        b2Vec2Ref r2;
        b2Vec2Ref r3;

        const struct b2RopeBend* c = &p->m_bendConstraints[i];

        b2Vec2Assign(p1, p->m_ps[c->i1]);
        b2Vec2Assign(p2, p->m_ps[c->i2]);
        b2Vec2Assign(p3, p->m_ps[c->i3]);

        b2Vec2Assign(v1, p->m_vs[c->i1]);
        b2Vec2Assign(v2, p->m_vs[c->i2]);
        b2Vec2Assign(v3, p->m_vs[c->i3]);

        b2Vec2Sub(d1, p2, p1);
        b2Vec2Sub(d2, p3, p2);

        if (p->m_tuning.isometric)
        {
            L1sqr = c->L1 * c->L1;
            L2sqr = c->L2 * c->L2;
        }
        else
        {
            L1sqr = b2Vec2SquaredLength(d1);
            L2sqr = b2Vec2SquaredLength(d2);
        }

        if (L1sqr * L2sqr == 0.0f)
        {
            continue;
        }

        a = b2Vec2CrossProduct(d1, d2);
        b = b2Vec2DotProduct(d1, d2);

        angle = b2Atan2(a, b);

        b2Vec2Skew(v, d1);
        b2Vec2Scale(Jd1, v, -1.0f / L1sqr);

        b2Vec2Skew(v, d2);
        b2Vec2Scale(Jd2, v, +1.0f / L2sqr);

        b2Vec2Negate(J1, Jd1);
        b2Vec2Sub(J2, Jd1, Jd2);
        b2Vec2Assign(J3, Jd2);

        if (p->m_tuning.fixedEffectiveMass)
        {
            sum = c->invEffectiveMass;
        }
        else
        {
            sum = 
                c->invMass1 * b2Vec2DotProduct(J1, J1) + 
                c->invMass2 * b2Vec2DotProduct(J2, J2) + 
                c->invMass3 * b2Vec2DotProduct(J3, J3);
        }

        if (sum == 0.0f)
        {
            continue;
        }

        mass = 1.0f / sum;

        spring = mass * omega * omega;
        damper = 2.0f * mass * p->m_tuning.bendDamping * omega;

        C = angle;
        Cdot = b2Vec2DotProduct(J1, v1) + b2Vec2DotProduct(J2, v2) + b2Vec2DotProduct(J3, v3);

        impulse = -dt * (spring * C + damper * Cdot);

        r1 = p->m_vs[c->i1];
        b2Vec2Scale(v, J1, c->invMass1 * impulse);
        b2Vec2Add(r1, r1, v);

        r2 = p->m_vs[c->i2];
        b2Vec2Scale(v, J2, c->invMass2 * impulse);
        b2Vec2Add(r2, r2, v);

        r3 = p->m_vs[c->i3];
        b2Vec2Scale(v, J3, c->invMass3 * impulse);
        b2Vec2Add(r3, r3, v);
    }
}

B2_API
void
b2RopeStep(
    struct b2Rope* p,
    float dt,
    int32 iterations,
    const b2Vec2 position)
{
    int32 i;

    float inv_dt;
    float d;

    if (dt == 0.0f)
    {
        return;
    }

    inv_dt = 1.0f / dt;
    d = expf(-dt * p->m_tuning.damping);

    // Apply gravity and damping
    for (i = 0; i < p->m_count; ++i)
    {
        if (p->m_invMasses[i] > 0.0f)
        {
            b2Vec2 v;
            b2Vec2Ref u = p->m_vs[i];
            b2Vec2Scale(u, u, d);
            b2Vec2Scale(v, p->m_gravity, dt);
            b2Vec2Add(u, u, v);
        }
        else
        {
            b2Vec2 v;
            b2Vec2Ref u = p->m_vs[i];
            b2Vec2Add(v, p->m_bindPositions[i], position);
            b2Vec2Sub(v, v, p->m_p0s[i]);
            b2Vec2Scale(u, v, inv_dt);
        }
    }

    // Apply bending spring
    if (p->m_tuning.bendingModel == b2BendingModelSpringAngle)
    {
        b2RopeApplyBendForces(p, dt);
    }

    for (i = 0; i < p->m_bendCount; ++i)
    {
        p->m_bendConstraints[i].lambda = 0.0f;
    }

    for (i = 0; i < p->m_stretchCount; ++i)
    {
        p->m_stretchConstraints[i].lambda = 0.0f;
    }

    // Update position
    for (i = 0; i < p->m_count; ++i)
    {
        b2Vec2 v;
        b2Vec2Ref u = p->m_ps[i];
        b2Vec2Scale(v, p->m_vs[i], dt);
        b2Vec2Add(u, u, v);
    }

    // Solve constraints
    for (i = 0; i < iterations; ++i)
    {
        switch (p->m_tuning.bendingModel)
        {
        case b2BendingModelPBDAngle:
            b2RopeSolveBend_PBD_Angle(p);
            break;
        case b2BendingModelXPBDAngle:
            b2RopeSolveBend_XPBD_Angle(p, dt);
            break;
        case b2BendingModelPBDDistance:
            b2RopeSolveBend_PBD_Distance(p);
            break;
        case b2BendingModelPBDHeight:
            b2RopeSolveBend_PBD_Height(p);
            break;
        case b2BendingModelPBDTriangle:
            b2RopeSolveBend_PBD_Triangle(p);
            break;
        default:
            break;
        }

        switch (p->m_tuning.stretchingModel)
        {
        case b2StretchingModelPBD:
            b2RopeSolveStretch_PBD(p);
            break;
        case b2StretchingModelXPBD:
            b2RopeSolveStretch_XPBD(p, dt);
            break;
        default:
            break;
        }
    }

    // Constrain velocity
    for (i = 0; i < p->m_count; ++i)
    {
        b2Vec2 v;
        b2Vec2Sub(v, p->m_ps[i], p->m_p0s[i]);
        b2Vec2Scale(p->m_vs[i], v, inv_dt);
        b2Vec2Assign(p->m_p0s[i], p->m_ps[i]);
    }
}

B2_API
void
b2RopeReset(
    struct b2Rope* p,
    const b2Vec2 position)
{
    int32 i;

    b2Vec2Assign(p->m_position, position);

    for (i = 0; i < p->m_count; ++i)
    {
        b2Vec2 v;
        b2Vec2Add(v, p->m_bindPositions[i], p->m_position);
        b2Vec2Assign(p->m_ps[i], v);
        b2Vec2Assign(p->m_p0s[i], v);
        b2Vec2SetZero(p->m_vs[i]);
    }

    for (i = 0; i < p->m_bendCount; ++i)
    {
        p->m_bendConstraints[i].lambda = 0.0f;
    }

    for (i = 0; i < p->m_stretchCount; ++i)
    {
        p->m_stretchConstraints[i].lambda = 0.0f;
    }
}

B2_API
void
b2RopeDraw(
    const struct b2Rope* p,
    struct b2Draw* draw)
{
    static const b2Color sc = { 0.4f, 0.5f, 0.7f, 1.0f };
    static const b2Color pg = { 0.1f, 0.8f, 0.1f, 1.0f };
    static const b2Color pd = { 0.7f, 0.2f, 0.4f, 1.0f };

    int32 i;

    b2ColorConstRef pc;

    for (i = 0; i < p->m_count - 1; ++i)
    {
        b2DrawSegment(draw, p->m_ps[i], p->m_ps[i + 1], sc);

        pc = p->m_invMasses[i] > 0.0f ? pd : pg;
        b2DrawPoint(draw, p->m_ps[i], 5.0f, pc);
    }

    pc = p->m_invMasses[p->m_count - 1] > 0.0f ? pd : pg;
    b2DrawPoint(draw, p->m_ps[p->m_count - 1], 5.0f, pc);
}
