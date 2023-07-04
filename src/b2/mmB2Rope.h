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

#ifndef __mmB2Rope_h__
#define __mmB2Rope_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"

#include "b2/mmB2Prefix.h"

struct b2Draw;
struct b2RopeStretch;
struct b2RopeBend;

enum b2StretchingModel
{
    b2StretchingModelPBD,
    b2StretchingModelXPBD,
};

enum b2BendingModel
{
    b2BendingModelSpringAngle = 0,
    b2BendingModelPBDAngle,
    b2BendingModelXPBDAngle,
    b2BendingModelPBDDistance,
    b2BendingModelPBDHeight,
    b2BendingModelPBDTriangle,
};

struct b2RopeTuning
{
    enum b2StretchingModel stretchingModel;
    enum b2BendingModel bendingModel;
    float damping;
    float stretchStiffness;
    float stretchHertz;
    float stretchDamping;
    float bendStiffness;
    float bendHertz;
    float bendDamping;
    int isometric;
    int fixedEffectiveMass;
    int warmStart;
};

B2_API
void
b2RopeTuningReset(
    struct b2RopeTuning* p);

struct b2RopeDef
{
    b2Vec2 position;
    b2Vec2* vertices;
    int32 count;
    float* masses;
    b2Vec2 gravity;
    struct b2RopeTuning tuning;
};

B2_API
void
b2RopeDefReset(
    struct b2RopeDef* p);

struct b2Rope
{
    b2Vec2 m_position;

    int32 m_count;
    int32 m_stretchCount;
    int32 m_bendCount;

    struct b2RopeStretch* m_stretchConstraints;
    struct b2RopeBend* m_bendConstraints;

    b2Vec2* m_bindPositions;
    b2Vec2* m_ps;
    b2Vec2* m_p0s;
    b2Vec2* m_vs;

    float* m_invMasses;
    b2Vec2 m_gravity;

    struct b2RopeTuning m_tuning;
};

B2_API
void
b2RopeInit(
    struct b2Rope* p);

B2_API
void
b2RopeDestroy(
    struct b2Rope* p);

B2_API
void
b2RopeCreate(
    struct b2Rope* p, 
    const struct b2RopeDef* def);

B2_API
void
b2RopeSetTuning(
    struct b2Rope* p, 
    const struct b2RopeTuning* tuning);

B2_API
void
b2RopeStep(
    struct b2Rope* p, 
    float dt,
    int32 iterations, 
    const b2Vec2 position);

B2_API
void
b2RopeReset(
    struct b2Rope* p, 
    const b2Vec2 position);

B2_API
void
b2RopeDraw(
    const struct b2Rope* p, 
    struct b2Draw* draw);

#include "b2/mmB2Suffix.h"

#endif//__mmB2Rope_h__
