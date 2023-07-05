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

#ifndef __mmB2Draw_h__
#define __mmB2Draw_h__

#include "b2/mmB2Api.h"
#include "b2/mmB2Types.h"
#include "b2/mmB2Math.h"

#include "b2/mmB2Prefix.h"

/// Color for debug drawing. Each value has the range [0,1].
/// b2Color (r, g, b, a)
typedef float b2Color[4];
typedef float* b2ColorRef;
typedef const float* b2ColorConstRef;

B2_API
void
b2ColorAssign(
    float r[4],
    const float v[4]);

B2_API
void
b2ColorMake(
    float c[4],
    float r, float g, float b, float a);

B2_API
void
b2ColorMakeRGB(
    float c[4],
    float r, float g, float b);

enum b2DrawBit
{
    b2DrawBitShape				= 0x0001,	///< draw shapes
    b2DrawBitJoint				= 0x0002,	///< draw joint connections
    b2DrawBitAabb				= 0x0004,	///< draw axis aligned bounding boxes
    b2DrawBitPair				= 0x0008,	///< draw broad-phase pairs
    b2DrawBitCenterOfMass		= 0x0010,	///< draw center of mass frame
};

struct b2DrawMeta
{
    /// Draw a closed polygon provided in CCW order.
    /// void(*DrawPolygon)(
    ///     void* obj,
    ///     const b2Vec2* vertices, 
    ///     int32 vertexCount, 
    ///     const b2Color color);
    void* DrawPolygon;

    /// Draw a solid closed polygon provided in CCW order.
    /// void(*DrawSolidPolygon)(
    ///     void* obj,
    ///     const b2Vec2* vertices, 
    ///     int32 vertexCount, 
    ///     const b2Color color);
    void* DrawSolidPolygon;

    /// Draw a circle.
    /// void(*DrawCircle)(
    ///     void* obj,
    ///     const b2Vec2 center, 
    ///     float radius, 
    ///     const b2Color color);
    void* DrawCircle;

    /// Draw a solid circle.
    /// void(*DrawSolidCircle)(
    ///     void* obj,
    ///     const b2Vec2 center, 
    ///     float radius, 
    ///     const b2Vec2 axis, 
    ///     const b2Color color);
    void* DrawSolidCircle;

    /// Draw a line segment.
    /// void(*DrawSegment)(
    ///     void* obj,
    ///     const b2Vec2 p1, 
    ///     const b2Vec2 p2, 
    ///     const b2Color color);
    void* DrawSegment;

    /// Draw a transform. Choose your own length scale.
    /// @param xf a transform.
    /// void(*DrawTransform)(
    ///     void* obj,
    ///     const b2Transform xf);
    void* DrawTransform;

    /// Draw a point.
    /// void(*DrawPoint)(
    ///     void* obj,
    ///     const b2Vec2 point, 
    ///     float size, 
    ///     const b2Color color);
    void* DrawPoint;
};

B2_API extern const struct b2DrawMeta b2DrawMetaDefault;

struct b2Draw
{
    uint32 m_drawFlags;

    const struct b2DrawMeta* Meta;
};

/// draw super member.
#define b2DrawSuper             \
uint32 m_drawFlags;             \
const struct b2DrawMeta* Meta

/// Reset the drawing flags.
B2_API
void
b2DrawReset(
    struct b2Draw* p);

/// Set the drawing flags.
static
inline
void
b2DrawSetFlags(
    struct b2Draw* p, 
    uint32 flags)
{
    p->m_drawFlags = flags;
}

/// Get the drawing flags.
static
inline
uint32
b2DrawGetFlags(
    const struct b2Draw* p)
{
    return p->m_drawFlags;
}

/// Append flags to the current flags.
B2_API
void
b2DrawAppendFlags(
    struct b2Draw* p, 
    uint32 flags);

/// Clear flags from the current flags.
B2_API
void
b2DrawClearFlags(
    struct b2Draw* p, 
    uint32 flags);

/// Draw a closed polygon provided in CCW order.
B2_API
void
b2DrawPolygon(
    struct b2Draw* obj,
    const b2Vec2* vertices,
    int32 vertexCount,
    const b2Color color);

/// Draw a solid closed polygon provided in CCW order.
B2_API
void
b2DrawSolidPolygon(
    struct b2Draw* obj,
    const b2Vec2* vertices,
    int32 vertexCount,
    const b2Color color);

/// Draw a circle.
B2_API
void
b2DrawCircle(
    struct b2Draw* obj,
    const b2Vec2 center,
    float radius,
    const b2Color color);

/// Draw a solid circle.
B2_API
void
b2DrawSolidCircle(
    struct b2Draw* obj,
    const b2Vec2 center,
    float radius,
    const b2Vec2 axis,
    const b2Color color);

/// Draw a line segment.
B2_API
void
b2DrawSegment(
    struct b2Draw* obj,
    const b2Vec2 p1,
    const b2Vec2 p2,
    const b2Color color);

/// Draw a transform. Choose your own length scale.
/// @param xf a transform.
B2_API
void
b2DrawTransform(
    struct b2Draw* obj,
    const b2Transform xf);

/// Draw a point.
B2_API
void
b2DrawPoint(
    struct b2Draw* obj,
    const b2Vec2 point,
    float size,
    const b2Color color);

#include "b2/mmB2Suffix.h"

#endif//__mmB2Draw_h__
