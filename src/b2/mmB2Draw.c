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

#include "mmB2Draw.h"

#include <stddef.h>

B2_API
void
b2ColorAssign(
    float r[4],
    const float v[4])
{
    r[0] = v[0];
    r[1] = v[1];
    r[2] = v[2];
    r[3] = v[3];
}

B2_API
void
b2ColorMake(
    float c[4],
    float r, float g, float b, float a)
{
    c[0] = r;
    c[1] = g;
    c[2] = b;
    c[3] = a;
}

B2_API
void
b2ColorMakeRGB(
    float c[4],
    float r, float g, float b)
{
    c[0] = r;
    c[1] = g;
    c[2] = b;
    c[3] = 1.0f;
}

B2_API const struct b2DrawMeta b2DrawMetaDefault =
{
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
};

B2_API
void
b2DrawReset(
    struct b2Draw* p)
{
    p->m_drawFlags = 0;
    p->Meta = &b2DrawMetaDefault;
}

/// Append flags to the current flags.
B2_API
void
b2DrawAppendFlags(
    struct b2Draw* p,
    uint32 flags)
{
    p->m_drawFlags |= flags;
}

/// Clear flags from the current flags.
B2_API
void
b2DrawClearFlags(
    struct b2Draw* p,
    uint32 flags)
{
    p->m_drawFlags &= ~flags;
}

/// Draw a closed polygon provided in CCW order.
B2_API
void
b2DrawPolygon(
    struct b2Draw* obj,
    const b2Vec2* vertices,
    int32 vertexCount,
    const b2Color color)
{
    typedef
    void(*DrawPolygon)(
        void* obj,
        const b2Vec2* vertices,
        int32 vertexCount,
        const b2Color color);

    (*((DrawPolygon)obj->Meta->DrawPolygon))(
        obj, 
        vertices, 
        vertexCount, 
        color);
}

/// Draw a solid closed polygon provided in CCW order.
B2_API
void
b2DrawSolidPolygon(
    struct b2Draw* obj,
    const b2Vec2* vertices,
    int32 vertexCount,
    const b2Color color)
{
    typedef
    void(*DrawSolidPolygon)(
        void* obj,
        const b2Vec2* vertices, 
        int32 vertexCount, 
        const b2Color color);

    (*((DrawSolidPolygon)obj->Meta->DrawSolidPolygon))(
        obj, 
        vertices, 
        vertexCount, 
        color);
}

/// Draw a circle.
B2_API
void
b2DrawCircle(
    struct b2Draw* obj,
    const b2Vec2 center,
    float radius,
    const b2Color color)
{
    typedef
    void(*DrawCircle)(
        void* obj,
        const b2Vec2 center, 
        float radius, 
        const b2Color color);

    (*((DrawCircle)obj->Meta->DrawCircle))(
        obj, 
        center, 
        radius, 
        color);
}

/// Draw a solid circle.
B2_API
void
b2DrawSolidCircle(
    struct b2Draw* obj,
    const b2Vec2 center,
    float radius,
    const b2Vec2 axis,
    const b2Color color)
{
    typedef
    void(*DrawSolidCircle)(
        void* obj,
        const b2Vec2 center, 
        float radius, 
        const b2Vec2 axis, 
        const b2Color color);

    (*((DrawSolidCircle)obj->Meta->DrawSolidCircle))(
        obj, 
        center,
        radius,
        axis,
        color);
}

/// Draw a line segment.
B2_API
void
b2DrawSegment(
    struct b2Draw* obj,
    const b2Vec2 p1,
    const b2Vec2 p2,
    const b2Color color)
{
    typedef
    void(*DrawSegment)(
        void* obj,
        const b2Vec2 p1, 
        const b2Vec2 p2, 
        const b2Color color);

    (*((DrawSegment)obj->Meta->DrawSegment))(
        obj, 
        p1,
        p2,
        color);
}

/// Draw a transform. Choose your own length scale.
/// @param xf a transform.
B2_API
void
b2DrawTransform(
    struct b2Draw* obj,
    const b2Transform xf)
{
    typedef
    void(*DrawTransform)(
        void* obj,
        const b2Transform xf);

    (*((DrawTransform)obj->Meta->DrawTransform))(
        obj, 
        xf);
}

/// Draw a point.
B2_API
void
b2DrawPoint(
    struct b2Draw* obj,
    const b2Vec2 point,
    float size,
    const b2Color color)
{
    typedef
    void(*DrawPoint)(
        void* obj,
        const b2Vec2 point, 
        float size, 
        const b2Color color);

    (*((DrawPoint)obj->Meta->DrawPoint))(
        obj, 
        point,
        size,
        color);
}
