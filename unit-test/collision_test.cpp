// MIT License

// Copyright (c) 2020 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

//#include "box2d/box2d.h"
#include "b2/mmB2Header.h"
#include "doctest.h"
#include <stdio.h>

// Unit tests for collision algorithms
DOCTEST_TEST_CASE("collision test")
{
	SUBCASE("polygon mass data")
	{
        const b2Vec2 center = { 100.0f, -50.0f };
		const float hx = 0.5f, hy = 1.5f;
		const float angle1 = 0.25f;

		// Data from issue #422. Not used because the data exceeds accuracy limits.
		//const b2Vec2 center(-15000.0f, -15000.0f);
		//const float hx = 0.72f, hy = 0.72f;
		//const float angle1 = 0.0f;

		struct b2ShapePolygon polygon1;
        b2ShapePolygonReset(&polygon1);
        b2ShapePolygonSetAsBoxDetail(&polygon1, hx, hy, center, angle1);

		const float absTol = 2.0f * b2_epsilon;
		const float relTol = 2.0f * b2_epsilon;

		CHECK(b2AbsFloat(polygon1.m_centroid[0] - center[0]) < absTol + relTol * b2AbsFloat(center[0]));
		CHECK(b2AbsFloat(polygon1.m_centroid[1] - center[1]) < absTol + relTol * b2AbsFloat(center[1]));

		b2Vec2 vertices[4];
		b2Vec2Make(vertices[0], center[0] - hx, center[1] - hy);
		b2Vec2Make(vertices[1], center[0] + hx, center[1] - hy);
		b2Vec2Make(vertices[2], center[0] - hx, center[1] + hy);
		b2Vec2Make(vertices[3], center[0] + hx, center[1] + hy);

        struct b2ShapePolygon polygon2;
        b2ShapePolygonReset(&polygon2);
		b2ShapePolygonSetPoints(&polygon2, vertices, 4);

		CHECK(b2AbsFloat(polygon2.m_centroid[0] - center[0]) < absTol + relTol * b2AbsFloat(center[0]));
		CHECK(b2AbsFloat(polygon2.m_centroid[1] - center[1]) < absTol + relTol * b2AbsFloat(center[1]));

		const float mass = 4.0f * hx * hy;
		const float inertia = (mass / 3.0f) * (hx * hx + hy * hy) + mass * b2Vec2DotProduct(center, center);

		struct b2MassData massData1;
		b2ShapePolygonComputeMass(&polygon1, &massData1, 1.0f);

		CHECK(b2AbsFloat(massData1.center[0] - center[0]) < absTol + relTol * b2AbsFloat(center[0]));
		CHECK(b2AbsFloat(massData1.center[1] - center[1]) < absTol + relTol * b2AbsFloat(center[1]));
		CHECK(b2AbsFloat(massData1.mass - mass) < 20.0f * (absTol + relTol * mass));
		CHECK(b2AbsFloat(massData1.I - inertia) < 40.0f * (absTol + relTol * inertia));

        struct b2MassData massData2;
		b2ShapePolygonComputeMass(&polygon2, &massData2, 1.0f);

		CHECK(b2AbsFloat(massData2.center[0] - center[0]) < absTol + relTol * b2AbsFloat(center[0]));
		CHECK(b2AbsFloat(massData2.center[1] - center[1]) < absTol + relTol * b2AbsFloat(center[1]));
		CHECK(b2AbsFloat(massData2.mass - mass) < 20.0f * (absTol + relTol * mass));
		CHECK(b2AbsFloat(massData2.I - inertia) < 40.0f * (absTol + relTol * inertia));
    }
}
