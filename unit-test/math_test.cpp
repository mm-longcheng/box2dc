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

DOCTEST_TEST_CASE("math test")
{
	SUBCASE("sweep")
	{
		// From issue #447
		struct b2Sweep sweep = { 0 };
        b2SweepReset(&sweep);
        b2Vec2SetZero(sweep.localCenter);
		b2Vec2Make(sweep.c0, -2.0f, 4.0f);
		b2Vec2Make(sweep.c, 3.0f, 8.0f);
		sweep.a0 = 0.5f;
		sweep.a = 5.0f;
		sweep.alpha0 = 0.0f;

		b2Transform transform;

		b2SweepGetTransform(&sweep, transform, 0.0f);
		DOCTEST_REQUIRE_EQ(transform[0][0], sweep.c0[0]);
		DOCTEST_REQUIRE_EQ(transform[0][1], sweep.c0[1]);
		DOCTEST_REQUIRE_EQ(transform[1][1], cosf(sweep.a0));
		DOCTEST_REQUIRE_EQ(transform[1][0], sinf(sweep.a0));

		b2SweepGetTransform(&sweep, transform, 1.0f);
		DOCTEST_REQUIRE_EQ(transform[0][0], sweep.c[0]);
		DOCTEST_REQUIRE_EQ(transform[0][1], sweep.c[1]);
		DOCTEST_REQUIRE_EQ(transform[1][1], cosf(sweep.a));
		DOCTEST_REQUIRE_EQ(transform[1][0], sinf(sweep.a));
	}
}
