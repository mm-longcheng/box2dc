// MIT License

// Copyright (c) 2019 Erin Catto

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

#include "test.h"

class Heavy1 : public Test
{
public:
    
    Heavy1()
	{
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);
            
            b2Vec2 a = { -40.0f, 0.0f };
            b2Vec2 b = { 40.0f, 0.0f };
            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
			b2ShapeEdgeSetTwoSided(&shape, a, b);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
		}
        
        struct b2BodyDef bd;
        b2BodyDefReset(&bd);
		bd.type = b2BodyTypeDynamic;
		b2Vec2Make(bd.position, 0.0f, 0.5f);
        struct b2Body* body = b2WorldCreateBody(m_world, &bd);
        
        struct b2ShapeCircle shape;
        b2ShapeCircleReset(&shape);
		shape.m_radius = 0.5f;
        b2BodyCreateFixtureFromShape(body, &shape, 10.0f);
        
        b2Vec2Make(bd.position, 0.0f, 6.0f);
        body = b2WorldCreateBody(m_world, &bd);
        shape.m_radius = 5.0f;
        b2BodyCreateFixtureFromShape(body, &shape, 10.0f);
	}
    
	static Test* Create()
	{
		return new Heavy1;
	}
};

static int testIndex = RegisterTest("Solver", "Heavy 1", Heavy1::Create);
