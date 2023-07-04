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

// A basic slider crank created for GDC tutorial: Understanding Constraints
class SliderCrank1 : public Test
{
public:
	SliderCrank1()
	{
		struct b2Body* ground = NULL;
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            b2Vec2Make(bd.position, 0.0f, 17.0f);
			ground = b2WorldCreateBody(m_world, &bd);
		}
        
		{
            struct b2Body* prevBody = ground;
            
			// Define crank.
			{
                struct b2ShapePolygon shape;
                b2ShapePolygonReset(&shape);
				b2ShapePolygonSetAsBox(&shape, 4.0f, 1.0f);
                
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, -8.0f, 20.0f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromShape(body, &shape, 2.0f);
                
                b2Vec2 anchor = { -12.0f, 20.0f };
                struct b2JointRevoluteDef rjd;
                b2JointRevoluteDefReset(&rjd);
				b2JointRevoluteDefInitialize(&rjd, prevBody, body, anchor);
                b2WorldCreateJoint(m_world, &rjd);
                
				prevBody = body;
			}
            
			// Define connecting rod
			{
				struct b2ShapePolygon shape;
                b2ShapePolygonReset(&shape);
				b2ShapePolygonSetAsBox(&shape, 8.0f, 1.0f);
                
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				b2Vec2Make(bd.position, 4.0f, 20.0f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromShape(body, &shape, 2.0f);
                
                b2Vec2 anchor = { -4.0f, 20.0f };
                struct b2JointRevoluteDef rjd;
                b2JointRevoluteDefReset(&rjd);
				b2JointRevoluteDefInitialize(&rjd, prevBody, body, anchor);
                b2WorldCreateJoint(m_world, &rjd);
                
				prevBody = body;
			}
            
			// Define piston
			{
                struct b2ShapePolygon shape;
                b2ShapePolygonReset(&shape);
				b2ShapePolygonSetAsBox(&shape, 3.0f, 3.0f);
                
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
				bd.type = b2BodyTypeDynamic;
				bd.fixedRotation = true;
				b2Vec2Make(bd.position, 12.0f, 20.0f);
                struct b2Body* body = b2WorldCreateBody(m_world, &bd);
				b2BodyCreateFixtureFromShape(body, &shape, 2.0f);
                
                b2Vec2 anchor = { 12.0f, 20.0f };
                struct b2JointRevoluteDef rjd;
                b2JointRevoluteDefReset(&rjd);
				b2JointRevoluteDefInitialize(&rjd, prevBody, body, anchor);
                b2WorldCreateJoint(m_world, &rjd);
                
                b2Vec2 anchor1 = { 12.0f, 17.0f };
                b2Vec2 axis1 = { 1.0f, 0.0f };
                struct b2JointPrismaticDef pjd;
                b2JointPrismaticDefReset(&pjd);
				b2JointPrismaticDefInitialize(&pjd, ground, body, anchor1, axis1);
                b2WorldCreateJoint(m_world, &pjd);
			}
  		}
	}
    
	static Test* Create()
	{
		return new SliderCrank1;
	}
};

static int testIndex = RegisterTest("Examples", "Slider Crank 1", SliderCrank1::Create);
