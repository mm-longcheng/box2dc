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

class ChainProblem : public Test
{
public:

    ChainProblem()
    {
        {
            b2Vec2 g = { 0.0f, -10.0f };
            b2WorldSetGravity(m_world, g);
            struct b2Body** bodies = (struct b2Body**)b2Alloc(2 * sizeof(struct b2Body*));
            struct b2Joint** joints = (struct b2Joint**)b2Alloc(0 * sizeof(struct b2Joint*));
            {
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
                bd.type = b2BodyType(0);
                bodies[0] = b2WorldCreateBody(m_world, &bd);

                {
                    struct b2FixtureDef fd;
                    b2FixtureDefReset(&fd);

                    b2Vec2 v1 = { 0.0f, 1.0f };
                    b2Vec2 v2 = { 0.0f, 0.0f };
                    b2Vec2 v3 = { 4.0f, 0.0f };

                    struct b2ShapeEdge shape;
                    b2ShapeEdgeReset(&shape);
                    b2ShapeEdgeSetTwoSided(&shape, v1, v2);
                    b2BodyCreateFixtureFromShape(bodies[0], &shape, 0.0f);

                    b2ShapeEdgeSetTwoSided(&shape, v2, v3);
                    b2BodyCreateFixtureFromShape(bodies[0], &shape, 0.0f);
                }
            }
            {
                struct b2BodyDef bd;
                b2BodyDefReset(&bd);
                bd.type = b2BodyType(2);
                //bd.position.Set(6.033980250358582e-01f, 3.028350114822388e+00f);
                b2Vec2Make(bd.position, 1.0f, 3.0f);
                bodies[1] = b2WorldCreateBody(m_world, &bd);

                {
                    struct b2FixtureDef fd;
                    b2FixtureDefReset(&fd);
                    fd.friction = 0.2f;
                    fd.density = 10.0f;
                    struct b2ShapePolygon shape;
                    b2ShapePolygonReset(&shape);
                    b2Vec2 vs[8];
                    b2Vec2Make(vs[0], 0.5f, -3.0f);
                    b2Vec2Make(vs[1], 0.5f, 3.0f);
                    b2Vec2Make(vs[2], -0.5f, 3.0f);
                    b2Vec2Make(vs[3], -0.5f, -3.0f);
                    b2ShapePolygonSetPoints(&shape, vs, 4);

                    fd.shape = (struct b2Shape*)&shape;

                    b2BodyCreateFixtureFromDef(bodies[1], &fd);
                }
            }
            b2Free(joints);
            b2Free(bodies);
            joints = NULL;
            bodies = NULL;
        }
    }

    static Test* Create()
    {
        return new ChainProblem;
    }

};

static int testIndex = RegisterTest("Bugs", "Chain Problem", ChainProblem::Create);
