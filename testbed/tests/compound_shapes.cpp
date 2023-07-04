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
#include "imgui/imgui.h"

class CompoundShapes : public Test
{
public:
	CompoundShapes()
	{
		{
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			b2Vec2Make(bd.position, 0.0f, 0.0f);
            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            b2Vec2 a = { 50.0f, 0.0f };
            b2Vec2 b = { -50.0f, 0.0f };
            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);
			b2ShapeEdgeSetTwoSided(&shape, a, b);

            b2BodyCreateFixtureFromShape(body, &shape, 0.0f);
		}

		// Table 1
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, -15.0f, 1.0f);
			m_table1 = b2WorldCreateBody(m_world, &bd);

            b2Vec2 center0 = { 0.0f, 3.5f };
            struct b2ShapePolygon top;
            b2ShapePolygonReset(&top);
			b2ShapePolygonSetAsBoxDetail(&top, 3.0f, 0.5f, center0, 0.0f);

            b2Vec2 center1 = { -2.5f, 1.5f };
            struct b2ShapePolygon leftLeg;
            b2ShapePolygonReset(&leftLeg);
			b2ShapePolygonSetAsBoxDetail(&leftLeg, 0.5f, 1.5f, center1, 0.0f);

            b2Vec2 center2 = { 2.5f, 1.5f };
            struct b2ShapePolygon rightLeg;
            b2ShapePolygonReset(&rightLeg);
			b2ShapePolygonSetAsBoxDetail(&rightLeg, 0.5f, 1.5f, center2, 0.0f);

			b2BodyCreateFixtureFromShape(m_table1, &top, 2.0f);
			b2BodyCreateFixtureFromShape(m_table1, &leftLeg, 2.0f);
			b2BodyCreateFixtureFromShape(m_table1, &rightLeg, 2.0f);
		}

		// Table 2
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, -5.0f, 1.0f);
			m_table2 = b2WorldCreateBody(m_world, &bd);

            b2Vec2 center0 = { 0.0f, 3.5f };
            struct b2ShapePolygon top;
            b2ShapePolygonReset(&top);
			b2ShapePolygonSetAsBoxDetail(&top, 3.0f, 0.5f, center0, 0.0f);

            b2Vec2 center1 = { -2.5f, 2.0f };
            struct b2ShapePolygon leftLeg;
            b2ShapePolygonReset(&leftLeg);
			b2ShapePolygonSetAsBoxDetail(&leftLeg, 0.5f, 2.0f, center1, 0.0f);

            b2Vec2 center2 = { 2.5f, 2.0f };
            struct b2ShapePolygon rightLeg;
            b2ShapePolygonReset(&rightLeg);
			b2ShapePolygonSetAsBoxDetail(&rightLeg, 0.5f, 2.0f, center2, 0.0f);

			b2BodyCreateFixtureFromShape(m_table2, &top, 2.0f);
			b2BodyCreateFixtureFromShape(m_table2, &leftLeg, 2.0f);
			b2BodyCreateFixtureFromShape(m_table2, &rightLeg, 2.0f);
		}

		// Spaceship 1
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 5.0f, 1.0f);
			m_ship1 = b2WorldCreateBody(m_world, &bd);

			b2Vec2 vertices[3];

            struct b2ShapePolygon left;
            b2ShapePolygonReset(&left);
			b2Vec2Make(vertices[0], -2.0f, 0.0f);
			b2Vec2Make(vertices[1], 0.0f, 4.0f / 3.0f);
			b2Vec2Make(vertices[2], 0.0f, 4.0f);
			b2ShapePolygonSetPoints(&left, vertices, 3);

            struct b2ShapePolygon right;
            b2ShapePolygonReset(&right);
            b2Vec2Make(vertices[0], 2.0f, 0.0f);
            b2Vec2Make(vertices[1], 0.0f, 4.0f / 3.0f);
            b2Vec2Make(vertices[2], 0.0f, 4.0f);
			b2ShapePolygonSetPoints(&right, vertices, 3);

			b2BodyCreateFixtureFromShape(m_ship1, &left, 2.0f);
			b2BodyCreateFixtureFromShape(m_ship1, &right, 2.0f);
		}

		// Spaceship 2
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Make(bd.position, 15.0f, 1.0f);
			m_ship2 = b2WorldCreateBody(m_world, &bd);

			b2Vec2 vertices[3];

            struct b2ShapePolygon left;
            b2ShapePolygonReset(&left);
            b2Vec2Make(vertices[0], -2.0f, 0.0f);
            b2Vec2Make(vertices[1], 1.0f, 2.0f);
            b2Vec2Make(vertices[2], 0.0f, 4.0f);
			b2ShapePolygonSetPoints(&left, vertices, 3);

            struct b2ShapePolygon right;
            b2ShapePolygonReset(&right);
            b2Vec2Make(vertices[0], 2.0f, 0.0f);
            b2Vec2Make(vertices[1], -1.0f, 2.0f);
            b2Vec2Make(vertices[2], 0.0f, 4.0f);
			b2ShapePolygonSetPoints(&right, vertices, 3);

			b2BodyCreateFixtureFromShape(m_ship2, &left, 2.0f);
			b2BodyCreateFixtureFromShape(m_ship2, &right, 2.0f);
		}
	}

	void Spawn()
	{
		// Table 1 obstruction
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Assign(bd.position, b2BodyGetPosition(m_table1));
			bd.angle = b2BodyGetAngle(m_table1);

            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            b2Vec2 center = { 0.0f, 3.0f };
            struct b2ShapePolygon box;
            b2ShapePolygonReset(&box);
			b2ShapePolygonSetAsBoxDetail(&box, 4.0f, 0.1f, center, 0.0f);
			
			b2BodyCreateFixtureFromShape(body, &box, 2.0f);
		}

		// Table 2 obstruction
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
            b2Vec2Assign(bd.position, b2BodyGetPosition(m_table2));
			bd.angle = b2BodyGetAngle(m_table2);

            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            b2Vec2 center = { 0.0f, 3.0f };
            struct b2ShapePolygon box;
            b2ShapePolygonReset(&box);
			b2ShapePolygonSetAsBoxDetail(&box, 4.0f, 0.1f, center, 0.0f);
			
			b2BodyCreateFixtureFromShape(body, &box, 2.0f);
		}

		// Ship 1 obstruction
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Assign(bd.position, b2BodyGetPosition(m_ship1));
			bd.angle = b2BodyGetAngle(m_ship1);
			bd.gravityScale = 0.0f;

            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeCircle circle;
            b2ShapeCircleReset(&circle);
			circle.m_radius = 0.5f;
			b2Vec2Make(circle.m_p, 0.0f, 2.0f);

			b2BodyCreateFixtureFromShape(body, &circle, 2.0f);
		}

		// Ship 2 obstruction
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Assign(bd.position, b2BodyGetPosition(m_ship2));
			bd.angle = b2BodyGetAngle(m_ship2);
			bd.gravityScale = 0.0f;

            struct b2Body* body = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeCircle circle;
            b2ShapeCircleReset(&circle);
			circle.m_radius = 0.5f;
			b2Vec2Make(circle.m_p, 0.0f, 2.0f);

			b2BodyCreateFixtureFromShape(body, &circle, 2.0f);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f));
		ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::Button("Spawn"))
		{
			Spawn();
		}

		ImGui::End();
	}

	static Test* Create()
	{
		return new CompoundShapes;
	}

	b2Body* m_table1;
	b2Body* m_table2;
	b2Body* m_ship1;
	b2Body* m_ship2;
};

static int testIndex = RegisterTest("Examples", "Compound Shapes", CompoundShapes::Create);
