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

class EdgeTest : public Test
{
public:

	EdgeTest()
	{
		b2Vec2 vertices[10] =
		{
			{10.0f, -4.0f},
			{10.0f, 0.0f},
			{6.0f, 0.0f},
			{4.0f, 2.0f},
			{2.0f, 0.0f},
			{-2.0f, 0.0f},
			{-6.0f, 0.0f},
			{-8.0f, -3.0f},
			{-10.0f, 0.0f},
			{-10.0f, -4.0f}
		};

		b2Vec2Make(m_offset1, 0.0f, 8.0f);
        b2Vec2Make(m_offset2, 0.0f, 16.0f);

		{
            b2Vec2 v1;
            b2Vec2 v2;
            b2Vec2 v3;
            b2Vec2 v4;
            b2Vec2 v5;
            b2Vec2 v6;
            b2Vec2 v7;
            b2Vec2 v8;
            b2Vec2 v9;
            b2Vec2 v10;


			b2Vec2Add(v1, vertices[0], m_offset1);
            b2Vec2Add(v2, vertices[1], m_offset1);
            b2Vec2Add(v3, vertices[2], m_offset1);
            b2Vec2Add(v4, vertices[3], m_offset1);
            b2Vec2Add(v5, vertices[4], m_offset1);
            b2Vec2Add(v6, vertices[5], m_offset1);
            b2Vec2Add(v7, vertices[6], m_offset1);
            b2Vec2Add(v8, vertices[7], m_offset1);
            b2Vec2Add(v9, vertices[8], m_offset1);
            b2Vec2Add(v10, vertices[9], m_offset1);

			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);

			b2ShapeEdgeSetOneSided(&shape, v10, v1, v2, v3);
			b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetOneSided(&shape, v1, v2, v3, v4);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetOneSided(&shape, v2, v3, v4, v5);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetOneSided(&shape, v3, v4, v5, v6);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetOneSided(&shape, v4, v5, v6, v7);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetOneSided(&shape, v5, v6, v7, v8);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetOneSided(&shape, v6, v7, v8, v9);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetOneSided(&shape, v7, v8, v9, v10);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetOneSided(&shape, v8, v9, v10, v1);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetOneSided(&shape, v9, v10, v1, v2);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
        }

		{
            b2Vec2 v1;
            b2Vec2 v2;
            b2Vec2 v3;
            b2Vec2 v4;
            b2Vec2 v5;
            b2Vec2 v6;
            b2Vec2 v7;
            b2Vec2 v8;
            b2Vec2 v9;
            b2Vec2 v10;

            b2Vec2Add(v1, vertices[0], m_offset2);
            b2Vec2Add(v2, vertices[1], m_offset2);
            b2Vec2Add(v3, vertices[2], m_offset2);
            b2Vec2Add(v4, vertices[3], m_offset2);
            b2Vec2Add(v5, vertices[4], m_offset2);
            b2Vec2Add(v6, vertices[5], m_offset2);
            b2Vec2Add(v7, vertices[6], m_offset2);
            b2Vec2Add(v8, vertices[7], m_offset2);
            b2Vec2Add(v9, vertices[8], m_offset2);
            b2Vec2Add(v10, vertices[9], m_offset2);

			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
            struct b2Body* ground = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeEdge shape;
            b2ShapeEdgeReset(&shape);

			b2ShapeEdgeSetTwoSided(&shape, v1, v2);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetTwoSided(&shape, v2, v3);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetTwoSided(&shape, v3, v4);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetTwoSided(&shape, v4, v5);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetTwoSided(&shape, v5, v6);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetTwoSided(&shape, v6, v7);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetTwoSided(&shape, v7, v8);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetTwoSided(&shape, v8, v9);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetTwoSided(&shape, v9, v10);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);

			b2ShapeEdgeSetTwoSided(&shape, v10, v1);
            b2BodyCreateFixtureFromShape(ground, &shape, 0.0f);
        }

		m_body1 = nullptr;
		m_body2 = nullptr;
		CreateBoxes();
		m_boxes = true;
	}

	void CreateBoxes()
	{
		if (m_body1)
		{
			b2WorldDeleteBody(m_world, m_body1);
			m_body1 = nullptr;
		}

		if (m_body2)
		{
			b2WorldDeleteBody(m_world, m_body2);
			m_body2 = nullptr;
		}

		{
            b2Vec2 pos = { 8.0f, 2.6f };
			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Add(bd.position, pos, m_offset1);
			bd.allowSleep = false;
			m_body1 = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.5f, 1.0f);

            b2BodyCreateFixtureFromShape(m_body1, &shape, 1.0f);
		}

		{
            b2Vec2 pos = { 8.0f, 2.6f };
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
            b2Vec2Add(bd.position, pos, m_offset2);
			bd.allowSleep = false;
			m_body2 = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon shape;
            b2ShapePolygonReset(&shape);
			b2ShapePolygonSetAsBox(&shape, 0.5f, 1.0f);

			b2BodyCreateFixtureFromShape(m_body2, &shape, 1.0f);
		}
	}

	void CreateCircles()
	{
		if (m_body1)
		{
			b2WorldDeleteBody(m_world, m_body1);
			m_body1 = nullptr;
		}

		if (m_body2)
		{
			b2WorldDeleteBody(m_world, m_body2);
			m_body2 = nullptr;
		}

		{
            b2Vec2 pos = { -0.5f, 0.6f };
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
			b2Vec2Add(bd.position, pos, m_offset1);
			bd.allowSleep = false;
			m_body1 = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
			shape.m_radius = 0.5f;

			b2BodyCreateFixtureFromShape(m_body1, &shape, 1.0f);
		}

		{
            b2Vec2 pos = { -0.5f, 0.6f };
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;
            b2Vec2Add(bd.position, pos, m_offset2);
			bd.allowSleep = false;
			m_body2 = b2WorldCreateBody(m_world, &bd);

            struct b2ShapeCircle shape;
            b2ShapeCircleReset(&shape);
			shape.m_radius = 0.5f;

			b2BodyCreateFixtureFromShape(m_body2, &shape, 1.0f);
		}
	}

	void UpdateUI() override
	{
		ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
		ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f));
		ImGui::Begin("Custom Controls", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

		if (ImGui::RadioButton("Boxes", m_boxes == true))
		{
			CreateBoxes();
			m_boxes = true;
		}

		if (ImGui::RadioButton("Circles", m_boxes == false))
		{
			CreateCircles();
			m_boxes = false;
		}

		ImGui::End();
	}

	void Step(Settings& settings) override
	{
		if (glfwGetKey(g_mainWindow, GLFW_KEY_A) == GLFW_PRESS)
		{
            b2Vec2 center = { -10.0f, 0.0f };
			b2BodyApplyForceToCenter(m_body1, center, true);
			b2BodyApplyForceToCenter(m_body2, center, true);
		}

		if (glfwGetKey(g_mainWindow, GLFW_KEY_D) == GLFW_PRESS)
		{
            b2Vec2 center = { 10.0f, 0.0f };
			b2BodyApplyForceToCenter(m_body1, center, true);
			b2BodyApplyForceToCenter(m_body2, center, true);
		}

		Test::Step(settings);
	}

	static Test* Create()
	{
		return new EdgeTest;
	}

	b2Vec2 m_offset1, m_offset2;
	struct b2Body* m_body1;
    struct b2Body* m_body2;
	bool m_boxes;
};

static int testIndex = RegisterTest("Geometry", "Edge Test", EdgeTest::Create);
