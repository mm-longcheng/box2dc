/*
Test case for collision/jerking issue.
*/

#include "test.h"

class Skier : public Test
{
public:
	Skier()
	{		
		struct b2Body* ground = NULL;
		{
            struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			ground = b2WorldCreateBody(m_world, &bd);

			float const PlatformWidth = 8.0f;

			/*
			First angle is from the horizontal and should be negative for a downward slope.
			Second angle is relative to the preceding slope, and should be positive, creating a kind of
			loose 'Z'-shape from the 3 edges.
			If A1 = -10, then A2 <= ~1.5 will result in the collision glitch.
			If A1 = -30, then A2 <= ~10.0 will result in the glitch.
			*/
			float const Angle1Degrees = -30.0f;
			float const Angle2Degrees = 10.0f;
			
			/*
			The larger the value of SlopeLength, the less likely the glitch will show up.
			*/
			float const SlopeLength = 2.0f;

			float const SurfaceFriction = 0.2f;

			// Convert to radians
			float const Slope1Incline = -Angle1Degrees * b2_pi / 180.0f;
			float const Slope2Incline = Slope1Incline - Angle2Degrees * b2_pi / 180.0f;
			//

			m_platform_width = PlatformWidth;

			// Horizontal platform
            b2Vec2 v1 = { -PlatformWidth, 0.0f };
            b2Vec2 v2 = { 0.0f, 0.0f };
            b2Vec2 v3 = { SlopeLength * cosf(Slope1Incline), -SlopeLength * sinf(Slope1Incline) };
            b2Vec2 v4 = { v3[0] + SlopeLength * cosf(Slope2Incline), v3[1] - SlopeLength * sinf(Slope2Incline) };
            b2Vec2 v5 = { v4[0], v4[1] - 1.0f };

            b2Vec2 vertices[5];
            b2Vec2Assign(vertices[0], v5);
            b2Vec2Assign(vertices[1], v4);
            b2Vec2Assign(vertices[2], v3);
            b2Vec2Assign(vertices[3], v2);
            b2Vec2Assign(vertices[4], v1);

			struct b2ShapeChain shape;
            b2ShapeChainReset(&shape);
			b2ShapeChainCreateLoop(&shape, vertices, 5);
            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.shape = (struct b2Shape*)&shape;
			fd.density = 0.0f;
			fd.friction = SurfaceFriction;

			b2BodyCreateFixtureFromDef(ground, &fd);
            b2ShapeChainClear(&shape);
		}

		{
			float const BodyWidth = 1.0f;
			float const BodyHeight = 2.5f;
			float const SkiLength = 3.0f;

			/*
			Larger values for this seem to alleviate the issue to some extent.
			*/
			float const SkiThickness = 0.3f;

			float const SkiFriction = 0.0f;
			float const SkiRestitution = 0.15f;

			struct b2BodyDef bd;
            b2BodyDefReset(&bd);
			bd.type = b2BodyTypeDynamic;

			float initial_y = BodyHeight / 2 + SkiThickness;
			b2Vec2Make(bd.position, -m_platform_width / 2, initial_y);

			struct b2Body* skier = b2WorldCreateBody(m_world, &bd);

            struct b2ShapePolygon ski;
            b2ShapePolygonReset(&ski);
			b2Vec2 verts[4];
			b2Vec2Make(verts[0], -SkiLength / 2 - SkiThickness, -BodyHeight / 2);
			b2Vec2Make(verts[1], -SkiLength / 2, -BodyHeight / 2 - SkiThickness);
			b2Vec2Make(verts[2], SkiLength / 2, -BodyHeight / 2 - SkiThickness);
			b2Vec2Make(verts[3], SkiLength / 2 + SkiThickness, -BodyHeight / 2);
			b2ShapePolygonSetPoints(&ski, verts, 4);

            struct b2FixtureDef fd;
            b2FixtureDefReset(&fd);
			fd.density = 1.0f;

			fd.friction = SkiFriction;
			fd.restitution = SkiRestitution;

			fd.shape = (struct b2Shape*)&ski;
			b2BodyCreateFixtureFromDef(skier, &fd);

            b2Vec2 LinearVelocity = { 0.5f, 0.0f };
			b2BodySetLinearVelocity(skier, LinearVelocity);

			m_skier = skier;
		}

		b2Vec2Make(g_camera.m_center, m_platform_width / 2.0f, 0.0f);
		g_camera.m_zoom = 0.4f;
		m_fixed_camera = true;
	}

	void Keyboard(int key) override
	{
		switch (key)
		{
			case GLFW_KEY_C:
			m_fixed_camera = !m_fixed_camera;
			if(m_fixed_camera)
			{
                b2Vec2Make(g_camera.m_center, m_platform_width / 2.0f, 0.0f);
			}
			break;
		}
	}

	void Step(Settings& settings) override
	{
		g_debugDraw.DrawString(5, m_textLine, "Keys: c = Camera fixed/tracking");
		m_textLine += m_textIncrement;

		if(!m_fixed_camera)
		{
			b2Vec2Assign(g_camera.m_center, b2BodyGetPosition(m_skier));
		}

		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Skier;
	}

	struct b2Body* m_skier;
	float m_platform_width;
	bool m_fixed_camera;
};

static int testIndex = RegisterTest("Bugs", "Skier", Skier::Create);
