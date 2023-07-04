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

#include "draw.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include "imgui/imgui.h"

#define BUFFER_OFFSET(x)  ((const void*) (x))

DebugDraw g_debugDraw;
Camera g_camera;

//
Camera::Camera()
{
	m_width = 1280;
	m_height = 800;
	ResetView();
}

//
void Camera::ResetView()
{
    b2Vec2Make(this->m_center, 0.0f, 20.0f);
	m_zoom = 1.0f;
}

//
void Camera::ConvertScreenToWorld(const b2Vec2 ps, b2Vec2 pw)
{
	float w = float(m_width);
	float h = float(m_height);
	float u = ps[0] / w;
	float v = (h - ps[1]) / h;

	float ratio = w / h;
    b2Vec2 extents = { ratio * 25.0f, 25.0f };
    b2Vec2Scale(extents, extents, m_zoom);

    b2Vec2 lower;
    b2Vec2 upper;
    b2Vec2Sub(lower, m_center, extents);
    b2Vec2Add(upper, m_center, extents);

	pw[0] = (1.0f - u) * lower[0] + u * upper[0];
	pw[1] = (1.0f - v) * lower[1] + v * upper[1];
}

//
void Camera::ConvertWorldToScreen(const b2Vec2 pw, b2Vec2 ps)
{
	float w = float(m_width);
	float h = float(m_height);
	float ratio = w / h;
    b2Vec2 extents = { ratio * 25.0f, 25.0f };
    b2Vec2Scale(extents, extents, m_zoom);

    b2Vec2 lower;
    b2Vec2 upper;
    b2Vec2Sub(lower, m_center, extents);
    b2Vec2Add(upper, m_center, extents);

	float u = (pw[0] - lower[0]) / (upper[0] - lower[0]);
	float v = (pw[1] - lower[1]) / (upper[1] - lower[1]);

	ps[0] = u * w;
	ps[1] = (1.0f - v) * h;
}

// Convert from world coordinates to normalized device coordinates.
// http://www.songho.ca/opengl/gl_projectionmatrix.html
void Camera::BuildProjectionMatrix(float* m, float zBias)
{
	float w = float(m_width);
	float h = float(m_height);
	float ratio = w / h;
    b2Vec2 extents = { ratio * 25.0f, 25.0f };
    b2Vec2Scale(extents, extents, m_zoom);

    b2Vec2 lower;
    b2Vec2 upper;
    b2Vec2Sub(lower, m_center, extents);
    b2Vec2Add(upper, m_center, extents);

	m[0] = 2.0f / (upper[0] - lower[0]);
	m[1] = 0.0f;
	m[2] = 0.0f;
	m[3] = 0.0f;

	m[4] = 0.0f;
	m[5] = 2.0f / (upper[1] - lower[1]);
	m[6] = 0.0f;
	m[7] = 0.0f;

	m[8] = 0.0f;
	m[9] = 0.0f;
	m[10] = 1.0f;
	m[11] = 0.0f;

	m[12] = -(upper[0] + lower[0]) / (upper[0] - lower[0]);
	m[13] = -(upper[1] + lower[1]) / (upper[1] - lower[1]);
	m[14] = zBias;
	m[15] = 1.0f;
}

//
static void sCheckGLError()
{
	GLenum errCode = glGetError();
	if (errCode != GL_NO_ERROR)
	{
		fprintf(stderr, "OpenGL error = %d\n", errCode);
		assert(false);
	}
}

// Prints shader compilation errors
static void sPrintLog(GLuint object)
{
	GLint log_length = 0;
	if (glIsShader(object))
		glGetShaderiv(object, GL_INFO_LOG_LENGTH, &log_length);
	else if (glIsProgram(object))
		glGetProgramiv(object, GL_INFO_LOG_LENGTH, &log_length);
	else
	{
		fprintf(stderr, "printlog: Not a shader or a program\n");
		return;
	}

	char* log = (char*)malloc(log_length);

	if (glIsShader(object))
		glGetShaderInfoLog(object, log_length, NULL, log);
	else if (glIsProgram(object))
		glGetProgramInfoLog(object, log_length, NULL, log);

	fprintf(stderr, "%s", log);
	free(log);
}


//
static GLuint sCreateShaderFromString(const char* source, GLenum type)
{
	GLuint res = glCreateShader(type);
	const char* sources[] = { source };
	glShaderSource(res, 1, sources, NULL);
	glCompileShader(res);
	GLint compile_ok = GL_FALSE;
	glGetShaderiv(res, GL_COMPILE_STATUS, &compile_ok);
	if (compile_ok == GL_FALSE)
	{
		fprintf(stderr, "Error compiling shader of type %d!\n", type);
		sPrintLog(res);
		glDeleteShader(res);
		return 0;
	}

	return res;
}

// 
static GLuint sCreateShaderProgram(const char* vs, const char* fs)
{
	GLuint vsId = sCreateShaderFromString(vs, GL_VERTEX_SHADER);
	GLuint fsId = sCreateShaderFromString(fs, GL_FRAGMENT_SHADER);
	assert(vsId != 0 && fsId != 0);

	GLuint programId = glCreateProgram();
	glAttachShader(programId, vsId);
	glAttachShader(programId, fsId);
	glBindFragDataLocation(programId, 0, "color");
	glLinkProgram(programId);

	glDeleteShader(vsId);
	glDeleteShader(fsId);

	GLint status = GL_FALSE;
	glGetProgramiv(programId, GL_LINK_STATUS, &status);
	assert(status != GL_FALSE);

	return programId;
}

//
struct GLRenderPoints
{
	void Create()
	{
		const char* vs = \
			"#version 330\n"
			"uniform mat4 projectionMatrix;\n"
			"layout(location = 0) in vec2 v_position;\n"
			"layout(location = 1) in vec4 v_color;\n"
			"layout(location = 2) in float v_size;\n"
			"out vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	f_color = v_color;\n"
			"	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
			"   gl_PointSize = v_size;\n"
			"}\n";

		const char* fs = \
			"#version 330\n"
			"in vec4 f_color;\n"
			"out vec4 color;\n"
			"void main(void)\n"
			"{\n"
			"	color = f_color;\n"
			"}\n";

		m_programId = sCreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = 0;
		m_colorAttribute = 1;
		m_sizeAttribute = 2;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(3, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(m_vertexAttribute);
		glEnableVertexAttribArray(m_colorAttribute);
		glEnableVertexAttribArray(m_sizeAttribute);

		// Vertex buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glVertexAttribPointer(m_sizeAttribute, 1, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_sizes), m_sizes, GL_DYNAMIC_DRAW);

		sCheckGLError();

		// Cleanup
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		m_count = 0;
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(3, m_vboIds);
			m_vaoId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void Vertex(const b2Vec2 v, const b2Color c, float size)
	{
		if (m_count == e_maxVertices)
			Flush();

		b2Vec2Assign(m_vertices[m_count], v);
        b2ColorAssign(m_colors[m_count], c);
		m_sizes[m_count] = size;
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
			return;

		glUseProgram(m_programId);

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix(proj, 0.0f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Vec2), m_vertices);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Color), m_colors);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[2]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(float), m_sizes);

		glEnable(GL_PROGRAM_POINT_SIZE);
		glDrawArrays(GL_POINTS, 0, m_count);
		glDisable(GL_PROGRAM_POINT_SIZE);

		sCheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	enum { e_maxVertices = 512 };
	b2Vec2 m_vertices[e_maxVertices];
	b2Color m_colors[e_maxVertices];
	float m_sizes[e_maxVertices];

	int32 m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[3];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_vertexAttribute;
	GLint m_colorAttribute;
	GLint m_sizeAttribute;
};

//
struct GLRenderLines
{
	void Create()
	{
		const char* vs = \
			"#version 330\n"
			"uniform mat4 projectionMatrix;\n"
			"layout(location = 0) in vec2 v_position;\n"
			"layout(location = 1) in vec4 v_color;\n"
			"out vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	f_color = v_color;\n"
			"	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
			"}\n";

		const char* fs = \
			"#version 330\n"
			"in vec4 f_color;\n"
			"out vec4 color;\n"
			"void main(void)\n"
			"{\n"
			"	color = f_color;\n"
			"}\n";

		m_programId = sCreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = 0;
		m_colorAttribute = 1;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(2, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(m_vertexAttribute);
		glEnableVertexAttribArray(m_colorAttribute);

		// Vertex buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);

		sCheckGLError();

		// Cleanup
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		m_count = 0;
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(2, m_vboIds);
			m_vaoId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void Vertex(const b2Vec2 v, const b2Color c)
	{
		if (m_count == e_maxVertices)
			Flush();

        b2Vec2Assign(m_vertices[m_count], v);
        b2ColorAssign(m_colors[m_count], c);
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
			return;

		glUseProgram(m_programId);

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix(proj, 0.1f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Vec2), m_vertices);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Color), m_colors);

		glDrawArrays(GL_LINES, 0, m_count);

		sCheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	enum { e_maxVertices = 2 * 512 };
	b2Vec2 m_vertices[e_maxVertices];
	b2Color m_colors[e_maxVertices];

	int32 m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_vertexAttribute;
	GLint m_colorAttribute;
};

//
struct GLRenderTriangles
{
	void Create()
	{
		const char* vs = \
			"#version 330\n"
			"uniform mat4 projectionMatrix;\n"
			"layout(location = 0) in vec2 v_position;\n"
			"layout(location = 1) in vec4 v_color;\n"
			"out vec4 f_color;\n"
			"void main(void)\n"
			"{\n"
			"	f_color = v_color;\n"
			"	gl_Position = projectionMatrix * vec4(v_position, 0.0f, 1.0f);\n"
			"}\n";

		const char* fs = \
			"#version 330\n"
			"in vec4 f_color;\n"
			"out vec4 color;\n"
			"void main(void)\n"
			"{\n"
			"	color = f_color;\n"
			"}\n";

		m_programId = sCreateShaderProgram(vs, fs);
		m_projectionUniform = glGetUniformLocation(m_programId, "projectionMatrix");
		m_vertexAttribute = 0;
		m_colorAttribute = 1;

		// Generate
		glGenVertexArrays(1, &m_vaoId);
		glGenBuffers(2, m_vboIds);

		glBindVertexArray(m_vaoId);
		glEnableVertexAttribArray(m_vertexAttribute);
		glEnableVertexAttribArray(m_colorAttribute);

		// Vertex buffer
		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glVertexAttribPointer(m_vertexAttribute, 2, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertices), m_vertices, GL_DYNAMIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glVertexAttribPointer(m_colorAttribute, 4, GL_FLOAT, GL_FALSE, 0, BUFFER_OFFSET(0));
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_colors), m_colors, GL_DYNAMIC_DRAW);

		sCheckGLError();

		// Cleanup
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		m_count = 0;
	}

	void Destroy()
	{
		if (m_vaoId)
		{
			glDeleteVertexArrays(1, &m_vaoId);
			glDeleteBuffers(2, m_vboIds);
			m_vaoId = 0;
		}

		if (m_programId)
		{
			glDeleteProgram(m_programId);
			m_programId = 0;
		}
	}

	void Vertex(const b2Vec2 v, const b2Color c)
	{
		if (m_count == e_maxVertices)
			Flush();

        b2Vec2Assign(m_vertices[m_count], v);
        b2ColorAssign(m_colors[m_count], c);
		++m_count;
	}

	void Flush()
	{
		if (m_count == 0)
			return;

		glUseProgram(m_programId);

		float proj[16] = { 0.0f };
		g_camera.BuildProjectionMatrix(proj, 0.2f);

		glUniformMatrix4fv(m_projectionUniform, 1, GL_FALSE, proj);

		glBindVertexArray(m_vaoId);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[0]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Vec2), m_vertices);

		glBindBuffer(GL_ARRAY_BUFFER, m_vboIds[1]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, m_count * sizeof(b2Color), m_colors);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDrawArrays(GL_TRIANGLES, 0, m_count);
		glDisable(GL_BLEND);

		sCheckGLError();

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		glUseProgram(0);

		m_count = 0;
	}

	enum { e_maxVertices = 3 * 512 };
	b2Vec2 m_vertices[e_maxVertices];
	b2Color m_colors[e_maxVertices];

	int32 m_count;

	GLuint m_vaoId;
	GLuint m_vboIds[2];
	GLuint m_programId;
	GLint m_projectionUniform;
	GLint m_vertexAttribute;
	GLint m_colorAttribute;
};


void
DebugDrawDrawPolygon(
    DebugDraw* p,
    const b2Vec2* vertices,
    int32 vertexCount,
    const b2Color color)
{
    b2Vec2ConstRef p1 = vertices[vertexCount - 1];
    for (int32 i = 0; i < vertexCount; ++i)
    {
        b2Vec2ConstRef p2 = vertices[i];
        p->m_lines->Vertex(p1, color);
        p->m_lines->Vertex(p2, color);
        p1 = p2;
    }
}

void
DebugDrawDrawSolidPolygon(
    DebugDraw* p,
    const b2Vec2* vertices,
    int32 vertexCount,
    const b2Color color)
{
    const b2Color fillColor = { 0.5f * color[0], 0.5f * color[1], 0.5f * color[2], 0.5f };

    for (int32 i = 1; i < vertexCount - 1; ++i)
    {
        p->m_triangles->Vertex(vertices[0], fillColor);
        p->m_triangles->Vertex(vertices[i], fillColor);
        p->m_triangles->Vertex(vertices[i + 1], fillColor);
    }

    b2Vec2ConstRef p1 = vertices[vertexCount - 1];
    for (int32 i = 0; i < vertexCount; ++i)
    {
        b2Vec2ConstRef p2 = vertices[i];
        p->m_lines->Vertex(p1, color);
        p->m_lines->Vertex(p2, color);
        p1 = p2;
    }
}

void
DebugDrawDrawCircle(
    DebugDraw* p,
    const b2Vec2 center,
    float radius,
    const b2Color color)
{
    const float k_segments = 16.0f;
    const float k_increment = 2.0f * b2_pi / k_segments;
    float sinInc = sinf(k_increment);
    float cosInc = cosf(k_increment);
    b2Vec2 r1 = { 1.0f, 0.0f };
    b2Vec2 v1;
    b2Vec2Scale(v1, r1, radius);
    b2Vec2Add(v1, center, v1);
    for (int32 i = 0; i < k_segments; ++i)
    {
        // Perform rotation to avoid additional trigonometry.
        b2Vec2 r2;
        r2[0] = cosInc * r1[0] - sinInc * r1[1];
        r2[1] = sinInc * r1[0] + cosInc * r1[1];
        b2Vec2 v2;

        b2Vec2Scale(v2, r2, radius);
        b2Vec2Add(v2, center, v2);
        p->m_lines->Vertex(v1, color);
        p->m_lines->Vertex(v2, color);
        b2Vec2Assign(r1, r2);
        b2Vec2Assign(v1, v2);
    }
}

void
DebugDrawDrawSolidCircle(
    DebugDraw* p,
    const b2Vec2 center,
    float radius,
    const b2Vec2 axis,
    const b2Color color)
{
    const float k_segments = 16.0f;
    const float k_increment = 2.0f * b2_pi / k_segments;
    float sinInc = sinf(k_increment);
    float cosInc = cosf(k_increment);
    b2Vec2 v0;
    b2Vec2Assign(v0, center);
    b2Vec2 r1 = { cosInc, sinInc };
    b2Vec2 v1;
    b2Vec2Scale(v1, r1, radius);
    b2Vec2Add(v1, center, v1);
    b2Color fillColor = { 0.5f * color[0], 0.5f * color[1], 0.5f * color[2], 0.5f };
    for (int32 i = 0; i < k_segments; ++i)
    {
        // Perform rotation to avoid additional trigonometry.
        b2Vec2 r2;
        r2[0] = cosInc * r1[0] - sinInc * r1[1];
        r2[1] = sinInc * r1[0] + cosInc * r1[1];
        b2Vec2 v2;
        b2Vec2Scale(v2, r2, radius);
        b2Vec2Add(v2, center, v2);
        p->m_triangles->Vertex(v0, fillColor);
        p->m_triangles->Vertex(v1, fillColor);
        p->m_triangles->Vertex(v2, fillColor);
        b2Vec2Assign(r1, r2);
        b2Vec2Assign(v1, v2);
    }

    b2Vec2Make(r1, 1.0f, 0.0f);
    b2Vec2Scale(v1, r1, radius);
    b2Vec2Add(v1, center, v1);
    for (int32 i = 0; i < k_segments; ++i)
    {
        b2Vec2 r2;
        r2[0] = cosInc * r1[0] - sinInc * r1[1];
        r2[1] = sinInc * r1[0] + cosInc * r1[1];
        b2Vec2 v2;
        b2Vec2Scale(v2, r2, radius);
        b2Vec2Add(v2, center, v2);
        p->m_lines->Vertex(v1, color);
        p->m_lines->Vertex(v2, color);
        b2Vec2Assign(r1, r2);
        b2Vec2Assign(v1, v2);
    }

    // Draw a line fixed in the circle to animate rotation.
    b2Vec2 pa;
    b2Vec2Scale(pa, axis, radius);
    b2Vec2Add(pa, center, pa);
    p->m_lines->Vertex(center, color);
    p->m_lines->Vertex(pa, color);
}

void
DebugDrawDrawSegment(
    DebugDraw* p,
    const b2Vec2 p1,
    const b2Vec2 p2,
    const b2Color color)
{
    p->m_lines->Vertex(p1, color);
    p->m_lines->Vertex(p2, color);
}

void
DebugDrawDrawTransform(
    DebugDraw* p,
    const b2Transform xf)
{
    const float k_axisScale = 0.4f;
    b2Color red = { 1.0f, 0.0f, 0.0f };
    b2Color green = { 0.0f, 1.0f, 0.0f };
    b2Vec2 p1, p2;
    b2Vec2 v;
    b2Vec2Assign(p1, xf[0]);

    p->m_lines->Vertex(p1, red);
    b2RotGetXAxis(xf[1], v);
    b2Vec2Scale(v, v, k_axisScale);
    b2Vec2Add(p2, p1, v);
    p->m_lines->Vertex(p2, red);

    p->m_lines->Vertex(p1, green);
    b2RotGetYAxis(xf[1], v);
    b2Vec2Scale(v, v, k_axisScale);
    b2Vec2Add(p2, p1, v);
    p->m_lines->Vertex(p2, green);
}

void
DebugDrawDrawPoint(
    DebugDraw* p,
    const b2Vec2 point,
    float size,
    const b2Color color)
{
    p->m_points->Vertex(point, color, size);
}

const struct b2DrawMeta b2DrawMetaDebugDraw =
{
    DebugDrawDrawPolygon,
    DebugDrawDrawSolidPolygon,
    DebugDrawDrawCircle,
    DebugDrawDrawSolidCircle,
    DebugDrawDrawSegment,
    DebugDrawDrawTransform,
    DebugDrawDrawPoint,
};

//
DebugDraw::DebugDraw()
{
    this->Meta = &b2DrawMetaDebugDraw;

	m_showUI = true;
	m_points = NULL;
	m_lines = NULL;
	m_triangles = NULL;
}

//
DebugDraw::~DebugDraw()
{
	b2Assert(m_points == NULL);
	b2Assert(m_lines == NULL);
	b2Assert(m_triangles == NULL);

    this->Meta = &b2DrawMetaDebugDraw;
}

//
void DebugDraw::Create()
{
	m_points = new GLRenderPoints;
	m_points->Create();
	m_lines = new GLRenderLines;
	m_lines->Create();
	m_triangles = new GLRenderTriangles;
	m_triangles->Create();
}

//
void DebugDraw::Destroy()
{
	m_points->Destroy();
	delete m_points;
	m_points = NULL;

	m_lines->Destroy();
	delete m_lines;
	m_lines = NULL;

	m_triangles->Destroy();
	delete m_triangles;
	m_triangles = NULL;
}

void DebugDraw::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color color)
{
    DebugDrawDrawPolygon(this, vertices, vertexCount, color);
}

void DebugDraw::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color color)
{
    DebugDrawDrawSolidPolygon(this, vertices, vertexCount, color);
}

void DebugDraw::DrawCircle(const b2Vec2 center, float radius, const b2Color color)
{
    DebugDrawDrawCircle(this, center, radius, color);
}

void DebugDraw::DrawSolidCircle(const b2Vec2 center, float radius, const b2Vec2 axis, const b2Color color)
{
    DebugDrawDrawSolidCircle(this, center, radius, axis, color);
}

void DebugDraw::DrawSegment(const b2Vec2 p1, const b2Vec2 p2, const b2Color color)
{
    DebugDrawDrawSegment(this, p1, p2, color);
}

void DebugDraw::DrawTransform(const b2Transform xf)
{
    DebugDrawDrawTransform(this, xf);
}

void DebugDraw::DrawPoint(const b2Vec2 point, float size, const b2Color color)
{
    DebugDrawDrawPoint(this, point, size, color);
}

//
void DebugDraw::DrawString(int x, int y, const char* string, ...)
{
	if (m_showUI == false)
	{
		return;
	}

	va_list arg;
	va_start(arg, string);
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::SetCursorPos(ImVec2(float(x), float(y)));
	ImGui::TextColoredV(ImColor(230, 153, 153, 255), string, arg);
	ImGui::End();
	va_end(arg);
}

//
void DebugDraw::DrawString(const b2Vec2& pw, const char* string, ...)
{
    b2Vec2 ps;
    g_camera.ConvertWorldToScreen(pw, ps);

	va_list arg;
	va_start(arg, string);
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::SetCursorPos(ImVec2(ps[0], ps[1]));
	ImGui::TextColoredV(ImColor(230, 153, 153, 255), string, arg);
	ImGui::End();
	va_end(arg);
}

//
void DebugDraw::DrawAABB(b2AABB* aabb, const b2Color& c)
{
    b2Vec2 p1 = { aabb->lowerBound[0], aabb->lowerBound[1] };
    b2Vec2 p2 = { aabb->upperBound[0], aabb->lowerBound[1] };
    b2Vec2 p3 = { aabb->upperBound[0], aabb->upperBound[1] };
    b2Vec2 p4 = { aabb->lowerBound[0], aabb->upperBound[1] };

	m_lines->Vertex(p1, c);
	m_lines->Vertex(p2, c);

	m_lines->Vertex(p2, c);
	m_lines->Vertex(p3, c);

	m_lines->Vertex(p3, c);
	m_lines->Vertex(p4, c);

	m_lines->Vertex(p4, c);
	m_lines->Vertex(p1, c);
}

//
void DebugDraw::Flush()
{
	m_triangles->Flush();
	m_lines->Flush();
	m_points->Flush();
}
