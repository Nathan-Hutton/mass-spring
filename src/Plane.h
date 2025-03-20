#pragma once

#include <GL/glew.h>
#include <vector>

class Plane
{
    public:
        Plane(float width)
        {
            const std::vector<GLfloat> vertices
            {
                -width, -width, 0.0f, 0.0f, 0.0f, 1.0f,
                 width, -width, 0.0f, 0.0f, 0.0f, 1.0f,
                -width,  width, 0.0f, 0.0f, 0.0f, 1.0f,
                 width,  width, 0.0f, 0.0f, 0.0f, 1.0f
            };

            GLuint planeVBO;
            glGenVertexArrays(1, &m_VAO);

            glBindVertexArray(m_VAO);

            glGenBuffers(1, &planeVBO);

            glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

            // Set vertex attributes
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void*)0); // Vertex positions
            glEnableVertexAttribArray(0);

            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void*)(sizeof(GLfloat) * 3)); // Vertex normals
            glEnableVertexAttribArray(1);

            glBindVertexArray(0);

        }

        void draw() const
        {
            glBindVertexArray(m_VAO);
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        }

    private:
        GLuint m_VAO;
};
