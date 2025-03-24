#pragma once

#include <GL/glew.h>
#include <array>

class CollisionPlane
{
    public:
        CollisionPlane(float width, float worldHeight)
        {
            const std::array<GLfloat, 12> vertices
            {
                -width, worldHeight, width,   // bottom left
                 width, worldHeight, width,   // bottom right
                -width, worldHeight, -width,  // top left
                 width, worldHeight, -width   // top right
            };

            GLuint planeVBO;
            glGenVertexArrays(1, &m_VAO);

            glBindVertexArray(m_VAO);

            glGenBuffers(1, &planeVBO);

            glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

            // Set vertex attributes
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0); // Vertex positions
            glEnableVertexAttribArray(0);

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
