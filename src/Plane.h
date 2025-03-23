#pragma once

#include <GL/glew.h>
#include <array>

class CollisionPlane
{
    public:
        CollisionPlane(float width, float worldHeight)
        {
            //const std::array<GLfloat, 18> vertices
            //{
            //    -width, worldHeight, -width, // Bottom left
            //     width, worldHeight, -width, // Bottom right
            //    -width,  worldHeight, width, // Top left

            //     width, worldHeight, -width, // Bottom right
            //     width,  worldHeight, width, // Top right
            //    -width,  worldHeight, width  // Top left
            //};
            const std::array<GLfloat, 18> vertices
            {
                -width, worldHeight, width, // bottom left
                 width, worldHeight, width, // bottom right
                 width, worldHeight, -width, // top right

                -width, worldHeight, width, // bottom left
                 width, worldHeight, -width, // top right
                -width, worldHeight, -width  // top left
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
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }

    private:
        GLuint m_VAO;
};
