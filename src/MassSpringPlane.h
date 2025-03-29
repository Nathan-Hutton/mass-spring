#pragma once

#include <GL/glew.h>
#include <array>
#include "Physics.h"

class MassSpringPlane
{
    public:
        MassSpringPlane(float width, size_t resolution=1)
        {
            const size_t numPointsPerSide{ resolution + 1 };

            // Make mass spring points
            for (size_t j{ 0 }; j < numPointsPerSide; ++j)
            {
                float y{ glm::mix(-width, width, static_cast<float>(j) / resolution) };
                for (size_t i{ 0 }; i < numPointsPerSide; ++i)
                {
                    float x{ glm::mix(-width, width, static_cast<float>(i) / resolution) };
                    glm::vec3 pos{ x, y, 0.0f };
                    const bool fixed{ j == resolution && (i == 0 || i == resolution) };
                    m_points.push_back({ pos, glm::vec3{ 0.0f }, glm::vec3{ 0.0f }, fixed });

                    m_vertices.push_back(pos.x);
                    m_vertices.push_back(pos.y);
                    m_vertices.push_back(pos.z);
                }
            }

            m_vertices.resize(m_points.size() * 3);
            for (size_t i{ 0 }; i < m_points.size(); ++i)
            {
                m_vertices[i * 3] = m_points[i].position.x;
                m_vertices[i * 3 + 1] = m_points[i].position.y;
                m_vertices[i * 3 + 2] = m_points[i].position.z;
            }

            // Make springs
            for (size_t j{ 0 }; j < numPointsPerSide; ++j) 
            {
                for (size_t i{ 0 }; i < numPointsPerSide; ++i) 
                {
                    const size_t index{ j * numPointsPerSide + i };

                    // Horizontal
                    if (i < resolution)
                    {
                        const size_t nextIndex{ index + 1 };
                        m_springs.push_back({index, nextIndex, glm::length(m_points[index].position - m_points[nextIndex].position), m_stiffness});
                    }

                    // Vertical
                    if (j < resolution)
                    {
                        const size_t nextIndex{ (j + 1) * numPointsPerSide + i };
                        m_springs.push_back({index, nextIndex, glm::length(m_points[index].position - m_points[nextIndex].position), m_stiffness});
                    }

                    // Diagonals
                    if (j < resolution && i < resolution)
                    {
                        // Connection from index to i+1, j+1
                        const size_t bottomRight{ (j + 1) * numPointsPerSide + (i + 1) };
                        m_springs.push_back({index, bottomRight, glm::length(m_points[index].position - m_points[bottomRight].position), m_stiffness});

                        // Connection from bottom i+1, j to i,j+1
                        const size_t upperRight{ j * numPointsPerSide + (i + 1) };
                        const size_t bottomLeft{ (j + 1) * numPointsPerSide + i };
                        m_springs.push_back({upperRight, bottomLeft, glm::length(m_points[upperRight].position - m_points[bottomLeft].position), m_stiffness});
                    }
                }
            }

            // Handle indices
            for (size_t j{ 0 }; j < resolution; ++j)
            {
                for (size_t i{ 0 }; i < resolution; ++i)
                {
                    const size_t a{ j * numPointsPerSide + i };
                    const size_t b{ j * numPointsPerSide + (i + 1) };
                    const size_t c{ (j + 1) * numPointsPerSide + i };
                    const size_t d{ (j + 1) * numPointsPerSide + (i + 1) };

                    m_indices.push_back(a);
                    m_indices.push_back(b);
                    m_indices.push_back(d);

                    m_indices.push_back(a);
                    m_indices.push_back(d);
                    m_indices.push_back(c);
                }
            }

            m_degreesOfFreedom = m_points.size() * 3;
            m_massMatrixInverse = Eigen::MatrixXf::Identity(m_degreesOfFreedom, m_degreesOfFreedom).inverse(); // Using identity since every point's mass = 1

            glGenVertexArrays(1, &m_VAO);

            glBindVertexArray(m_VAO);

            glGenBuffers(1, &m_EBO);

            glGenBuffers(1, &m_VBO);

            glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * m_vertices.size(), m_vertices.data(), GL_STATIC_DRAW);

            // Set vertex attributes
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0); // Vertex positions
            glEnableVertexAttribArray(0);

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_EBO);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * m_indices.size(), m_indices.data(), GL_STATIC_DRAW);

            glBindBuffer(GL_ARRAY_BUFFER, 0);
            glBindVertexArray(0);
        }

        void updateVBO()
        {
            for (size_t i{ 0 }; i < m_points.size(); ++i)
            {
                m_vertices[i * 3] = m_points[i].position.x;
                m_vertices[i * 3 + 1] = m_points[i].position.y;
                m_vertices[i * 3 + 2] = m_points[i].position.z;
            }

            glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * m_vertices.size(), m_vertices.data());
        }

        void updatePhysics(float deltaTime)
        {
            Eigen::VectorXf force(m_degreesOfFreedom);
            Eigen::MatrixXf stiffnessMatrix(m_degreesOfFreedom, m_degreesOfFreedom);
            force.setZero();
            stiffnessMatrix.setZero();

            Eigen::VectorXf velocity(m_degreesOfFreedom);
            for (size_t i{ 0 }; i < m_points.size(); ++i) {
                velocity(3 * i + 0) = m_points[i].velocity.x;
                velocity(3 * i + 1) = m_points[i].velocity.y;
                velocity(3 * i + 2) = m_points[i].velocity.z;
            }

            Physics::getForceFromGravity(m_points, force);
            Physics::handleSpringForces(m_springs, m_points, force, stiffnessMatrix);

            // I commented this line and the b calculation line out since m_massMatrixInverse is just identity
            //Eigen::MatrixXf A{ Eigen::MatrixXf::Identity(m_degreesOfFreedom, m_degreesOfFreedom) - deltaTime * deltaTime * m_massMatrixInverse * stiffnessMatrix };
            Eigen::MatrixXf A{ Eigen::MatrixXf::Identity(m_degreesOfFreedom, m_degreesOfFreedom) - deltaTime * deltaTime * stiffnessMatrix };
            const float epsilon = 1e-5f + 0.01f * m_stiffness * 0.1f;
            A += epsilon * Eigen::MatrixXf::Identity(m_degreesOfFreedom, m_degreesOfFreedom); // Regularization
            //const Eigen::VectorXf b{ velocity + deltaTime * m_massMatrixInverse * force };
            const Eigen::VectorXf b{ velocity + deltaTime * force };
            const Eigen::VectorXf vNext{ A.ldlt().solve(b) };

            // Set new values
            Physics::setNewPoints(m_points, vNext, deltaTime);

            // Update VBO
            updateVBO();
        }

        void draw() const
        {
            glBindVertexArray(m_VAO);
            glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
        }

    private:
        GLuint m_VAO, m_VBO, m_EBO;
        std::vector<Physics::MassPoint> m_points{};
        std::vector<Physics::Spring> m_springs{};
        std::vector<GLfloat> m_vertices{};
        std::vector<GLuint> m_indices{};
        Eigen::MatrixXf m_massMatrixInverse{};
        int m_degreesOfFreedom{};
        float m_stiffness{ 50.0f };
};
