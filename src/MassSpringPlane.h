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

            m_degreesOfFreedom = m_points.size() * 3;

            // Set sparsity pattern for A
            {
                m_A.resize(m_degreesOfFreedom, m_degreesOfFreedom);

                std::vector<Eigen::Triplet<float>> triplets;
                Eigen::VectorXf force{ Eigen::VectorXf::Zero(m_degreesOfFreedom) };
                Physics::handleSpringForces(m_springs, m_points, force, triplets);

                m_A.setFromTriplets(triplets.begin(), triplets.end());

                for (int i{ 0 }; i < m_degreesOfFreedom; ++i)
                    m_A.coeffRef(i, i) += 1e-5f;
            }

            // Damping matrix
            {
                m_dampingMatrix.resize(m_degreesOfFreedom, m_degreesOfFreedom);
                std::vector<Eigen::Triplet<float>> triplets;

                constexpr float dampingCoefficient{ 0.4f };
                for (int i{ 0 }; i < m_degreesOfFreedom; ++i)
                    triplets.emplace_back(i, i, dampingCoefficient);
                m_dampingMatrix.setFromTriplets(triplets.begin(), triplets.end());
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

            // Setup VAO
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
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
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
            force.setZero();

            Eigen::VectorXf velocity(m_degreesOfFreedom);
            for (size_t i{ 0 }; i < m_points.size(); ++i) {
                velocity(3 * i + 0) = m_points[i].velocity.x;
                velocity(3 * i + 1) = m_points[i].velocity.y;
                velocity(3 * i + 2) = m_points[i].velocity.z;
            }

            Physics::getForceFromGravity(m_points, force);

            for (int k = 0; k < m_A.outerSize(); ++k)
                for (Eigen::SparseMatrix<float>::InnerIterator it(m_A, k); it; ++it)
                    it.valueRef() = 0.0f;

            const float epsilon = 1e-4f + 0.01f * m_stiffness * 0.1f;
            for (int i = 0; i < m_degreesOfFreedom; ++i)
                m_A.coeffRef(i, i) = 1.0f + epsilon;

            std::vector<Eigen::Triplet<float>> stiffnessTriplets;
            Physics::handleSpringForces(m_springs, m_points, force, stiffnessTriplets);
            for (const auto& triplet : stiffnessTriplets)
                m_A.coeffRef(triplet.row(), triplet.col()) -= (deltaTime * deltaTime) * triplet.value();

            for (int k{ 0 }; k < m_dampingMatrix.outerSize(); ++k)
                for (Eigen::SparseMatrix<float>::InnerIterator it(m_dampingMatrix, k); it; ++it)
                    m_A.coeffRef(it.row(), it.col()) -= deltaTime * it.value();
                                                                                                           //
            //m_solver.factorize(m_A);
            m_solver.compute(m_A);
            const Eigen::VectorXf b{ velocity + deltaTime * force };
            const Eigen::VectorXf vNext{ m_solver.solve(b) };

            // Set new values
            Physics::setNewPoints(m_points, vNext, deltaTime);
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
        int m_degreesOfFreedom{};
        float m_stiffness{ 300.0f };
        Eigen::SparseMatrix<float> m_A;
        Eigen::SparseMatrix<float> m_dampingMatrix;
        Eigen::ConjugateGradient<Eigen::SparseMatrix<float>, Eigen::Lower | Eigen::Upper> m_solver;
};
