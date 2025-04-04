#pragma once

#include <GL/glew.h>
#include <array>
#include <set>
#include <chrono>
#include "Physics.h"

struct TripletInfo {
    int row;
    int col;
    size_t index;
};

struct SpringStiffnessIndices {
    std::array<std::array<size_t, 3>, 3> i_i;
    std::array<std::array<size_t, 3>, 3> j_j;
    std::array<std::array<size_t, 3>, 3> i_j;
    std::array<std::array<size_t, 3>, 3> j_i;
};

class MassSpringPlane
{
    public:
        MassSpringPlane(float width, size_t resolution=1)
        {
            const size_t numPointsPerSide{ resolution + 1 };
            m_points.resize(numPointsPerSide * numPointsPerSide);
            m_degreesOfFreedom = m_points.size() * 3;

            // Make mass spring points
            size_t index{ 0 };
            for (size_t j{ 0 }; j < numPointsPerSide; ++j)
            {
                float y{ glm::mix(-width, width, static_cast<float>(j) / resolution) };
                for (size_t i{ 0 }; i < numPointsPerSide; ++i)
                {
                    float x{ glm::mix(-width, width, static_cast<float>(i) / resolution) };
                    glm::vec3 pos{ x, y, 0.0f };
                    const bool fixed{ j == resolution && (i == 0 || i == resolution) };

                    Physics::MassPoint point{ pos, glm::vec3{ 0.0f }, fixed };
                    m_points[index++] = point;
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
                        const Eigen::Vector3f pi(
                            m_points[index].position.x,
                            m_points[index].position.y,
                            m_points[index].position.z);

                        const Eigen::Vector3f pj(
                            m_points[nextIndex].position.x,
                            m_points[nextIndex].position.y,
                            m_points[nextIndex].position.z);

                        const Eigen::Vector3f d{ pi - pj };
                        const float len{ d.norm() };

                        const Eigen::Matrix3f outer{ (d / len) * (d / len).transpose() };
                        const Eigen::Matrix3f K{ m_stiffness * (Eigen::Matrix3f::Identity() - outer) };
                        m_springs.push_back({index, nextIndex, glm::distance(m_points[index].position, m_points[nextIndex].position), m_stiffness, K});
                    }

                    // Vertical
                    if (j < resolution)
                    {
                        const size_t nextIndex{ (j + 1) * numPointsPerSide + i };
                        const Eigen::Vector3f pi(
                            m_points[index].position.x,
                            m_points[index].position.y,
                            m_points[index].position.z);

                        const Eigen::Vector3f pj(
                            m_points[nextIndex].position.x,
                            m_points[nextIndex].position.y,
                            m_points[nextIndex].position.z);

                        const Eigen::Vector3f d = pi - pj;
                        const float len = d.norm();

                        const Eigen::Matrix3f outer = (d / len) * (d / len).transpose();
                        const Eigen::Matrix3f K = m_stiffness * (Eigen::Matrix3f::Identity() - outer);
                        m_springs.push_back({index, nextIndex, glm::distance(m_points[index].position, m_points[nextIndex].position), m_stiffness, K});
                    }

                    // Diagonals
                    if (j < resolution && i < resolution)
                    {
                        // Connection from index to i+1, j+1
                        const size_t bottomRight{ (j + 1) * numPointsPerSide + (i + 1) };
                        {
                            const Eigen::Vector3f pi(
                                m_points[index].position.x,
                                m_points[index].position.y,
                                m_points[index].position.z);

                            const Eigen::Vector3f pj(
                                m_points[bottomRight].position.x,
                                m_points[bottomRight].position.y,
                                m_points[bottomRight].position.z);

                            const Eigen::Vector3f d = pi - pj;
                            const float len = d.norm();

                            const Eigen::Matrix3f outer = (d / len) * (d / len).transpose();
                            const Eigen::Matrix3f K = m_stiffness * (Eigen::Matrix3f::Identity() - outer);
                            m_springs.push_back({index, bottomRight, glm::distance(m_points[index].position, m_points[bottomRight].position), m_stiffness, K});
                        }

                        // Connection from bottom i+1, j to i,j+1
                        const size_t upperRight{ j * numPointsPerSide + (i + 1) };
                        const size_t bottomLeft{ (j + 1) * numPointsPerSide + i };
                        {
                            const Eigen::Vector3f pi(
                                m_points[upperRight].position.x,
                                m_points[upperRight].position.y,
                                m_points[upperRight].position.z);

                            const Eigen::Vector3f pj(
                                m_points[bottomLeft].position.x,
                                m_points[bottomLeft].position.y,
                                m_points[bottomLeft].position.z);

                            const Eigen::Vector3f d = pi - pj;
                            const float len = d.norm();

                            const Eigen::Matrix3f outer = (d / len) * (d / len).transpose();
                            const Eigen::Matrix3f K = m_stiffness * (Eigen::Matrix3f::Identity() - outer);
                            m_springs.push_back({upperRight, bottomLeft, glm::distance(m_points[upperRight].position, m_points[bottomLeft].position), m_stiffness, K});
                        }
                    }
                }
            }

            m_A.resize(m_degreesOfFreedom, m_degreesOfFreedom);
            m_force.resize(m_degreesOfFreedom);
            m_velocity.resize(m_degreesOfFreedom);
            m_b.resize(m_degreesOfFreedom);
            m_vNext.resize(m_degreesOfFreedom);

            std::set<std::pair<int, int>> uniqueTripletSet;
            for (int i{ 0 }; i < m_degreesOfFreedom; ++i)
                uniqueTripletSet.emplace(i, i);

            for (const Physics::Spring& spring : m_springs)
            {
                const int iBase{ 3 * static_cast<int>(spring.i) };
                const int jBase{ 3 * static_cast<int>(spring.j) };

                for (int row{ 0 }; row < 3; ++row)
                {
                    for (int col{ 0 }; col < 3; ++col)
                    {
                        uniqueTripletSet.emplace(iBase + row, iBase + col);
                        uniqueTripletSet.emplace(jBase + row, jBase + col);
                        uniqueTripletSet.emplace(iBase + row, jBase + col);
                        uniqueTripletSet.emplace(jBase + row, iBase + col);
                    }
                }
            }

            std::vector<Eigen::Triplet<float>> triplets;
            triplets.reserve(uniqueTripletSet.size());
            for (const auto& [row, col] : uniqueTripletSet)
                triplets.emplace_back(row, col, 0.0f);

            m_A.setFromTriplets(triplets.begin(), triplets.end());
            m_A.makeCompressed();
            m_solver.compute(m_A);

            m_tripletValues.resize(triplets.size());

            std::vector<TripletInfo> tripletIndexList;
            tripletIndexList.reserve(triplets.size());
            for (size_t i{ 0 }; i < triplets.size(); ++i)
                tripletIndexList.push_back({ triplets[i].row(), triplets[i].col(), i });

            m_indexGrid.resize(m_degreesOfFreedom * m_degreesOfFreedom, static_cast<size_t>(-1));
            for (const auto& t : tripletIndexList)
                m_indexGrid[t.row * m_degreesOfFreedom + t.col] = t.index;

            m_solver.setMaxIterations(50);
            m_solver.setTolerance(1e-1);

            m_springIndices.clear();
            m_springIndices.reserve(m_springs.size());

            for (const auto& spring : m_springs)
            {
                SpringStiffnessIndices indices;

                for (size_t d{ 0 }; d < 3; ++d)
                {
                    const size_t iRow{ 3 * spring.i + d };
                    const size_t jRow{ 3 * spring.j + d };

                    for (size_t e{ 0 }; e < 3; ++e)
                    {
                        const size_t iCol{ 3 * spring.i + e };
                        const size_t jCol{ 3 * spring.j + e };

                        indices.i_i[d][e] = m_indexGrid[iRow * m_degreesOfFreedom + iCol];
                        indices.j_j[d][e] = m_indexGrid[jRow * m_degreesOfFreedom + jCol];
                        indices.i_j[d][e] = m_indexGrid[iRow * m_degreesOfFreedom + jCol];
                        indices.j_i[d][e] = m_indexGrid[jRow * m_degreesOfFreedom + iCol];
                    }
                }

                m_springIndices.push_back(indices);
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
            //using clock = std::chrono::high_resolution_clock;
            //const auto start{ clock::now() };

            std::fill(m_tripletValues.begin(), m_tripletValues.end(), 0.0f);
            m_force.setZero();

            for (size_t i{ 0 }; i < m_points.size(); ++i) {
                m_velocity.segment<3>(3 * i) = Eigen::Vector3f(
                    m_points[i].velocity.x,
                    m_points[i].velocity.y,
                    m_points[i].velocity.z);
            }

            Physics::getForceFromGravity(m_points, m_force);
            Physics::getSpringForces(m_springs, m_points, m_force);

            constexpr float velocityDampingCoeff{ 3.5f };
            m_force -= velocityDampingCoeff  * m_velocity;

            const float epsilon{ 1e-4f + 0.001f * m_stiffness };
            constexpr float massDampingCoef{ 0.5f };
            const float massDampingDiagonal{ 1.0f + epsilon + deltaTime * massDampingCoef };

            // Mass + damping diagonal
            for (int i{ 0 }; i < m_degreesOfFreedom; ++i)
                m_tripletValues[m_indexGrid[i * m_degreesOfFreedom + i]] = massDampingDiagonal;

             // Add stiffness contributions per spring (directly to triplets)
            const float dt2{ -deltaTime * deltaTime };
            for (size_t i{ 0 }; i < m_springs.size(); ++i)
            {
                const Physics::Spring& spring{ m_springs[i] };
                if (m_points[spring.i].fixed && m_points[spring.j].fixed)
                    continue;

                const float dx = m_points[spring.i].position.x - m_points[spring.j].position.x;
                const float dy = m_points[spring.i].position.y - m_points[spring.j].position.y;
                const float dz = m_points[spring.i].position.z - m_points[spring.j].position.z;
                const float len2{ dx*dx + dy*dy + dz*dz };
                if (len2 < 1e-10f) continue;

                const SpringStiffnessIndices& indices{ m_springIndices[i] };
                for (size_t d{ 0 }; d < 3; ++d)
                {
                    for (size_t e{ 0 }; e < 3; ++e)
                    {
                        const float val{ dt2 * spring.K(d, e) };

                        m_tripletValues[indices.i_i[d][e]] += val;
                        m_tripletValues[indices.j_j[d][e]] += val;
                        m_tripletValues[indices.i_j[d][e]] -= val;
                        m_tripletValues[indices.j_i[d][e]] -= val;
                    }
                }
            }

            float* values{ m_A.valuePtr() };
            for (size_t i{ 0 }; i < m_tripletValues.size(); ++i)
                values[i] = m_tripletValues[i];

            // I commented this out because right now it's not doing anyting
            // But in the future I might want to call this once every 20 frames or something to keep things stable
            //m_solver.factorize(m_A);
            m_b = m_velocity + deltaTime * m_force;
            m_vNext = m_solver.solveWithGuess(m_b, m_velocity);

            Physics::setNewPoints(m_points, m_vNext, deltaTime);
            updateVBO();

            //const auto end{ clock::now() };
            //const auto elapsed{ std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() };
            //std::cout << "updatePhysics() took " << elapsed / 1000.0 << " ms" << std::endl;
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
        float m_stiffness{ 200.0f };
        Eigen::SparseMatrix<float> m_A;
        Eigen::ConjugateGradient<Eigen::SparseMatrix<float>, Eigen::Lower|Eigen::Upper, Eigen::DiagonalPreconditioner<float>> m_solver;
        Eigen::VectorXf m_force;
        Eigen::VectorXf m_velocity;
        Eigen::VectorXf m_b;
        Eigen::VectorXf m_vNext;
        std::vector<float> m_tripletValues;
        std::vector<size_t> m_indexGrid;
        std::vector<SpringStiffnessIndices> m_springIndices;
};
