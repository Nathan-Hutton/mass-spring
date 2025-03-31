#pragma once

#include <GL/glew.h>
#include <array>
#include <set>
#include <chrono>
#include "Physics.h"

struct pair_hash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);
    }
};

struct TripletInfo {
    int row;
    int col;
    size_t index;
};

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

            m_degreesOfFreedom = m_points.size() * 3;
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

            m_tripletsFixedStructure.reserve(uniqueTripletSet.size());
            for (const auto& [row, col] : uniqueTripletSet)
                m_tripletsFixedStructure.emplace_back(row, col, 0.0f);
            m_A.setFromTriplets(m_tripletsFixedStructure.begin(), m_tripletsFixedStructure.end());
            m_A.makeCompressed();
            m_solver.compute(m_A);

            m_tripletValues.resize(m_tripletsFixedStructure.size());

            {
                std::vector<TripletInfo> tripletIndexList;
                tripletIndexList.reserve(m_tripletsFixedStructure.size());
                for (size_t i{ 0 }; i < m_tripletsFixedStructure.size(); ++i)
                {
                    const auto& triplet{ m_tripletsFixedStructure[i] };
                    tripletIndexList.push_back({ triplet.row(), triplet.col(), i });
                }

                m_indexGrid.resize(m_degreesOfFreedom * m_degreesOfFreedom, static_cast<size_t>(-1));
                for (const auto& t : tripletIndexList)
                    m_indexGrid[t.row * m_degreesOfFreedom + t.col] = t.index;
            }

            m_solver.setMaxIterations(50);
            m_solver.setTolerance(1e-4);

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
            using clock = std::chrono::high_resolution_clock;
            const auto start{ clock::now() };

            std::fill(m_tripletValues.begin(), m_tripletValues.end(), 0.0f);
            m_force.setZero();

            for (size_t i{ 0 }; i < m_points.size(); ++i) {
                m_velocity.segment<3>(3 * i) = Eigen::Vector3f(
                    m_points[i].velocity.x,
                    m_points[i].velocity.y,
                    m_points[i].velocity.z);
            }

            Physics::getForceFromGravity(m_points, m_force);

            const auto springForcesStart{ clock::now() };
            Physics::getSpringForces(m_springs, m_points, m_force);
            const auto springForcesEnd{ clock::now() };
            const auto springForcesElapsed{ std::chrono::duration_cast<std::chrono::microseconds>(springForcesEnd - springForcesStart).count() };
            std::cout << "springForces took " << springForcesElapsed / 1000.0 << " ms" << std::endl;

            constexpr float velocityDampingCoeff{ 3.5f };
            m_force -= velocityDampingCoeff  * m_velocity;

            const float epsilon{ 1e-4f + 0.001f * m_stiffness };
            constexpr float massDampingCoef{ 0.5f };
            const float massDampingDiagonal{ 1.0f + epsilon + deltaTime * massDampingCoef };

            // Mass + damping diagonal
            for (int i{ 0 }; i < m_degreesOfFreedom; ++i)
                m_tripletValues[m_indexGrid[i * m_degreesOfFreedom + i]] = massDampingDiagonal;

             // Add stiffness contributions per spring (directly to triplets)
            const auto secondLoopStart{ clock::now() };

            const float dt2{ -deltaTime * deltaTime };
            for (const auto& spring : m_springs)
            {
                if (m_points[spring.i].fixed && m_points[spring.j].fixed)
                    continue;

                const float dx = m_points[spring.i].position.x - m_points[spring.j].position.x;
                const float dy = m_points[spring.i].position.y - m_points[spring.j].position.y;
                const float dz = m_points[spring.i].position.z - m_points[spring.j].position.z;
                const float len{ std::sqrt(dx * dx + dy * dy + dz * dz) };
                if (len < 1e-5f) continue;

                const size_t iBase{ 3 * spring.i };
                const size_t jBase{ 3 * spring.j };

                for (size_t row{ 0 }; row < 3; ++row)
                {
                    for (size_t col{ 0 }; col < 3; ++col)
                    {
                        const size_t iRow{ iBase + row };
                        const size_t jRow{ jBase + row };
                        const size_t iCol{ iBase + col };
                        const size_t jCol{ jBase + col };
                        const size_t iRow_iCol{ m_indexGrid[iRow * m_degreesOfFreedom + iCol] };
                        const size_t jRow_jCol{ m_indexGrid[jRow * m_degreesOfFreedom + jCol] };
                        const size_t iRow_jCol{ m_indexGrid[iRow * m_degreesOfFreedom + jCol] };
                        const size_t jRow_iCol{ m_indexGrid[jRow * m_degreesOfFreedom + iCol] };

                        const float val{ dt2 * spring.K(row, col) };

                        m_tripletValues[iRow_iCol] += val;
                        m_tripletValues[jRow_jCol] += val;
                        m_tripletValues[iRow_jCol] -= val;
                        m_tripletValues[jRow_iCol] -= val;
                    }
                }
            }
            const auto secondLoopEnd{ clock::now() };
            const auto secondLoopElapsed{ std::chrono::duration_cast<std::chrono::microseconds>(secondLoopEnd - secondLoopStart).count() };
            std::cout << "secondLoop took " << secondLoopElapsed / 1000.0 << " ms" << std::endl;

            const auto thirdLoopStart{ clock::now() };

            for (size_t i{ 0 }; i < m_tripletValues.size(); ++i)
            {
                auto& trip{ m_tripletsFixedStructure[i] };
                trip = Eigen::Triplet<float>(trip.row(), trip.col(), m_tripletValues[i]);
            }

            const auto thirdLoopEnd{ clock::now() };
            const auto thirdLoopElapsed{ std::chrono::duration_cast<std::chrono::microseconds>(thirdLoopEnd - thirdLoopStart).count() };
            std::cout << "thirdLoop took " << thirdLoopElapsed / 1000.0 << " ms" << std::endl;

            float* values{ m_A.valuePtr() };
            for (size_t i{ 0 }; i < m_tripletValues.size(); ++i)
                values[i] = m_tripletValues[i];

            const auto solveStart{ clock::now() };

            m_solver.factorize(m_A);
            m_b = m_velocity + deltaTime * m_force;
            m_vNext = m_solver.solveWithGuess(m_b, m_velocity);

            const auto solveEnd{ clock::now() };
            const auto solveElapsed{ std::chrono::duration_cast<std::chrono::microseconds>(solveEnd - solveStart).count() };
            std::cout << "solve took " << solveElapsed / 1000.0 << " ms" << std::endl;

            Physics::setNewPoints(m_points, m_vNext, deltaTime);
            updateVBO();

            const auto end{ clock::now() };
            const auto elapsed{ std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() };
            std::cout << "updatePhysics() took " << elapsed / 1000.0 << " ms" << std::endl;
            std::cout << '\n';
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
        std::vector<Eigen::Triplet<float>> m_tripletsFixedStructure;
        std::vector<float> m_tripletValues;
        std::vector<size_t> m_indexGrid;
};
