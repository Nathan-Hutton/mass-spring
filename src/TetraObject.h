#pragma once

#include <fstream>
#include <sstream>
#include <unordered_map>
#include <GL/glew.h>
#include <array>
#include <set>
#include <functional>
#include <chrono>
#include <array>
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

class TetraObject
{
    public:
        TetraObject(const std::string& filePath, float uniformScalingFactor)
        {
            // fill m_points
            std::ifstream file{filePath + ".node"};
            if (!file.is_open())
                std::cerr << "Failed to open node file.\n";

            std::string line{};
            std::getline(file, line);
            while (std::getline(file, line))
            {
                if (line[0] == '#') continue;

                std::istringstream lineStream{ line };
                glm::vec3 pos;
                int index;
                lineStream >> index >> pos.x >> pos.y >> pos.z;

                m_points.push_back({ pos * uniformScalingFactor, glm::vec3{ 0.0f }, false });
            }

            m_vertices.resize(m_points.size() * 3);
            for (size_t i{ 0 }; i < m_points.size(); ++i)
            {
                m_vertices[i * 3] = m_points[i].position.x;
                m_vertices[i * 3 + 1] = m_points[i].position.y;
                m_vertices[i * 3 + 2] = m_points[i].position.z;
            }

            // Fill m_springs
            file = std::ifstream{filePath + ".ele"};
            if (!file.is_open())
                std::cerr << "Failed to open ele file.\n";

            struct Array3Hash {
                std::size_t operator()(const std::array<size_t, 3>& arr) const {
                    std::size_t h1{ std::hash<size_t>{}(arr[0]) };
                    std::size_t h2{ std::hash<size_t>{}(arr[1]) };
                    std::size_t h3{ std::hash<size_t>{}(arr[2]) };
                    return h1 ^ (h2 << 1) ^ (h3 << 2);
                }
            };

            // When we have a face on the boundary, [a,b,c], as part of tetrahedron, [a,b,c,d],
            // if a vector from the face to d is in the same direction as the normal, then the winding order is wrong
            auto fixWinding = [this](const std::array<size_t, 3>& face, size_t d) -> std::array<size_t, 3> {
                const glm::vec3 a{ m_points[face[0]].position };
                const glm::vec3 b{ m_points[face[1]].position };
                const glm::vec3 c{ m_points[face[2]].position };
                const glm::vec3 dp{ m_points[d].position };

                const glm::vec3 normal{ glm::normalize(glm::cross(b - a, c - a)) };
                const glm::vec3 toD{ glm::normalize(dp - a) };

                if (glm::dot(normal, toD) > 0.0f)
                    return { face[0], face[2], face[1] };
                return face;
            };

            // Store faces in sorted index order so duplicates don't have different winding order
            // Key is the count and the unsorted face so that I can retrieve them later
            std::unordered_map<std::array<size_t, 3>, std::pair<int, std::array<size_t, 3>>, Array3Hash> faceCounts;
            std::getline(file, line);
            while (std::getline(file, line))
            {
                if (line[0] == '#') continue;

                std::istringstream lineStream{ line };
                size_t index, i0, i1, i2, i3;
                lineStream >> index >> i0 >> i1 >> i2 >> i3;

                makeSpring(i0, i1);
                makeSpring(i0, i2);
                makeSpring(i0, i3);
                makeSpring(i1, i2);
                makeSpring(i1, i3);
                makeSpring(i2, i3);

                std::array<size_t, 3> face1{ fixWinding({ i0, i1, i2 }, i3) };
                std::array<size_t, 3> face2{ fixWinding({ i0, i1, i3 }, i2) };
                std::array<size_t, 3> face3{ fixWinding({ i0, i2, i3 }, i1) };
                std::array<size_t, 3> face4{ fixWinding({ i1, i2, i3 }, i0) };
                for (const std::array<size_t, 3>& face : {face1, face2, face3, face4})
                {
                    std::array<size_t, 3> sortedFace{ face };
                    std::sort(sortedFace.begin(), sortedFace.end());

                    auto it{ faceCounts.find(sortedFace) };
                    if (it == faceCounts.end())
                        faceCounts[sortedFace] = { 1, face };
                    else
                        ++it->second.first;
                }
            }

            for (const auto& [_, value] : faceCounts)
            {
                if (value.first != 1) continue;

                std::array<size_t, 3> face{ value.second };
                m_indices.push_back(static_cast<GLuint>(face[0]));
                m_indices.push_back(static_cast<GLuint>(face[1]));
                m_indices.push_back(static_cast<GLuint>(face[2]));
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

        void makeSpring(size_t i0, size_t i1)
        {
            const Eigen::Vector3f pi
            {
                m_points[i0].position.x,
                m_points[i0].position.y,
                m_points[i0].position.z
            };

            const Eigen::Vector3f pj
            {
                m_points[i1].position.x,
                m_points[i1].position.y,
                m_points[i1].position.z
            };

            const Eigen::Vector3f d{ pi - pj };
            const float len{ d.norm() };

            const Eigen::Matrix3f outer{ (d / len) * (d / len).transpose() };
            const Eigen::Matrix3f K{ m_stiffness * (Eigen::Matrix3f::Identity() - outer) };
            m_springs.push_back({ i0, i1, glm::distance(m_points[i0].position, m_points[i1].position), m_stiffness, K });
        }
};
