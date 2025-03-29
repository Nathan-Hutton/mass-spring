#pragma once

#include <GL/glew.h>
#include <array>
#include "Physics.h"

class MassSpringPlane
{
    public:
        MassSpringPlane(float width)
        {
            m_points.push_back({ glm::vec3{ -width, -1.0f, 0.0f }, glm::vec3{ 0.0f }, glm::vec3{ 0.0f }, false }); // Bottom left
            m_points.push_back({ glm::vec3{ width, -1.0f, 0.0f }, glm::vec3{ 0.0f }, glm::vec3{ 0.0f }, false }); // Bottom right
            m_points.push_back({ glm::vec3{ -width, 9.0f, 0.0f }, glm::vec3{ 0.0f }, glm::vec3{ 0.0f }, true }); // Top left
            m_points.push_back({ glm::vec3{ width, 9.0f, 0.0f }, glm::vec3{ 0.0f }, glm::vec3{ 0.0f }, true }); // Top right

            constexpr float stiffness{ 50.0f };
            m_springs = 
            {
                Physics::Spring{0, 1, glm::length(m_points[0].position - m_points[1].position), stiffness},
                Physics::Spring{0, 2, glm::length(m_points[0].position - m_points[2].position), stiffness},
                Physics::Spring{1, 3, glm::length(m_points[1].position - m_points[3].position), stiffness},
                Physics::Spring{2, 3, glm::length(m_points[2].position - m_points[3].position), stiffness}
            };

            m_vertices.resize(12);
            for (size_t i{ 0 }; i < 4; ++i)
            {
                m_vertices[i * 3] = m_points[i].position.x;
                m_vertices[i * 3 + 1] = m_points[i].position.y;
                m_vertices[i * 3 + 2] = m_points[i].position.z;
            }

            m_degreesOfFreedom = 12;
            m_massMatrixInverse = Eigen::MatrixXf::Identity(m_degreesOfFreedom, m_degreesOfFreedom).inverse();

            glGenVertexArrays(1, &m_VAO);

            glBindVertexArray(m_VAO);

            glGenBuffers(1, &m_VBO);

            glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
            glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * m_vertices.size(), m_vertices.data(), GL_STATIC_DRAW);

            // Set vertex attributes
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0); // Vertex positions
            glEnableVertexAttribArray(0);

            glBindVertexArray(0);
        }

        void updateVBO()
        {
            for (size_t i{ 0 }; i < 4; ++i)
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
            constexpr int m_degreesOfFreedom{ 12 };
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

            const Eigen::MatrixXf A{ Eigen::MatrixXf::Identity(m_degreesOfFreedom, m_degreesOfFreedom) - deltaTime * deltaTime * m_massMatrixInverse * stiffnessMatrix };
            const Eigen::VectorXf b{ velocity + deltaTime * m_massMatrixInverse * force };
            const Eigen::VectorXf vNext{ A.colPivHouseholderQr().solve(b) };

            // Set new values
            Physics::setNewPoints(m_points, vNext, deltaTime);

            // Update VBO
            updateVBO();
        }

        void draw() const
        {
            glBindVertexArray(m_VAO);
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        }

    private:
        GLuint m_VAO, m_VBO;
        std::vector<Physics::MassPoint> m_points{};
        std::vector<Physics::Spring> m_springs{};
        std::vector<GLfloat> m_vertices{};
        Eigen::MatrixXf m_massMatrixInverse{};
        int m_degreesOfFreedom{};
};
