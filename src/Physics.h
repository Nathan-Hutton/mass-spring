#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <Eigen/Dense>
#include <vector>

namespace Physics
{
    struct MassPoint
    {
        glm::vec3 position;
        glm::vec3 velocity;
        glm::vec3 force;
        bool fixed;
    };

    struct Spring
    {
        size_t i, j;
        float restLength;
        float stiffness;
    };

    void getForceFromGravity(const std::vector<MassPoint>& points, Eigen::VectorXf& force)
    {
        constexpr float damping{ 0.4f };
        constexpr glm::vec3 gravity{ 0.0f, -9.81f, 0.0f };
        for (size_t i{ 0 }; i < points.size(); ++i)
        {
            if (points[i].fixed)
                continue;

            const glm::vec3 forceFromGravity{ gravity - damping * points[i].velocity };
            force.segment<3>(3 * i) = Eigen::Vector3f(forceFromGravity.x, forceFromGravity.y, forceFromGravity.z);
        }
    }

    void handleSpringForces(const std::vector<Spring>& springs, const std::vector<MassPoint>& points, Eigen::VectorXf& force, Eigen::MatrixXf& stiffnessMatrix)
    {
        for (const Physics::Spring& spring : springs)
        {
            const glm::vec3 diff{ points[spring.i].position - points[spring.j].position };
            const Eigen::Vector3f d(diff.x, diff.y, diff.z);
            const float len{ d.norm() };
            if (len < 1e-5f)
                continue;

            const Eigen::Vector3f dir{ d.normalized() };
            const float stretch{ len - spring.restLength };
            const Eigen::Vector3f springForce{ -spring.stiffness * stretch * dir };

            if (!points[spring.i].fixed)
                force.segment<3>(3 * spring.i) += springForce;
            if (!points[spring.j].fixed)
                force.segment<3>(3 * spring.j) -= springForce;

            const glm::vec3 v_ij{ points[spring.i].velocity - points[spring.j].velocity };
            constexpr float dampingCoef{ 0.4f };
            const float dampingForce{ dampingCoef * glm::dot(v_ij, glm::normalize(diff)) };
            const Eigen::Vector3f dampingVec{ dampingForce * dir };

            if (!points[spring.i].fixed)
                force.segment<3>(3 * spring.i) -= dampingVec;
            if (!points[spring.j].fixed)
                force.segment<3>(3 * spring.j) += dampingVec;

            const Eigen::Matrix3f contribution{ spring.stiffness * (Eigen::Matrix3f::Identity() - (d * d.transpose()) / (len * len)) };

            const size_t row{ spring.i * 3 };
            const size_t col{ spring.j * 3 };
            stiffnessMatrix.block<3,3>(row, row) += contribution;
            stiffnessMatrix.block<3,3>(col, col) += contribution;
            stiffnessMatrix.block<3,3>(row, col) -= contribution;
            stiffnessMatrix.block<3,3>(col, row) -= contribution;
        }
    }

    void setNewPoints(std::vector<MassPoint>& points, const Eigen::VectorXf& vNext, float deltaTime)
    {
        for (size_t i{ 0 }; i < points.size(); ++i)
        {
            if (points[i].fixed)
                continue;

            points[i].velocity.x = vNext(3 * i);
            points[i].velocity.y = vNext(3 * i + 1);
            points[i].velocity.z = vNext(3 * i + 2);
            points[i].velocity *= 0.98f;
            const float speed{ glm::length(points[i].velocity) };
            if (speed > 1.0f)
                points[i].velocity *= 1.0f / speed;

            points[i].position += points[i].velocity * deltaTime;
            if (points[i].position.y < -8.0f)
                points[i].position.y = -8.0f;
        }
    }
}
