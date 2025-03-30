#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

namespace Physics
{
    struct MassPoint
    {
        glm::vec3 position;
        glm::vec3 velocity;
        const glm::vec3 force;
        const bool fixed;
    };

    struct Spring
    {
        const size_t i, j;
        const float restLength;
        const float stiffness;
        const Eigen::Matrix3f K;
    };

    void getForceFromGravity(const std::vector<MassPoint>& points, Eigen::VectorXf& force)
    {
        const Eigen::Vector3f gravity{ 0.0f, -9.81f, 0.0f };
        for (size_t i{ 0 }; i < points.size(); ++i)
        {
            if (points[i].fixed)
                continue;

            force.block<3, 1>(3 * i, 0) = gravity;
        }
    }

    void getSpringForces(std::vector<Spring>& springs, const std::vector<MassPoint>& points, Eigen::VectorXf& force)
    {
        for (const Spring& spring : springs)
        {
            const glm::vec3 diff = points[spring.i].position - points[spring.j].position;
            const Eigen::Vector3f d(diff.x, diff.y, diff.z);
            const float len = d.norm();
            if (len < 1e-5f) continue;

            const Eigen::Vector3f dir = d / len;
            const float stretch = len - spring.restLength;
            const Eigen::Vector3f f = -spring.stiffness * stretch * dir;

            if (!points[spring.i].fixed)
                force.block<3, 1>(3 * spring.i, 0) += f;
            if (!points[spring.j].fixed)
                force.block<3, 1>(3 * spring.j, 0) -= f;
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
            points[i].velocity *= 0.95f;
            const float speed{ glm::length(points[i].velocity) };
            if (speed > 1.0f)
                points[i].velocity *= 1.0f / speed;

            points[i].position += points[i].velocity * deltaTime;
        }
    }
}
