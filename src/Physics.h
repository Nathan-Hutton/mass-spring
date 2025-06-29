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
struct MassPoint {
    glm::vec3 position;
    glm::vec3 velocity;
    bool fixed;

    MassPoint() = default;
    MassPoint(const glm::vec3& pos, const glm::vec3& vel, bool isFixed)
        : position(pos), velocity(vel), fixed(isFixed) {}

    MassPoint(const MassPoint&) = default;
    MassPoint(MassPoint&&) noexcept = default;
    MassPoint& operator=(const MassPoint&) = default;
    MassPoint& operator=(MassPoint&&) noexcept = default;
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
        for (size_t i{ 0 }; i < points.size(); ++i)
        {
            if (points[i].fixed)
                continue;

            if (points[i].position.y < -10.0f)
                force[3 * i + 1] += 200;
            else
                force[3 * i + 1] -= 9.81f;
        }
    }

    // Apply a force to the first vertex in a selected triangle
    void getForceFromSelectedTriangle(MassPoint& point, size_t pointIndex, Eigen::VectorXf& force, const glm::vec3& externalForce)
    {
        //point.fixed = true;
        //return;
        if (point.fixed)
            return;

        force[3 * pointIndex] += externalForce.x;
        force[3 * pointIndex + 1] += externalForce.y;
        force[3 * pointIndex + 2] += externalForce.z;
    }

    void getSpringForces(std::vector<Spring>& springs, const std::vector<MassPoint>& points, Eigen::VectorXf& force)
    {
        for (const Spring& spring : springs)
        {
            const MassPoint& pi{ points[spring.i] };
            const MassPoint& pj{ points[spring.j] };
            if (pi.fixed && pj.fixed) continue;

            const float dx{ pi.position.x - pj.position.x };
            const float dy{ pi.position.y - pj.position.y };
            const float dz{ pi.position.z - pj.position.z };

            const float len{ std::sqrt(dx * dx + dy * dy + dz * dz) };
            if (len < 1e-5f) continue;

            const float invLen{ 1.0f / len };
            const float stretch{ len - spring.restLength };
            const float kStretch{ -spring.stiffness * stretch * invLen };

            const float fx{ kStretch * dx };
            const float fy{ kStretch * dy };
            const float fz{ kStretch * dz };

            if (!pi.fixed)
            {
                force[3 * spring.i] += fx;
                force[3 * spring.i + 1] += fy;
                force[3 * spring.i + 2] += fz;
            }
            if (!pj.fixed)
            {
                force[3 * spring.j] -= fx;
                force[3 * spring.j + 1] -= fy;
                force[3 * spring.j + 2] -= fz;
            }
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
