#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <Eigen/Dense>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <array>

#include "ShaderHandler.h"
#include "Input.h"
#include "Plane.h"

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

//int main(int argc, char* argv[])
int main()
{
    //if (argc < 2) // Since I want to just be able to ./main
    //{
    //    argv[1] = strdup("../assets/dragon_80k.obj");
    //    //argv[1] = strdup("../assets/armadillo_50k_tet.obj");
    //    //return -1;
    //}

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // These 2 lines are just so I can set the window size to be the size of the monitor in glfwCreateWindow
    GLFWmonitor* monitor { glfwGetPrimaryMonitor() };
    const GLFWvidmode* mode { glfwGetVideoMode(monitor) };
    GLFWwindow* window { glfwCreateWindow(mode->width, mode->height, "Rigid-body", monitor, NULL) };
    if (window == NULL)
    {
        std::cout << "Failed to create window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, resize_window);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL); // So the cursor won't hit the edge of the screen and stop
    if (glfwRawMouseMotionSupported())
        glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);

    // Query and load all OpenGL extensions allowed by your drivers
    // Allows us to access features/extensions not in the core OpenGL specification
    if(glewInit() != GLEW_OK)
    {
        glfwDestroyWindow(window);
        glfwTerminate();
        throw std::runtime_error("Glew initialization failed");
    }
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
    compileShaders();

    // Handle objects
    const CollisionPlane collisionPlane{ 10.0f, -5.0f };
    std::array<MassPoint, 4> points{};
    points[0] = { glm::vec3{ -5.0f, -1.0f, 0.0f }, glm::vec3{ 0.0f }, glm::vec3{ 0.0f }, false }; // Bottom left
    points[1] = { glm::vec3{ 5.0f, -1.0f, 0.0f }, glm::vec3{ 0.0f }, glm::vec3{ 0.0f }, false }; // Bottom right
    points[2] = { glm::vec3{ -5.0f, 9.0f, 0.0f }, glm::vec3{ 0.0f }, glm::vec3{ 0.0f }, true }; // Top left
    points[3] = { glm::vec3{ 5.0f, 9.0f, 0.0f }, glm::vec3{ 0.0f }, glm::vec3{ 0.0f }, true }; // Top right

    constexpr float stiffness{ 100.0f };
    std::array<Spring, 4> springs
    {
        Spring{0, 1, glm::length(points[0].position - points[1].position), stiffness},
        Spring{0, 2, glm::length(points[0].position - points[2].position), stiffness},
        Spring{1, 3, glm::length(points[1].position - points[3].position), stiffness},
        Spring{2, 3, glm::length(points[2].position - points[3].position), stiffness}
    };

    GLuint planeVAO, planeVBO;
    {
        size_t i{ 0 };
        std::array<GLfloat, 24> vertices;
        for (const MassPoint& point : points)
        {
            vertices[i++] = point.position.x;
            vertices[i++] = point.position.y;
            vertices[i++] = point.position.z;
            vertices[i++] = 0.0f;
            vertices[i++] = 0.0f;
            vertices[i++] = 1.0f;
        }
        glGenVertexArrays(1, &planeVAO);

        glBindVertexArray(planeVAO);

        glGenBuffers(1, &planeVBO);

        glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_STATIC_DRAW);

        // Set vertex attributes
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void*)0); // Vertex positions
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void*)(sizeof(GLfloat) * 3)); // Vertex normals
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);
    }

    // ****************
    // Scene properties
    // ****************
    glClearColor(0.0f, 0.1f, 0.1f, 1.0f);

    const glm::mat4 projection { glm::perspective(glm::radians(45.0f), (float)mode->width / mode->height, 0.1f, 500.0f) };
    constexpr glm::vec3 viewDir { 0.0f, 0.0f, 1.0f };

    GLfloat xCameraRotateAmountObject{ 0.0f };
    GLfloat zCameraRotateAmountObject{ 0.0f };
    GLfloat viewDistance{-30.0f};

    // Parameters to change light rotation
	GLfloat zLightRotateAmount{ -45.0f };
	GLfloat yLightRotateAmount{ 30.0f };

    // Set uniform variables in shaders that won't change
    glUseProgram(mainShader);
    glUniformMatrix4fv(glGetUniformLocation(mainShader, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    GLfloat lastFrameTime{ static_cast<GLfloat>(glfwGetTime()) };
    constexpr int degreesOfFreedom{ 12 };
    const Eigen::MatrixXf massMatrixInverse{ Eigen::MatrixXf::Identity(degreesOfFreedom, degreesOfFreedom).inverse() };
    while (!glfwWindowShouldClose(window)) 
    {
        const GLfloat currentTime{ static_cast<GLfloat>(glfwGetTime()) };
        const GLfloat deltaTime{ currentTime - lastFrameTime };
        lastFrameTime = currentTime;
        
        //Physics
        constexpr glm::vec3 gravity{ 0.0f, -9.81f, 0.0f };
        Eigen::VectorXf velocity(degreesOfFreedom);
        Eigen::VectorXf force(degreesOfFreedom);
        Eigen::MatrixXf stiffnessMatrix(degreesOfFreedom, degreesOfFreedom);
        velocity.setZero();
        force.setZero();
        stiffnessMatrix.setZero();

        for (size_t i{ 0 }; i < 4; ++i) {
            velocity(3 * i + 0) = points[i].velocity.x;
            velocity(3 * i + 1) = points[i].velocity.y;
            velocity(3 * i + 2) = points[i].velocity.z;
        }


        // Get forces from gravity
        constexpr float damping{ 0.4f };
        for (size_t i{ 0 }; i < 4; ++i)
        {
            if (points[i].fixed)
                continue;

            const glm::vec3 forceFromGravity{ gravity - damping * points[i].velocity };
            force.segment<3>(3 * i) = Eigen::Vector3f(forceFromGravity.x, forceFromGravity.y, forceFromGravity.z);
        }

        // Handle spring forces
        for (const Spring& spring : springs)
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

            const Eigen::Matrix3f contribution{ spring.stiffness * (Eigen::Matrix3f::Identity() - (d * d.transpose()) / (len * len)) };

            const size_t row{ spring.i * 3 };
            const size_t col{ spring.j * 3 };
            stiffnessMatrix.block<3,3>(row, row) += contribution;
            stiffnessMatrix.block<3,3>(col, col) += contribution;
            stiffnessMatrix.block<3,3>(row, col) -= contribution;
            stiffnessMatrix.block<3,3>(col, row) -= contribution;
        }

        // Integrate with explicit Euler
        const Eigen::MatrixXf A{ Eigen::MatrixXf::Identity(degreesOfFreedom, degreesOfFreedom) - deltaTime * deltaTime * massMatrixInverse * stiffnessMatrix };
        const Eigen::VectorXf b{ velocity + deltaTime * massMatrixInverse * force };
        const Eigen::VectorXf vNext{ A.colPivHouseholderQr().solve(b) };

        for (size_t i{ 0 }; i < 4; ++i)
        {
            if (points[i].fixed)
                continue;

            points[i].velocity.x = vNext(3 * i);
            points[i].velocity.y = vNext(3 * i + 1);
            points[i].velocity.z = vNext(3 * i + 2);
            points[i].position += points[i].velocity * deltaTime;
        }

        // Update VBO
        std::array<GLfloat, 24> vertices;
        for (size_t i{ 0 }; i < 4; ++i)
        {
            const MassPoint& point{ points[i] };
            vertices[i * 6    ] = point.position.x;
            vertices[i * 6 + 1]= point.position.y;
            vertices[i * 6 + 2] = point.position.z;
            vertices[i * 6 + 3] = 0.0f;
            vertices[i * 6 + 4] = 0.0f;
            vertices[i * 6 + 5] = 1.0f;
        }

        glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * vertices.size(), vertices.data());

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Input
        GLfloat xRotateAmountChange;
        GLfloat zRotateAmountChange;
        processMouseInputObjectRotation(window, xRotateAmountChange, zRotateAmountChange);

        GLfloat cameraDistanceChange;
        processMouseInputObjectDistance(window, cameraDistanceChange);

        GLfloat zLightRotateChange;
        GLfloat yLightRotateChange;
        processMouseInputLightControls(window, yLightRotateChange, zLightRotateChange);

        processKeyboardInputExit(window);

        // Apply changes from input
        viewDistance += cameraDistanceChange * 0.05;

        zCameraRotateAmountObject += xRotateAmountChange;
        xCameraRotateAmountObject += zRotateAmountChange;
        zCameraRotateAmountObject = fmod(zCameraRotateAmountObject, 360.0f);
        xCameraRotateAmountObject = fmod(xCameraRotateAmountObject, 360.0f);

        yLightRotateAmount += yLightRotateChange;
        zLightRotateAmount += zLightRotateChange;
		yLightRotateAmount = fmod(yLightRotateAmount, 360.0f);
		zLightRotateAmount = fmod(zLightRotateAmount, 360.0f);

        // Setup transforms
        glm::mat4 view { glm::translate(glm::mat4{1.0f}, viewDir * viewDistance) };
        view = glm::rotate(view, glm::radians(-xCameraRotateAmountObject), glm::vec3{1.0f, 0.0f,0.0f});
        view = glm::rotate(view, glm::radians(-zCameraRotateAmountObject), glm::vec3{0.0f, 1.0f, 0.0f});

		glm::mat4 lightRotateMatrix { glm::rotate(glm::mat4{ 1.0f }, glm::radians(zLightRotateAmount), glm::vec3(0.0f, 0.0f, 1.0f)) };
		lightRotateMatrix = glm::rotate(lightRotateMatrix, glm::radians(yLightRotateAmount), glm::vec3(0.0f, 1.0f, 0.0f));
        const glm::vec3 lightDir { glm::vec3{lightRotateMatrix * glm::vec4{1.0f, 0.0f, 0.0f, 0.0f}} };
        const glm::vec3 lightDirInViewSpace { -glm::normalize(view * glm::vec4(lightDir, 0.0f)) };

        const glm::mat4 model{ 1.0f };
        const glm::mat4 modelViewTransform { view * model };

        // Render object to screen
        glUseProgram(mainShader);
        glUniformMatrix4fv(glGetUniformLocation(mainShader, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(mainShader, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(mainShader, "modelView"), 1, GL_FALSE, glm::value_ptr(modelViewTransform));
        glUniformMatrix4fv(glGetUniformLocation(mainShader, "normalModelView"), 1, GL_FALSE, glm::value_ptr(glm::transpose(glm::inverse(modelViewTransform))));
		glUniform3fv(glGetUniformLocation(mainShader, "lightDir"), 1, glm::value_ptr(lightDirInViewSpace));
        glUniform3fv(glGetUniformLocation(mainShader, "diffuseMaterialColor"), 1, glm::value_ptr(glm::vec3{ 0.0f, 0.5f, 0.0f }));
        collisionPlane.draw();

        // Draw mass-spring plane
        glUniform3fv(glGetUniformLocation(mainShader, "diffuseMaterialColor"), 1, glm::value_ptr(glm::vec3{ 0.0f, 0.0f, 1.0f }));
        glBindVertexArray(planeVAO);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

