#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <vector>

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
    std::vector<MassPoint> points(4);
    points[0] = { glm::vec3{ -5.0f, 5.0f, -5.0f }, glm::vec3{ 0.0f, }, glm::vec3{ 0.0f }, false }; // Bottom left
    points[1] = { glm::vec3{ 5.0f, 5.0f, -5.0f }, glm::vec3{ 0.0f, }, glm::vec3{ 0.0f }, false }; // Bottom right
    points[2] = { glm::vec3{ -5.0f, 5.0f, 5.0f }, glm::vec3{ 0.0f, }, glm::vec3{ 0.0f }, true }; // Top left
    points[3] = { glm::vec3{ 5.0f, 5.0f, 5.0f }, glm::vec3{ 0.0f, }, glm::vec3{ 0.0f }, true }; // Top right
    GLuint planeVAO, planeVBO;
    {
        std::vector<GLfloat> vertices;
        for (const MassPoint& point : points)
        {
            vertices.push_back(point.position.x);
            vertices.push_back(point.position.y);
            vertices.push_back(point.position.z);
            vertices.push_back(0.0f);
            vertices.push_back(0.0f);
            vertices.push_back(1.0f);
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
	GLfloat zLightRotateAmount{ 0.0f };
	GLfloat yLightRotateAmount{ 45.0f };

    // Set uniform variables in shaders that won't change
    glUseProgram(mainShader);
    glUniformMatrix4fv(glGetUniformLocation(mainShader, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    GLfloat lastFrameTime{ static_cast<GLfloat>(glfwGetTime()) };
    while (!glfwWindowShouldClose(window)) 
    {
        const GLfloat currentTime{ static_cast<GLfloat>(glfwGetTime()) };
        const GLfloat deltaTime{ currentTime - lastFrameTime };
        lastFrameTime = currentTime;
        
        //Physics
        constexpr glm::vec3 gravity{ 0.0f, -10.f, 0.0f };
        constexpr float stiffness{ 10.0f };
        constexpr float restLen{ 5.0f };
        constexpr float damping{ 0.1f };
        for (size_t i{ 0 }; i < 4; ++i)
        {
            if (points[i].fixed)
                continue;

            points[i].force = gravity;

            std::vector<size_t> connectedNodes;
            if (i == 0) // Bottom left
                connectedNodes = {1, 2};
            if (i == 1) // Bottom right
                connectedNodes = {0, 4};

            for (size_t j : connectedNodes)
            {
                const glm::vec3 difference{ points[i].position - points[j].position };
                const float differenceLen{ glm::length(difference) };
                const glm::vec3 springForce{ -stiffness * (differenceLen - restLen) * glm::normalize(difference) };
                points[i].force += springForce;
            }

            points[i].force += -damping * points[i].velocity;
        }

        // Integrate with explicit Euler
        constexpr float mass{ 1.0f };
        for (size_t i{ 0 }; i < 4; ++i)
        {
            if (points[i].fixed)
                continue;

            const glm::vec3 acceleration{ points[i].force / mass };
            points[i].velocity += acceleration * deltaTime;
            points[i].position += points[i].velocity * deltaTime;
        }

        // Update VBO
        std::vector<GLfloat> vertices;
        for (const MassPoint& point : points)
        {
            vertices.push_back(point.position.x);
            vertices.push_back(point.position.y);
            vertices.push_back(point.position.z);
            vertices.push_back(0.0f);
            vertices.push_back(0.0f);
            vertices.push_back(1.0f);
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

