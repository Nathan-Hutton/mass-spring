#pragma once


void processKeyboardInputExit(GLFWwindow* window)
{

    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_CAPS_LOCK) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

void processMouseInputObjectRotation(GLFWwindow* window, GLfloat& yRotateAmountChange, GLfloat& zRotateAmountChange)
{
    static bool leftMouseButtonHeld { false }; // This is here so we don't get some huge jump right when you do a left click
    static double xCursorPos {};
    static double yCursorPos {};

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)
    {
        leftMouseButtonHeld = false;
        yRotateAmountChange = 0.0f;
        zRotateAmountChange = 0.0f;
        return;
    }

    // Pressing ctrl. Adjusting light or plane, not teapot
    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    {
        leftMouseButtonHeld = false;
        yRotateAmountChange = 0.0f;
        zRotateAmountChange = 0.0f;
        return;
    }

    if (!leftMouseButtonHeld)
    {
        glfwGetCursorPos(window, &xCursorPos, &yCursorPos);
        leftMouseButtonHeld = true;
        yRotateAmountChange = 0.0f;
        zRotateAmountChange = 0.0f;
        return;
    }

    double newXPos, newYPos;
    glfwGetCursorPos(window, &newXPos, &newYPos);
    yRotateAmountChange = xCursorPos - newXPos;
    zRotateAmountChange = yCursorPos - newYPos;
    xCursorPos = newXPos;
    yCursorPos = newYPos;
}

void processMouseInputObjectDistance(GLFWwindow* window, GLfloat& cameraDistanceChange)
{
    static double yCursorPos {};
    static bool rightMouseButtonHeld { false };

    // Not pressing right mouse
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE)
	{
        rightMouseButtonHeld = false;
        cameraDistanceChange = 0.0f;
        return;
	}

    if (!rightMouseButtonHeld)
    {
        glfwGetCursorPos(window, nullptr, &yCursorPos);
        rightMouseButtonHeld = true;
        cameraDistanceChange = 0;
        return;
    }

    double newYPos;
    glfwGetCursorPos(window, nullptr, &newYPos);
    cameraDistanceChange = yCursorPos - newYPos;
    yCursorPos = newYPos;
}

void processMouseInputLightControls(GLFWwindow* window, GLfloat& zLightRotateChange, GLfloat& yLightRotateChange)
{
    static bool leftMouseAndCtrlHeld { false };
    static double xCursorPos {};
    static double yCursorPos {};

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE || glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_RELEASE)
	{
        leftMouseAndCtrlHeld = false;
        zLightRotateChange = 0.0f;
        yLightRotateChange = 0.0f;
        return;
	}

    if (!leftMouseAndCtrlHeld)
    {
        glfwGetCursorPos(window, &xCursorPos, &yCursorPos);
        leftMouseAndCtrlHeld = true;
        zLightRotateChange = 0.0f;
        yLightRotateChange = 0.0f;
        return;
    }

    double newXPos, newYPos;
    glfwGetCursorPos(window, &newXPos, &newYPos);
    zLightRotateChange = xCursorPos - newXPos;
    yLightRotateChange = yCursorPos - newYPos;
    xCursorPos = newXPos;
    yCursorPos = newYPos;
}

void resize_window(GLFWwindow* window, int width, int height)
{
    (void)window; // This just gets rid of the unused parameter warning
    glViewport(0, 0, width, height);
}

