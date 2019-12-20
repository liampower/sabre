#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <glad/glad.h>
#include <glfw/glfw3.h>

#include "sabre.h"

static constexpr u32 DisplayWidth = 1280;
static constexpr u32 DisplayHeight = 720;
static constexpr const char* const DisplayTitle = "Sabre";

static void
HandleOpenGLError(GLenum Src, GLenum Type, GLenum ID, GLenum Severity, GLsizei Length, const GLchar* Message, const void* Data)
{
    fprintf(stderr, "[OpenGL Error] %s\n", Message);
}

extern int
main(int ArgCount, const char** const Args)
{
    if (GLFW_FALSE == glfwInit())
    {
        fprintf(stderr, "Failed to initialise GLFW\n");

        return EXIT_FAILURE;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // NOTE: 4x MSAA
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    
    GLFWwindow* Window = glfwCreateWindow(DisplayWidth, DisplayHeight, DisplayTitle, nullptr, nullptr);
    glfwMakeContextCurrent(Window);
    glfwSwapInterval(1);

    if (0 == gladLoadGL())
    {
        fprintf(stderr, "Failed to initialise GLAD\n");
        glfwTerminate();
        return EXIT_FAILURE;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_DEPTH_CLAMP);
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(HandleOpenGLError, nullptr);

    int FramebufferWidth;
    int FramebufferHeight;
    glfwGetFramebufferSize(Window, &FramebufferWidth, &FramebufferHeight);

    glViewport(0, 0, FramebufferWidth, FramebufferHeight);
    glfwSetInputMode(Window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    while (GLFW_FALSE == glfwWindowShouldClose(Window))
    {
        glClearColor(1.0f, 0.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (glfwGetKey(Window, GLFW_KEY_Q))
        {
            glfwSetWindowShouldClose(Window, GLFW_TRUE);
        }

        glfwPollEvents();
        glfwSwapBuffers(Window);
    }

    glfwTerminate();

    return EXIT_SUCCESS;
}


