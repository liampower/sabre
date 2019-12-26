#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <glad/glad.h>
#include <glfw/glfw3.h>

#include "sabre.h"
#include "sabre_math.h"
#include "sabre_svo.h"
#include "data.h"

typedef GLuint gl_uint;
typedef GLint gl_int;


static constexpr u32 DisplayWidth = 1280;
static constexpr u32 DisplayHeight = 720;
static constexpr const char* const DisplayTitle = "Sabre";


static void
HandleOpenGLError(GLenum Src, GLenum Type, GLenum ID, GLenum Severity, GLsizei Length, const GLchar* Message, const void* Data)
{
    fprintf(stderr, "[OpenGL Error] %s\n", Message);
}


static inline f32
Squared(f32 X)
{
    return X*X;
}

static inline bool
CubeSphereIntersection(vec3 Min, vec3 Max)
{
    const vec3 S = vec3(0, 0, 0);
    const f32 R = 16.0f;

    f32 DistanceSqToCube = R * R;

    printf("MIN (%f, %f, %f), MAX (%f, %f, %f)\n", Min.X, Min.Y, Min.Z, Max.X, Max.Y, Max.Z);

    // STACKOVER
    if (S.X < Min.X) DistanceSqToCube -= Squared(S.X - Min.X);
    else if (S.X > Max.X) DistanceSqToCube -= Squared(S.X - Max.X);
    if (S.Y < Min.Y) DistanceSqToCube -= Squared(S.Y - Min.Y);
    else if (S.Y > Max.Y) DistanceSqToCube -= Squared(S.Y - Max.Y);
    if (S.Z < Min.Z) DistanceSqToCube -= Squared(S.Z - Min.Z);
    else if (S.Z > Max.Z) DistanceSqToCube -= Squared(S.Z - Max.Z);

    if (DistanceSqToCube > 0)
    {
        return true;
    }
    else
    {
        printf("******************* RET FALSE ************************\n");
        return false;
    }
}


static gl_uint
CompileComputeShader(const char* const ComputeShaderCode)
{
    gl_uint ShaderID = glCreateShader(GL_COMPUTE_SHADER);
    gl_uint ProgramID = glCreateProgram();
    gl_int  Success = 0;

    glShaderSource(ShaderID, 1, &ComputeShaderCode, nullptr);
    glCompileShader(ShaderID);
    glGetShaderiv(ShaderID, GL_COMPILE_STATUS, &Success);

    if (0 == Success)
    {
        char Log[512] = { 0 };
        glGetShaderInfoLog(ShaderID, 512, nullptr, Log);
        fprintf(stdout, "Failed to compile compute shader\nLog is: %s\n", Log);

        return 0;
    }

    glAttachShader(ProgramID, ShaderID);
    glLinkProgram(ProgramID);

    Success = 0;
    glGetProgramiv(ProgramID, GL_LINK_STATUS, &Success);

    if (0 == Success)
    {
        fprintf(stderr, "Failed to link compute shader program\n");
        return 0;
    }

    glDeleteShader(ShaderID);

    return ProgramID;
}

static gl_uint
CompileShader(const char* VertSrc, const char* FragSrc)
{
    gl_uint VertShader = 0;
    gl_uint FragShader = 0;
    gl_uint Program    = 0;
    gl_int  Success    = 0;

    VertShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(VertShader, 1, &VertSrc, nullptr);
    glCompileShader(VertShader);
    glGetShaderiv(VertShader, GL_COMPILE_STATUS, &Success);
    
    if (0 == Success)
    {
        char Log[512] = { 0 };
        glGetShaderInfoLog(VertShader, 512, nullptr, Log);
        fprintf(stderr, "Failed to compile vertex shader\n%s", Log);
        return 0;
    }

    Success = false;
    FragShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(FragShader, 1, &FragSrc, nullptr);
    glCompileShader(FragShader);
    glGetShaderiv(FragShader, GL_COMPILE_STATUS, &Success);

    if (0 == Success)
    {
        char Log[512] = { 0 };
        glGetShaderInfoLog(FragShader, 512, nullptr, Log);
        fprintf(stderr, "Failed to compile fragment shader\n%s", Log);
        return 0;
    }

    Success = false;
    Program = glCreateProgram();
    glAttachShader(Program, VertShader);
    glAttachShader(Program, FragShader);
    glLinkProgram(Program);
    glGetProgramiv(Program, GL_LINK_STATUS, &Success);

    if (0 == Success)
    {
        fprintf(stderr, "Failed to link shader program\n");
        return 0;
    }

    glDeleteShader(VertShader);
    glDeleteShader(FragShader);

    return Program;
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

    gl_uint ComputeShader = CompileComputeShader(RaycasterComputeKernel);
    gl_uint MainShader = CompileShader(MainVertexCode, MainFragmentCode);

    if (0 == MainShader)
    {
        fprintf(stderr, "Failed to compile shader\n");

        glfwTerminate();
        return EXIT_FAILURE;
    }

    if (0 == ComputeShader)
    {
        fprintf(stderr, "Failed to compile compute shader\n");

        glfwTerminate();
        return EXIT_FAILURE;
    }

    svo* WorldSvo = BuildSparseVoxelOctree(4, &CubeSphereIntersection);

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

    glUseProgram(0);

    glDeleteProgram(MainShader);
    glDeleteProgram(ComputeShader);

    DeleteSparseVoxelOctree(WorldSvo);
    glfwTerminate();

    return EXIT_SUCCESS;
}

