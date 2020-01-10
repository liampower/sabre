#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

#include <glad/glad.h>
#include <glfw/glfw3.h>

#include "sabre.h"
#include "sabre_math.h"
#include "sabre_svo.h"
#include "sabre_data.h"

#define SABRE_MAX_TREE_DEPTH 4
#define SABRE_WORK_SIZE_X 512
#define SABRE_WORK_SIZE_Y 512


typedef GLuint gl_uint;
typedef GLint  gl_int;

static constexpr u32 DisplayWidth = 512;
static constexpr u32 DisplayHeight = 512;
static constexpr const char* const DisplayTitle = "Sabre";

static const f32 ScreenQuadVerts[12] = {
    -1.0f, -1.0f,
    -1.0f,  1.0f,
     1.0f,  1.0f,

     1.0f,  1.0f,
     1.0f, -1.0f,
    -1.0f, -1.0f,
};

struct camera
{
    f32    Velocity;

    vec3   Up;
    vec3   Right;
    vec3   Forward;
    vec3   Position;
};


static void
HandleOpenGLError(GLenum Src, GLenum Type, GLenum ID, GLenum Severity, GLsizei Length, const GLchar* Msg, const void*)
{
    if (GL_DEBUG_TYPE_ERROR == Type)
    {
        fprintf(stderr, "[OpenGL Error] %s\n", Msg);
    }
    else
    {
        fprintf(stderr, "[OpenGL Info] %s\n", Msg);
    }
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

    //printf("MIN (%f, %f, %f), MAX (%f, %f, %f)\n", Min.X, Min.Y, Min.Z, Max.X, Max.Y, Max.Z);

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
        //printf("******************* RET FALSE ************************\n");
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


// TODO(Liam): Replace with a more performant design so that we don't have to keep
// calling this for every node!
static gl_uint
PackSvoNodeToGLUint(svo_node* Node)
{
    gl_uint Packed = 0;

    Packed |= ((u32)Node->ChildPtr << 16);
    Packed |= ((u32)Node->OccupiedMask << 8);
    Packed |= ((u32)Node->LeafMask);

    return Packed;
}

static gl_uint
UploadOctreeBlockData(const svo* const Svo)
{
    gl_uint SvoBuffer;
    glGenBuffers(1, &SvoBuffer);

    // FIXME(Liam): BIG cleanup here needed - may need to bite the bullet
    // and just have the nodes be typedef'd into U32s anyway.
    static_assert(sizeof(svo_node) == sizeof(u32), "svo_node needs to be 32 bits!");

    if (SvoBuffer)
    {
        usize BlockDataSize = sizeof(svo_node) * SVO_ENTRIES_PER_BLOCK;
        usize TotalDataSize = BlockDataSize * Svo->UsedBlockCount;

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, SvoBuffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, TotalDataSize, nullptr, GL_DYNAMIC_COPY);

        static_assert(sizeof(svo_node) == sizeof(GLuint), "Node size must == GLuint size!");

        GLuint* GPUTreeBuffer = (GLuint*) glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);

        svo_block* CurrentBlock = Svo->CurrentBlock;
        usize BlockOffset = 0;

        for (u32 BlockIndex = 0; BlockIndex < Svo->UsedBlockCount; ++BlockIndex)
        {
            //memcpy(GPUTreeBuffer + BlockOffset, CurrentBlock->Entries, BlockDataSize);

            // FIXME(Liam): Slow as balls
            for (u32 NodeIndex = 0; NodeIndex < CurrentBlock->NextFreeSlot; ++NodeIndex)
            {
                GPUTreeBuffer[BlockOffset + NodeIndex] = PackSvoNodeToGLUint(&CurrentBlock->Entries[NodeIndex]);
            }

            BlockOffset += CurrentBlock->NextFreeSlot; // (in nodes)
            CurrentBlock = CurrentBlock->Prev;
        }

        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
        
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, SvoBuffer);
    }

    return SvoBuffer;
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
    glfwSetWindowPos(Window, 100, 100);
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

    svo* WorldSvo = BuildSparseVoxelOctree(SABRE_MAX_TREE_DEPTH, &CubeSphereIntersection);
    gl_uint SvoShaderBuffer = UploadOctreeBlockData(WorldSvo);

    if (0 == SvoShaderBuffer)
    {
        fprintf(stderr, "Failed to upload octree block data\n");

        glfwTerminate();
        return EXIT_FAILURE;
    }

    glUseProgram(ComputeShader);

    gl_uint OutputTexture;
    glGenTextures(1, &OutputTexture);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, OutputTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, DisplayWidth, DisplayHeight, 0, GL_RGBA, GL_FLOAT, nullptr);
    glUniform1i(glGetUniformLocation(ComputeShader, "OutputImgUniform"), 0);
    glBindImageTexture(0, OutputTexture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);

    glUniform1ui(glGetUniformLocation(ComputeShader, "MaxDepthUniform"), WorldSvo->MaxDepth);
    glUniform1ui(glGetUniformLocation(ComputeShader, "BlockCountUniform"), WorldSvo->UsedBlockCount);

    gl_int ViewMatrixUniformLocation = glGetUniformLocation(ComputeShader, "ViewMatrixUniform");

    glUseProgram(MainShader);
    glUniform1i(glGetUniformLocation(MainShader, "RenderedTextureUniform"), 0);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, SvoShaderBuffer);


    gl_uint VAO, VBO;
    {
        glGenBuffers(1, &VBO);
        glGenVertexArrays(1, &VAO);

        assert(VAO);
        assert(VBO);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, 12*sizeof(f32), ScreenQuadVerts, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
    }

    camera Cam = { };
    Cam.Forward = vec3(0, 0, -1);
    Cam.Right = vec3(1, 0, 0);
    Cam.Up = vec3(0, 1, 0);
    Cam.Position = vec3(0, 0, -4);
    Cam.Velocity = 2.4f;

    const vec3 WorldYAxis = vec3(0, 1, 0);

    f64 LastMouseX, LastMouseY;
    glfwGetCursorPos(Window, &LastMouseX, &LastMouseY);

    // NOTE: Camera yaw and pitch
    f32 Yaw, Pitch;

    m4x4 CameraTransform;
    while (GLFW_FALSE == glfwWindowShouldClose(Window))
    {
        glClearColor(1.0f, 0.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (glfwGetKey(Window, GLFW_KEY_Q))
        {
            glfwSetWindowShouldClose(Window, GLFW_TRUE);
        }

        if (glfwGetKey(Window, GLFW_KEY_W))
        {
            Cam.Position += Cam.Velocity * Cam.Forward;
        }

        if (glfwGetKey(Window, GLFW_KEY_S))
        {
            Cam.Position -= Cam.Velocity * Cam.Forward;
        }

        if (glfwGetKey(Window, GLFW_KEY_A))
        {
            Cam.Position -= Cam.Velocity * Cam.Right;
        }

        if (glfwGetKey(Window, GLFW_KEY_D))
        {
            Cam.Position += Cam.Velocity * Cam.Right;
        }

        if (glfwGetKey(Window, GLFW_KEY_SPACE))
        {
            Cam.Position += Cam.Velocity * Cam.Up;
        }

        if (glfwGetKey(Window, GLFW_KEY_LEFT_SHIFT))
        {
            Cam.Position -= Cam.Velocity * Cam.Up;
        }
#if 0

        { // NOTE: Mouse look
            Cam.Right = Normalize(Cross(Cam.Forward, WorldYAxis));
            Cam.Up = Normalize(Cross(Cam.Right, Cam.Forward));

            f64 MouseX, MouseY;
            glfwGetCursorPos(Window, &MouseX, &MouseY);

            const f32 DX = (f32)(MouseX - LastMouseX);
            const f32 DY = (f32)(MouseY - LastMouseY);

            LastMouseX = MouseX;
            LastMouseY = MouseY;

            Yaw = Rads(DX) * -0.05f;
            Pitch = Rads(DY) * -0.05f;

            quat YawRotation = RotationQuaternion(Yaw, WorldYAxis);
            quat PitchRotation = RotationQuaternion(Pitch, Cam.Right);

            Cam.Forward = Normalize(Rotate((YawRotation * PitchRotation), Cam.Forward));
        }

        m4x4 T = TranslationMatrix(-Cam.Position);
        m4x4 R = {{
            { Cam.Right.X, Cam.Right.Y, Cam.Right.Z, 0.0f },
            { Cam.Up.X,    Cam.Up.Y,    Cam.Up.Z,    0.0f },
            { -Cam.Forward.X, -Cam.Forward.Y, -Cam.Forward.Z, 0.0f },
            { 0.0f, 0.0f, 0.0f, 1.0f }
        }};

        CameraTransform =  R * T;
#endif

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, SvoShaderBuffer);
        glUseProgram(ComputeShader);
        //glUniformMatrix4fv(ViewMatrixUniformLocation, 1, GL_TRUE, *CameraTransform.M);
        f32 F[3] = { Cam.Position.X, Cam.Position.Y, Cam.Position.Z };
        glUniform3fv(glGetUniformLocation(ComputeShader, "ViewPosUniform"), 1, F);
        glDispatchCompute(SABRE_WORK_SIZE_X, SABRE_WORK_SIZE_Y, 1);

        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

        glUseProgram(MainShader);
        glBindVertexArray(VAO);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, OutputTexture);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        glfwPollEvents();
        glfwSwapBuffers(Window);
    }

    glUseProgram(0);

    glDeleteProgram(MainShader);
    glDeleteProgram(ComputeShader);

    glDeleteBuffers(1, &SvoShaderBuffer);

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);

    DeleteSparseVoxelOctree(WorldSvo);
    glfwTerminate();

    return EXIT_SUCCESS;
}

