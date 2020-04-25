#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <vector>
#include <string>

#include <glad/glad.h>
#include <glfw/glfw3.h>

#include "sabre.h"
#include "sabre_math.h"
#include "sabre_svo.h"
#include "sabre_data.h"
#include "sabre_render.h"

static constexpr u32 DEMO_MAX_TREE_DEPTH = 10;
static constexpr u32 DEMO_SCALE_EXPONENT = 5;

static constexpr u32 DisplayWidth = 1280;
static constexpr u32 DisplayHeight = 720;
static constexpr const char* const DisplayTitle = "Sabre";

// NOTE(Liam): Forces use of nVidia GPU on hybrid graphics systems.
extern "C" {
    _declspec(dllexport) int NvOptimusEnablement = 0x00000001;
}

struct camera
{
    f32  Velocity;

    sbrv3 Up;
    sbrv3 Right;
    sbrv3 Forward;
    sbrv3 Position;
};


static void
HandleOpenGLError(GLenum Src, GLenum Type, GLenum ID, GLenum Severity, GLsizei Length, const GLchar* Msg, const void*)
{
    if (GL_DEBUG_TYPE_ERROR == Type)
    {
        fprintf(stderr, "[OpenGL Error] %s\n", Msg);
    }
}

static inline f32
Squared(f32 X)
{
    return X*X;
}


static inline void
OutputGraphicsDeviceInfo(void)
{
    printf("Graphics Vendor: %s\n", glGetString(GL_VENDOR));
    printf("Graphics Renderer: %s\n", glGetString(GL_RENDERER));
}

// NOTE(Liam): Warning! Does not work if Min,Max are not the **actual** (dimension-wise) min and
// max corners.
//
// Tree parameter ignored here.
static bool
CubeSphereIntersection(sbrv3 Min, sbrv3 Max, const sbr_svo* const, const void* const)
{
    const sbrv3 S = sbrv3(16);
    const f32 R = 8;

    f32 DistanceSqToCube = R * R;

    // STACKOVER
    if (S.X < Min.X) DistanceSqToCube -= Squared(S.X - Min.X);
    else if (S.X > Max.X) DistanceSqToCube -= Squared(S.X - Max.X);

    if (S.Y < Min.Y) DistanceSqToCube -= Squared(S.Y - Min.Y);
    else if (S.Y > Max.Y) DistanceSqToCube -= Squared(S.Y - Max.Y);

    if (S.Z < Min.Z) DistanceSqToCube -= Squared(S.Z - Min.Z);
    else if (S.Z > Max.Z) DistanceSqToCube -= Squared(S.Z - Max.Z);

    return DistanceSqToCube >= 0;
}


static inline sbrv3
SphereNormal(sbrv3 C, const sbr_svo* const, const void* const)
{
    const sbrv3 S = sbrv3(16);

    sbrv3 Normal = Normalize(C - S);

    return Normal;
}

static inline sbrv3
SphereColour(sbrv3 C, const sbr_svo* const, const void* const)
{
    return sbrv3(1, 0, 0);
}


static inline sbr_svo*
CreateCubeSphereTestScene(void)
{
    shape_sampler ShapeSampler = shape_sampler{ nullptr, &CubeSphereIntersection };
    data_sampler NormalSampler = data_sampler{ nullptr, &SphereNormal };
    data_sampler ColourSampler = data_sampler{ nullptr, &SphereColour };

    sbr_svo* WorldSvo = SBR_CreateScene(DEMO_SCALE_EXPONENT,
                                    DEMO_MAX_TREE_DEPTH,
                                    &ShapeSampler,
                                    &NormalSampler,
                                    &ColourSampler);

    SBR_InsertVoxel(WorldSvo, sbrv3(0, 0, 0), 16);
    //InsertVoxel(WorldSvo, sbrv3(0, 17, 0), 16);
    //InsertVoxel(WorldSvo, sbrv3(20, 20, 20), 16);
    //InsertVoxel(WorldSvo, sbrv3(0, 0, 0), 16);
    SBR_DeleteVoxel(WorldSvo, sbrv3(0, 0, 0));

    return WorldSvo;
}


static inline sbr_svo*
CreateImportedMeshTestScene(const char* const GLBFileName)
{
    return SBR_ImportGLBFile(DEMO_MAX_TREE_DEPTH, GLBFileName);
}

static inline sbr_svo*
CreateLoadedMeshTestScene(const char* const SvoMeshFileName)
{
    FILE* SvoInFile;
    errno_t Result = fopen_s(&SvoInFile, SvoMeshFileName, "rb");

    if (0 == Result)
    {
        sbr_svo* WorldSvo = LoadSvoFromFile(SvoInFile);

        if (nullptr == WorldSvo)
        {
            fprintf(stderr, "Failed to load SVO file\n");
        }

        fclose(SvoInFile);

        return WorldSvo;

    }
    else
    {
        return nullptr;
    }
}

extern "C" void
InsertVoxel2(sbrv3 CastOrigin, sbrv3 Ray, sbr_svo* const Tree);

static void
InsertVoxelAtMousePoint(f64 MouseX, f64 MouseY, const camera& Cam, sbr_svo* const Svo)
{
    // Unproject the MouseX & Y positions into worldspace.
    
    sbrv3 D = sbrv3(f32(512) / 2.0f, f32(512) / 2.0f, 0.0f);
    
    // Origin of the screen plane in world-space
    sbrv3 WorldVOrigin = Cam.Position - sbrv3(256, 256, 512);

    D = WorldVOrigin + D;

    sbrv3 R = Normalize(D - Cam.Position);

    mat3 CameraMatrix = mat3{{
        { Cam.Right.X, Cam.Right.Y, Cam.Right.Z },
        { Cam.Up.X, Cam.Up.Y, Cam.Up.Z },
        { -Cam.Forward.X, -Cam.Forward.Y, -Cam.Forward.Z },
    }};
    /*mat3 CameraMatrix = mat3{{
        { Cam.Right.Z, Cam.Up.X, -Cam.Forward.Z },
        { Cam.Right.Y, Cam.Up.Y, -Cam.Forward.Y },
        { Cam.Right.X, Cam.Up.Z, -Cam.Forward.X },
    }};*/

    R = R * CameraMatrix;

    printf("D: "); DEBUGPrintVec3(D); printf("\n");
    printf("R: "); DEBUGPrintVec3(R); printf("\n");

    InsertVoxel2(Cam.Position, R, Svo);
}

extern "C" void
DeleteVoxel2(sbrv3 CastOrigin, sbrv3 Ray, sbr_svo* const Tree);


static void
DeleteVoxelAtMousePoint(f64 MouseX, f64 MouseY, const camera& Cam, sbr_svo* const Svo)
{
    // Unproject the MouseX & Y positions into worldspace.
    
    //sbrv3 DeleteP = (Cam.Position - sbrv3(256, 256, 512)) + sbrv3((f32)MouseX, (f32)MouseY, 0);
    sbrv3 D = sbrv3(f32(512) / 2.0f, f32(512) / 2.0f, 0.0f);
    
    // Origin of the screen plane in world-space
    sbrv3 WorldVOrigin = Cam.Position - sbrv3(256, 256, 512);

    D = WorldVOrigin + D;
    //D.X *= 1280.0/720.0;

    sbrv3 R = Normalize(D - Cam.Position);

    mat3 CameraMatrix = mat3{{
        { Cam.Right.X, Cam.Right.Y, Cam.Right.Z },
        { Cam.Up.X, Cam.Up.Y, Cam.Up.Z },
        { -Cam.Forward.X, -Cam.Forward.Y, -Cam.Forward.Z },
    }};
    /*mat3 CameraMatrix = mat3{{
        { Cam.Right.Z, Cam.Up.X, -Cam.Forward.Z },
        { Cam.Right.Y, Cam.Up.Y, -Cam.Forward.Y },
        { Cam.Right.X, Cam.Up.Z, -Cam.Forward.X },
    }};*/

    R = R * CameraMatrix;

    printf("D: "); DEBUGPrintVec3(D); printf("\n");
    printf("R: "); DEBUGPrintVec3(R); printf("\n");

    DeleteVoxel2(Cam.Position, R, Svo);
}



extern int
main(int ArgCount, const char** const Args)
{
    if (GLFW_FALSE == glfwInit())
    {
        fprintf(stderr, "Failed to initialise GLFW\n");
        MessageBox(NULL, "Failed to initialise GLFW\n", "Error", MB_ICONWARNING);

        return EXIT_FAILURE;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    
    GLFWwindow* Window = glfwCreateWindow(DisplayWidth, DisplayHeight, DisplayTitle, nullptr, nullptr);
    glfwMakeContextCurrent(Window);
    glfwSetWindowPos(Window, 100, 100);
    glfwSwapInterval(1);

    if (0 == gladLoadGL())
    {
        fprintf(stderr, "Failed to initialise GLAD\n");
        MessageBox(NULL, "Failed to initialise OpenGL context, make sure you are running this application with up-to-date graphics drivers", "Error", MB_ICONWARNING);
        glfwTerminate();
        return EXIT_FAILURE;
    }

    if (GLFW_FALSE == glfwExtensionSupported("GL_ARB_sparse_textur"))
    {
        fprintf(stderr, "Failed to initialise application\n");
        MessageBox(NULL, "The application cannot start because your system does not support OpenGL sparse textures (ensure you are running this application with up-to-date graphics drivers)", "Error", MB_ICONWARNING);
        glfwTerminate();
        return EXIT_FAILURE;
    }

    OutputGraphicsDeviceInfo();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(Window, true);
    ImGui_ImplOpenGL3_Init("#version 430 core");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_DEPTH_CLAMP);
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(HandleOpenGLError, nullptr);

    int FramebufferWidth;
    int FramebufferHeight;
    glfwGetFramebufferSize(Window, &FramebufferWidth, &FramebufferHeight);

    glViewport(0, 0, FramebufferWidth, FramebufferHeight);
    glfwSetInputMode(Window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

#if 1
    sbr_svo* WorldSvo = CreateImportedMeshTestScene("data/Showcase/serapis.glb");
#else
    svo* WorldSvo = CreateCubeSphereTestScene();
#endif
    if (nullptr == WorldSvo)
    {
        fprintf(stderr, "Failed to load World SVO\n");
        MessageBox(NULL, "Failed to create world scene\n", "Error", MB_ICONWARNING);
        glfwTerminate();
        
        return EXIT_FAILURE;
    }

    // Initialise the render data
    sbr_view_data ViewData = { };
    ViewData.ScreenWidth = 512;
    ViewData.ScreenHeight = 512;

    sbr_render_data* RenderData = SBR_CreateRenderData(WorldSvo, &ViewData);

#if 0
    FILE* File = fopen("logs/cs.nvasm", "wb");
    if (File) DEBUGOutputRenderShaderAssembly(RenderData, File);
    fclose(File);
#endif

    if (nullptr == RenderData)
    {
        fprintf(stderr, "Failed to initialise render data\n");
        MessageBox(NULL, "Failed to initialise render data\n", "Error", MB_ICONWARNING);

        SBR_DeleteScene(WorldSvo);
        glfwTerminate();
        return EXIT_FAILURE;
    }

    camera Cam = { };
    Cam.Forward = sbrv3(0, 0, -1);
    Cam.Right = sbrv3(1, 0, 0);
    Cam.Up = sbrv3(0, 1, 0);
    Cam.Position = sbrv3(4, 4, 96);
    Cam.Velocity = 1.32f;

    const sbrv3 WorldYAxis = sbrv3(0, 1, 0);

    f64 LastMouseX, LastMouseY;
    glfwGetCursorPos(Window, &LastMouseX, &LastMouseY);

    // NOTE: Camera yaw and pitch
    f32 Yaw, Pitch;

    f64 DeltaTime = 0.0;
    f64 FrameStartTime = 0.0;
    f64 FrameEndTime = 0.0;

    f64 LastMouseLTime = 0.0;
    f64 LastMouseRTime = 0.0;

    /* int k = 0x7fffffff;
      k += ArgCount;*/
    while (GLFW_FALSE == glfwWindowShouldClose(Window))
    {
        FrameStartTime = glfwGetTime();
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        if (ImGui::BeginMainMenuBar())
        {
            ImGui::Text("%fms CPU  %d BLKS  %d LVLS", 1000.0*DeltaTime, GetSvoUsedBlockCount(WorldSvo), GetSvoDepth(WorldSvo));
            ImGui::EndMainMenuBar();
        }

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

        if (glfwGetKey(Window, GLFW_KEY_Y))
        {
            printf("Right: "); DEBUGPrintVec3(Cam.Right); printf("\n");
            printf("Up: "); DEBUGPrintVec3(Cam.Up); printf("\n");
            printf("Forward: "); DEBUGPrintVec3(Cam.Forward); printf("\n");
            printf("Position: "); DEBUGPrintVec3(Cam.Position); printf("\n");
        }

        { // NOTE: Mouse look
            Cam.Right = Normalize(Cross(Cam.Forward, WorldYAxis));
            Cam.Up = Normalize(Cross(Cam.Right, Cam.Forward));

            f64 MouseX, MouseY;
            glfwGetCursorPos(Window, &MouseX, &MouseY);

            f64 CurrentTime = glfwGetTime();
            if (GLFW_PRESS == glfwGetMouseButton(Window, GLFW_MOUSE_BUTTON_RIGHT) && ((CurrentTime - LastMouseRTime)) >= 1)
            {
                InsertVoxelAtMousePoint(MouseX, MouseY, Cam, WorldSvo);
                SBR_UpdateRenderData(WorldSvo, RenderData);
                LastMouseRTime = CurrentTime;
            }
            
            if (GLFW_PRESS == glfwGetMouseButton(Window, GLFW_MOUSE_BUTTON_LEFT) && ((CurrentTime - LastMouseLTime)) >= 1)
            {
                DeleteVoxelAtMousePoint(MouseX, MouseY, Cam, WorldSvo);
                SBR_UpdateRenderData(WorldSvo, RenderData);
                LastMouseLTime = CurrentTime;
            }

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

        f32 CameraMatrix[3][3] = {
            { Cam.Right.X, Cam.Right.Y, Cam.Right.Z },
            { Cam.Up.X, Cam.Up.Y, Cam.Up.Z },
            { -Cam.Forward.X, -Cam.Forward.Y, -Cam.Forward.Z },
        };

        f32 F[3] = { Cam.Position.X, Cam.Position.Y, Cam.Position.Z };
        ViewData.CamTransform = (float*)CameraMatrix;
        ViewData.CamPos = F;

        SBR_DrawScene(RenderData, &ViewData);

        ImGui::Render();

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(Window);
        FrameEndTime = glfwGetTime();
        DeltaTime = FrameEndTime - FrameStartTime;
    }

    SBR_DeleteScene(WorldSvo);
    SBR_DeleteRenderData(RenderData);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();

    return EXIT_SUCCESS;
}

