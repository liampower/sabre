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
#include "sabre_svo.h"
#include "sabre_data.h"
#include "sabre_render.h"

using namespace vm;

static constexpr u32 DEMO_MAX_TREE_DEPTH = 12;
static constexpr u32 DEMO_SCALE_EXPONENT = 5;

static constexpr u32 DisplayWidth = 1280;
static constexpr u32 DisplayHeight = 720;
static constexpr const char* const DisplayTitle = "Sabre";

// NOTE(Liam): Forces use of nVidia GPU on hybrid graphics systems.
extern "C" {
    __declspec(dllexport) unsigned int NvOptimusEnablement = 0x00000001;
}

struct camera
{
    f32  Velocity;

    vec3 Up;
    vec3 Right;
    vec3 Forward;
    vec3 Position;
};

struct sphere
{
    vec3 Centre;
    float Radius;
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
CubeSphereIntersection(vec3 Min, vec3 Max, const svo* const, const void* const UserData)
{
    const vec3 S = ((const sphere*const)UserData)->Centre;//vec3(16);
    const f32 R = ((const sphere* const)UserData)->Radius;//8;

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


static inline vec3
SphereNormal(vec3 C, const svo* const, const void* const UserData)
{
    const vec3 S = vec3(16);
    const vec3 SphereCentre = ((const sphere* const)UserData)->Centre;

    return Normalize(C - S);
}

static inline vec3
SphereColour(vec3 C, const svo* const, const void* const)
{
    return vec3(1, 0, 1);
}


static inline svo*
CreateCubeSphereTestScene(u32 Lod)
{
    sphere Sphere = { vec3{16.0f, 16.0f, 16.0f}, 8.0f };

    shape_sampler ShapeSampler = shape_sampler{ &Sphere, &CubeSphereIntersection };
    data_sampler NormalSampler = data_sampler{ &Sphere, &SphereNormal };
    data_sampler ColourSampler = data_sampler{ &Sphere, &SphereColour };

    svo* WorldSvo = CreateScene(DEMO_SCALE_EXPONENT,
                                Lod,
                                &ShapeSampler,
                                &NormalSampler,
                                &ColourSampler);

    return WorldSvo;
}


static vec3
UnprojectViewDirection(const camera& Cam)
{
    // Unproject the MouseX & Y positions into worldspace.
    vec3 D = vec3(f32(512) / 2.0f, f32(512) / 2.0f, 0.0f);
    
    // Origin of the screen plane in world-space
    vec3 WorldVOrigin = Cam.Position - vec3(256, 256, 512);
    D = WorldVOrigin + D;

    vec3 R = Normalize(D - Cam.Position);

    mat3x3 CameraMatrix = mat3x3{{
        { Cam.Right.X, Cam.Right.Y, Cam.Right.Z },
        { Cam.Up.X, Cam.Up.Y, Cam.Up.Z },
        { -Cam.Forward.X, -Cam.Forward.Y, -Cam.Forward.Z },
    }};

    R = R * CameraMatrix;

    return R;
}

static void
InsertVoxelAtMousePoint(f64 MouseX, f64 MouseY, const camera& Cam, svo* const Svo)
{
    vec3 R = UnprojectViewDirection(Cam);
    vec3 VoxelPos = GetNearestFreeSlot(Cam.Position, R, Svo);
    DEBUGPrintVec3(VoxelPos);
    InsertVoxel(Svo, VoxelPos);
}



static void
DeleteVoxelAtMousePoint(f64 MouseX, f64 MouseY, const camera& Cam, svo* const Svo)
{
    vec3 R = UnprojectViewDirection(Cam);
    vec3 VoxelPos = GetNearestLeafSlot(Cam.Position, R, Svo);
    DeleteVoxel(Svo, VoxelPos);
}


static void
HandleGLFWError(int, const char* const ErrorMsg)
{
    MessageBox(nullptr, ErrorMsg, "GLFW Error", MB_ICONWARNING);
}


extern int
main(int ArgCount, const char** const Args)
{
    glfwSetErrorCallback(HandleGLFWError);
    if (GLFW_FALSE == glfwInit())
    {
        fprintf(stderr, "Failed to initialise GLFW\n");
        MessageBox(nullptr, "Failed to initialise GLFW\n", "Error", MB_ICONWARNING);

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
        MessageBox(nullptr, "Failed to initialise OpenGL context, make sure you are running this application with up-to-date graphics drivers", "Error", MB_ICONWARNING);
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

    svo* WorldSvo = nullptr;

    // FIXME: BROKEN!! Last two members left uninitialised!
    // Initialise the render data
    view_data ViewData = { };
    ViewData.ScreenWidth = 512;
    ViewData.ScreenHeight = 512;

    render_data* RenderData = nullptr;


    camera Cam;
    Cam.Forward = vec3(0, 0, -1);
    Cam.Right = vec3(1, 0, 0);
    Cam.Up = vec3(0, 1, 0);
    Cam.Position = vec3(4, 4, 96);
    Cam.Velocity = 1.32f;

    const vec3 WorldYAxis = vec3(0, 1, 0);

    f64 LastMouseX, LastMouseY;
    glfwGetCursorPos(Window, &LastMouseX, &LastMouseY);

    // NOTE: Camera yaw and pitch
    f32 Yaw, Pitch;

    f64 DeltaTime = 0.0;
    f64 FrameStartTime = 0.0;
    f64 FrameEndTime = 0.0;

    f64 LastMouseLTime = 0.0;
    f64 LastMouseRTime = 0.0;

    bool ShowMenu = true;
    int Lod = 0;
    u64 GPUTime = 0;
    while (GLFW_FALSE == glfwWindowShouldClose(Window))
    {
        FrameStartTime = glfwGetTime();
        glfwPollEvents();

        glClearColor(0.02f, 0.02f, 0.02f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        if (glfwGetKey(Window, GLFW_KEY_Q))
        {
            glfwSetWindowShouldClose(Window, GLFW_TRUE);
        }

        if (ShowMenu)
        {
            if (! ImGui::Begin("Sabre Viewer Demo"))
            {
                ImGui::End();
            }

            ImGui::SliderInt("Level of Detail", &Lod, 0, DEMO_MAX_TREE_DEPTH);
            ImGui::TextUnformatted("Higher levels will take longer to generate");
            ImGui::Separator();

            if (ImGui::Button("Load Sibenik"))
            {
                WorldSvo = ImportGLBFile(SafeIntToU32(Lod), "data/Showcase/sib2.glb");
                ShowMenu = false;
            }
            else if (ImGui::Button("Load Test Cube"))
            {
                WorldSvo = ImportGLBFile(SafeIntToU32(Lod), "data/Showcase/tex_cube.glb");
                ShowMenu = false;
            }
            else if (ImGui::Button("Load Fireplace Room"))
            {
                WorldSvo = ImportGLBFile(SafeIntToU32(Lod), "data/Showcase/fireplace_room.glb");
                ShowMenu = false;
            }
            else if (ImGui::Button("Load Gallery"))
            {
                WorldSvo = ImportGLBFile(SafeIntToU32(Lod), "data/Showcase/gallery.glb");
                ShowMenu = false;
            }
            else if (ImGui::Button("Load Dragon"))
            {
                WorldSvo = ImportGLBFile(SafeIntToU32(Lod), "data/Showcase/dragon.glb");
                ShowMenu = false;
            }
            else if (ImGui::Button("Load Bunny"))
            {
                WorldSvo = ImportGLBFile(SafeIntToU32(Lod), "data/Showcase/bunny.glb");
                ShowMenu = false;
            }
            else if (ImGui::Button("Load Buddha"))
            {
                WorldSvo = ImportGLBFile(SafeIntToU32(Lod), "data/Showcase/buddha.glb");
                ShowMenu = false;
            }
            else if (ImGui::Button("Load Serapis"))
            {
                WorldSvo = ImportGLBFile(SafeIntToU32(Lod), "data/Showcase/serapis.glb");
                ShowMenu = false;
            }
            else if (ImGui::Button("Load Indonesian"))
            {
                WorldSvo = ImportGLBFile(SafeIntToU32(Lod), "data/Showcase/indonesian.glb");
                ShowMenu = false;
            }
            else if (ImGui::Button("Load Sphere"))
            {
                WorldSvo = CreateCubeSphereTestScene(SafeIntToU32(Lod));
                ShowMenu = false;
            }

            if (false == ShowMenu)
            {
                RenderData = CreateRenderData(WorldSvo, &ViewData);
                glfwSetInputMode(Window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
                if (nullptr == RenderData)
                {
                    fprintf(stderr, "Failed to initialise render data\n");
                    MessageBox(nullptr, "Failed to initialise render data\n", "Error", MB_ICONWARNING);

                    DeleteScene(WorldSvo);
                    glfwTerminate();
                    return EXIT_FAILURE;
                }
            }

            ImGui::End();
        }
        else
        {
            if (nullptr == WorldSvo)
            {
                fprintf(stderr, "Failed to initialise SVO data\n");
                MessageBox(nullptr, "Failed to initialise SVO data\n", "Error", MB_ICONWARNING);

                DeleteScene(WorldSvo);
                glfwTerminate();
                return EXIT_FAILURE;
            }

            if (WorldSvo)
            {
                if (ImGui::BeginMainMenuBar())
                {
                    ImGui::Text("%fms CPU  %fms GPU  %d BLKS  %d LVLS  %llu DATA", 
                                 1000.0*DeltaTime,
                                 f64(GPUTime) / 1000000.0,
                                 GetSvoUsedBlockCount(WorldSvo),
                                 GetSvoDepth(WorldSvo),
                                 WorldSvo->AttribData.size());
                    ImGui::EndMainMenuBar();
                }

                if (glfwGetKey(Window, GLFW_KEY_Q))
                {
                    glfwSetWindowShouldClose(Window, GLFW_TRUE);
                }

                if (glfwGetKey(Window, GLFW_KEY_W)) Cam.Position += Cam.Forward * Cam.Velocity;
                if (glfwGetKey(Window, GLFW_KEY_S)) Cam.Position -= Cam.Forward * Cam.Velocity;
                if (glfwGetKey(Window, GLFW_KEY_A)) Cam.Position -= Cam.Right * Cam.Velocity;
                if (glfwGetKey(Window, GLFW_KEY_D)) Cam.Position += Cam.Right * Cam.Velocity;
                if (glfwGetKey(Window, GLFW_KEY_SPACE)) Cam.Position += Cam.Up * Cam.Velocity;
                if (glfwGetKey(Window, GLFW_KEY_LEFT_SHIFT)) Cam.Position -= Cam.Up * Cam.Velocity;

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
                        UpdateRenderData(WorldSvo, RenderData);
                        LastMouseRTime = CurrentTime;
                    }
                    
                    if (GLFW_PRESS == glfwGetMouseButton(Window, GLFW_MOUSE_BUTTON_LEFT) && ((CurrentTime - LastMouseLTime)) >= 1)
                    {
                        DeleteVoxelAtMousePoint(MouseX, MouseY, Cam, WorldSvo);
                        UpdateRenderData(WorldSvo, RenderData);
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

                GPUTime = DrawScene(RenderData, &ViewData);
            }


        }

        ImGui::Render();

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(Window);
        FrameEndTime = glfwGetTime();
        DeltaTime = FrameEndTime - FrameStartTime;
    }

    DeleteScene(WorldSvo);
    DeleteRenderData(RenderData);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();

    return EXIT_SUCCESS;
}

