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
#include <nanoprofile.h>

#include <glad/glad.h>
#include <glfw/glfw3.h>

#include "sabre.hh"
#include "svo.hh"
#include "render.hh"
#include "noise_gen.hh"

using namespace vm;

static constexpr u32 DEMO_MAX_TREE_DEPTH = 12;
static constexpr u32 DEMO_SCALE_EXPONENT = 5;

// Dimensions of the window the user sees
static constexpr u32 DISPLAY_WIDTH  = 1280;
static constexpr u32 DISPLAY_HEIGHT = 720;

// Dimensions of the rendered scene. If these values differ from their DISPLAY
// counterparts, then the GPU will automatically scale the rendered texture up
// or down to match the window dimensions. It is *these* values, *not* DISPLAY
// width or height, that have the biggest impact on render performance.
static constexpr u32 RENDER_WIDTH  = 1024;
static constexpr u32 RENDER_HEIGHT = 768;

// How many seconds to wait before checking for updated shader files.
static constexpr f64 SHADER_FILE_CHECK_HZ = 0.5;

static constexpr const char* const DISPLAY_TITLE = "Sabre";

struct scene
{
    const char* const Name;
    const char* const Path;
};

struct shader_files
{
    const char* Contents[SHADER_ID_COUNT];
    const char* FileName[SHADER_ID_COUNT];
    FILETIME    LastModified[SHADER_ID_COUNT];
};

static const scene GlobalSceneTable[] = {
    { "Sibenik", "data/Showcase/sib2.glb" },
    { "UV Cube", "data/Showcase/tex_cube.glb" },
    { "Fireplace Room", "data/Showcase/fireplace_room.glb" },
    { "Gallery", "data/Showcase/gallery.glb" },
    { "Dragon", "data/Showcase/dragon.glb" },
    { "Bunny", "data/Showcase/bunny.glb" },
    { "Buddha", "data/Showcase/buddha.glb" },
    { "Serapis", "data/Showcase/serapis.glb" },
    { "Indonesian", "data/Showcase/Indonesian.glb" },
    { "Face", "data/Showcase/face.glb" },
};

static const char* const ShaderFileNames[SHADER_ID_COUNT] = {
    "code/main_vs.glsl",
    "code/main_fs.glsl",
    "code/render_cs.glsl",
    "code/htable_cs.glsl",
};

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
    f32  Radius;
};

static char*
ReadEntireFile(const char* const Path)
{
    FILE* File;
    fopen_s(&File, Path, "rb");
    if (nullptr == File)
    {
        fprintf(stderr, "Failed to open file\n");
        return nullptr;
    }

    std::fseek(File, 0L, SEEK_END);
    usize FileSize = static_cast<usize>(std::ftell(File));
    std::rewind(File);

    void* FileData = static_cast<char*>(std::calloc(1, FileSize + 1));
    if (nullptr == FileData)
    {
        fprintf(stderr, "Failed to alloc file data\n");
        fclose(File);
        return nullptr;
    }

    std::fread(FileData, FileSize, 1, File);
    std::fclose(File);

    char* DataBytes = static_cast<char*>(FileData);
    DataBytes[FileSize] = '\0';

    return DataBytes;
}

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
    const vec3 S = ((const sphere* const)UserData)->Centre;//vec3(16);
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
InsertVoxelAtMousePoint(f64 MouseX, f64 MouseY, const camera& Cam, svo* Svo)
{
    vec3 R = UnprojectViewDirection(Cam);
    vec3 VoxelPos = GetNearestFreeSlot(Cam.Position, R, Svo);
    DEBUGPrintVec3(VoxelPos);
    InsertVoxel(Svo, VoxelPos);
}


static void
DeleteVoxelAtMousePoint(f64 MouseX, f64 MouseY, const camera& Cam, svo* Svo)
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


static u32
ReloadChangedShaders(shader_files* Files)
{
    u32 ChangedMsk = 0x00;
    for (u32 ID = 0; ID < SHADER_ID_COUNT; ++ID)
    {
        WIN32_FILE_ATTRIBUTE_DATA AttrData;
        GetFileAttributesExA(Files->FileName[ID],
                             GetFileExInfoStandard,
                             &AttrData);
        FILETIME LastModified = AttrData.ftLastWriteTime;

        if (0 != CompareFileTime(&LastModified, &Files->LastModified[ID]))
        {
            printf("Got new file for shader id %d\n", ID);
            const char* OldContents = Files->Contents[ID];
            Files->Contents[ID] = ReadEntireFile(ShaderFileNames[ID]);
            assert(Files->Contents[ID]);
            Files->LastModified[ID] = LastModified;

            free((void*)OldContents);

            ChangedMsk |= ID;
        }
    }

    return ChangedMsk;
}


static shader_files*
LoadShaderFiles(const char* const FileNames[SHADER_ID_COUNT])
{
    shader_files* Files = (shader_files*)calloc(1, sizeof(shader_files));
    assert(Files);

    for (u32 ID = 0; ID < SHADER_ID_COUNT; ++ID)
    {
        const char* Contents = ReadEntireFile(FileNames[ID]);
        assert(Contents);

        WIN32_FILE_ATTRIBUTE_DATA AttrData;
        GetFileAttributesExA(Files->FileName[ID],
                             GetFileExInfoStandard,
                             &AttrData);
        FILETIME LastModified = AttrData.ftLastWriteTime;

        Files->Contents[ID] = Contents;
        Files->LastModified[ID] = LastModified;
        Files->FileName[ID] = FileNames[ID];
    }

    return Files;
}

static void
DeleteShaderFiles(shader_files* Files)
{
    for (u32 ShaderID = 0; ShaderID < SHADER_ID_COUNT; ++ShaderID)
    {
        free((void*)Files->Contents[ShaderID]);
    }

    free(Files);
}

static bool
SceneSelectionMenu(svo** WorldSvo, int* LodOut)
{
    bool ShowMenu = true;
    int Lod = *LodOut;

    if (! ImGui::Begin("Sabre Viewer Demo"))
    {
        ImGui::End();
    }

    ImGui::SliderInt("Level of Detail", &Lod, 0, DEMO_MAX_TREE_DEPTH);
    ImGui::TextUnformatted("Higher levels will take longer to generate");
    ImGui::Separator();

    for (usize ScIndex = 0; ScIndex < ArrayCount(GlobalSceneTable); ++ScIndex)
    {
        const scene& Scene = GlobalSceneTable[ScIndex];
        if (ImGui::Button(Scene.Name))
        {
            *WorldSvo = ImportGLBFile(SafeIntToU32(Lod), Scene.Path);
            ShowMenu = false;
        }
    }

    if (ImGui::Button("Noise"))
    {
        *WorldSvo = BuildNoiseSvo(SafeIntToU32(Lod), 5);
        ShowMenu = false;
    }

    if (ImGui::Button("Sphere"))
    {
        *WorldSvo = CreateCubeSphereTestScene(SafeIntToU32(Lod));
        ShowMenu = false;
    }

    ImGui::End();
    *LodOut = Lod;

    return ShowMenu;
}


extern int
main(int ArgCount, const char** const Args)
{
    np_profile Prof;
    np_event* EvtStorage = (np_event*)malloc(1000*sizeof(np_event));
    NP_InitProfile(EvtStorage, 1000, &Prof);

    NP_PushTraceEvent(&Prof, "Init");
    NP_PushTraceEvent(&Prof, "Init GLFW");
    glfwSetErrorCallback(HandleGLFWError);
    if (GLFW_FALSE == glfwInit())
    {
        fprintf(stderr, "Failed to initialise GLFW\n");
        MessageBox(nullptr,
                   "Failed to initialise GLFW\n",
                   "Error",
                   MB_ICONWARNING);

        free(EvtStorage);
        return EXIT_FAILURE;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    
    GLFWwindow* Window = glfwCreateWindow(DISPLAY_WIDTH,
                                          DISPLAY_HEIGHT,
                                          DISPLAY_TITLE,
                                          nullptr,
                                          nullptr);
    printf("Created window\n");
    glfwMakeContextCurrent(Window);
    glfwSetWindowPos(Window, 100, 100);
    glfwSwapInterval(1);
    NP_PopTraceEvent(&Prof);

    NP_PushTraceEvent(&Prof, "Init GLAD");
    if (0 == gladLoadGL())
    {
        fprintf(stderr, "Failed to initialise GLAD\n");
        MessageBox(nullptr,
                   "Failed to initialise OpenGL context, make sure you are running this application with up-to-date graphics drivers",
                   "Error",
                   MB_ICONWARNING);

        free(EvtStorage);
        glfwTerminate();
        return EXIT_FAILURE;
    }
    NP_PopTraceEvent(&Prof);

    OutputGraphicsDeviceInfo();

    NP_PushTraceEvent(&Prof, "Init ImGUI");
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(Window, true);
    ImGui_ImplOpenGL3_Init("#version 430 core");
    NP_PopTraceEvent(&Prof);

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
    ViewData.ScreenWidth = RENDER_WIDTH;
    ViewData.ScreenHeight = RENDER_HEIGHT;

    render_data* RenderData = nullptr;

    camera Cam;
    Cam.Forward = vec3(0, 0, -1);
    Cam.Right = vec3(1, 0, 0);
    Cam.Up = vec3(0, 1, 0);
    Cam.Position = vec3(4, 4, 96);
    Cam.Velocity = 0.15f;

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

    int Lod = 0;

    bool ShowMenu = true;
    u64 GPUTime = 0;
    NP_PopTraceEvent(&Prof); // Init

    shader_files* ShaderFiles = LoadShaderFiles(ShaderFileNames);
    shader_data Shaders{ ShaderFiles->Contents };
    printf("Contents: %s\n", *ShaderFiles->Contents);
    f64 LastShaderCheckTime = 0.0;

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
            ShowMenu = SceneSelectionMenu(&WorldSvo, &Lod);

            if (false == ShowMenu)
            {
                RenderData = CreateRenderData(WorldSvo, &ViewData, &Shaders);
                glfwSetInputMode(Window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
                if (nullptr == RenderData)
                {
                    fprintf(stderr, "Failed to initialise render data\n");
                    MessageBox(nullptr, 
                               "Failed to initialise render data\n",
                               "Error",
                               MB_ICONWARNING);

                    DeleteScene(WorldSvo);
                    free(EvtStorage);
                    glfwTerminate();
                    return EXIT_FAILURE;
                }
            }
        }
        else
        {
            if (nullptr == WorldSvo)
            {
                fprintf(stderr, "Failed to initialise SVO data\n");
                MessageBox(nullptr,
                           "Failed to initialise SVO data\n",
                           "Error",
                           MB_ICONWARNING);

                DeleteScene(WorldSvo);
                free(EvtStorage);
                glfwTerminate();
                return EXIT_FAILURE;
            }

            if (WorldSvo)
            {
                f64 CurrentTime = glfwGetTime();
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

                // Every 0.5 seconds, check if we need to update the shaders
                if ((CurrentTime - LastShaderCheckTime) >= SHADER_FILE_CHECK_HZ)
                {
                    u32 ChMsk = ReloadChangedShaders(ShaderFiles);
                    UpdateRenderShaders(WorldSvo, &Shaders, ChMsk, RenderData);
                    LastShaderCheckTime = CurrentTime;
                }

                if (glfwGetKey(Window, GLFW_KEY_W)) Cam.Position += Cam.Forward * Cam.Velocity;
                if (glfwGetKey(Window, GLFW_KEY_S)) Cam.Position -= Cam.Forward * Cam.Velocity;
                if (glfwGetKey(Window, GLFW_KEY_A)) Cam.Position -= Cam.Right * Cam.Velocity;
                if (glfwGetKey(Window, GLFW_KEY_D)) Cam.Position += Cam.Right * Cam.Velocity;
                if (glfwGetKey(Window, GLFW_KEY_SPACE)) Cam.Position += Cam.Up * Cam.Velocity;
                if (glfwGetKey(Window, GLFW_KEY_LEFT_SHIFT)) Cam.Position -= Cam.Up * Cam.Velocity;
                if (glfwGetKey(Window, GLFW_KEY_V)&& ((CurrentTime - LastMouseRTime)) >= 1) Cam.Velocity += 0.15f;
                if (glfwGetKey(Window, GLFW_KEY_X)&& ((CurrentTime - LastMouseRTime)) >= 1) Cam.Velocity -= 0.15f;

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

                    if (GLFW_PRESS == glfwGetMouseButton(Window, GLFW_MOUSE_BUTTON_RIGHT) && ((CurrentTime - LastMouseRTime)) >= 1)
                    {
                        InsertVoxelAtMousePoint(MouseX, MouseY, Cam, WorldSvo);
                        UpdateRenderScene(WorldSvo, RenderData);
                        LastMouseRTime = CurrentTime;
                    }
                    
                    if (GLFW_PRESS == glfwGetMouseButton(Window, GLFW_MOUSE_BUTTON_LEFT) && ((CurrentTime - LastMouseLTime)) >= 1)
                    {
                        DeleteVoxelAtMousePoint(MouseX, MouseY, Cam, WorldSvo);
                        UpdateRenderScene(WorldSvo, RenderData);
                        LastMouseLTime = CurrentTime;
                    }

                    const f32 DX = static_cast<f32>(MouseX - LastMouseX);
                    const f32 DY = static_cast<f32>(MouseY - LastMouseY);

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
                ViewData.CamTransform = (f32*)CameraMatrix;
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

    printf("Deleting SVO\n");
    DeleteScene(WorldSvo);
    printf("Deleting Render Data\n");
    DeleteRenderData(RenderData);

    printf("Shutting down IMGUI\n");
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    printf("Shutting down GLFW\n");
    glfwTerminate();
    printf("Writing JSON trace\n");
    //NP_WriteJSONTrace(&Prof, nullptr, 0);

    free(EvtStorage);

    DeleteShaderFiles(ShaderFiles);
    return EXIT_SUCCESS;
}

