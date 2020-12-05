#ifndef SABRE_RENDER_H
#define SABRE_RENDER_H


struct render_data;

enum shader_id
{
    SHADER_MAIN_VS,
    SHADER_MAIN_FS,
    SHADER_RENDER_CS,
    SHADER_HASHER_CS,

    SHADER_ID_COUNT
};

struct view_data
{
    int ScreenWidth;
    int ScreenHeight;

    // TODO(Liam): Tighten type safety here.
    float* CamTransform;
    float* CamPos;
};

#if 0
struct shader_data
{
    const char* const MainVertCode;
    const char* const MainFragCode;
    const char* const RenderKernelCode;
    const char* const HasherKernelCode;
};
#endif

struct shader_data
{
    const char** Code;
};

extern render_data*
CreateRenderData(const svo* Scene,
                 const view_data* ViewData,
                 const shader_data* Shaders);

extern void
UpdateRenderScene(const svo* Scene, render_data* RenderDataOut);

extern bool
UpdateRenderShaders(const svo* Scene, const shader_data* Shaders, u32 Changed, render_data* Out);

extern u64
DrawScene(const render_data* RenderData, const view_data* ViewData);

extern void
DeleteRenderData(render_data* RenderData);

extern bool
DEBUGOutputRenderShaderAssembly(const render_data* RenderData, FILE* OutFile);

#endif

