#ifndef SABRE_RENDER_H
#define SABRE_RENDER_H


struct render_data;

struct view_data
{
    int ScreenWidth;
    int ScreenHeight;

    // TODO(Liam): Tighten type safety here.
    float* CamTransform;
    float* CamPos;
};

struct shader_data
{
    const char* const MainVertCode;
    const char* const MainFragCode;
    const char* const RenderKernelCode;
    const char* const HasherKernelCode;
};


extern render_data*
CreateRenderData(const svo* Scene,
                 const view_data* ViewData,
                 const shader_data* Shaders);

extern void
UpdateRenderScene(const svo* Scene, render_data* RenderDataOut);

extern void
UpdateRenderShaders(const shader_data* Shaders, render_data* RenderDataOut);

extern u64
DrawScene(const render_data* RenderData, const view_data* ViewData);

extern void
DeleteRenderData(render_data* RenderData);

extern bool
DEBUGOutputRenderShaderAssembly(const render_data* RenderData, FILE* OutFile);

#endif

