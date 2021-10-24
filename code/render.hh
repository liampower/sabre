#ifndef SABRE_RENDER_H
#define SABRE_RENDER_H


struct render_data;

typedef enum shader_id
{
    SHADER_MAIN_VS,
    SHADER_MAIN_FS,
    SHADER_RENDER_CS,
    SHADER_HASHER_CS,

    SHADER_ID_COUNT
} shader_id;

typedef struct view_data
{
    int ScreenWidth;
    int ScreenHeight;

    // TODO(Liam): Tighten type safety here.
    float* CamTransform;
    float* CamPos;
} view_data;

typedef struct shader_data
{
    const char** Code;
} shader_data;

extern render_data*
CreateRenderData(const svo* Scene,
                 const view_data* ViewData,
                 const shader_data* Shaders);

extern void
UpdateRenderScene(const svo* Scene, render_data* RenderDataOut);

extern bool
UpdateRenderShaders(const svo* Scene,
                    const shader_data* Shaders,
                    u32 ChangeMsk,
                    render_data* Out);

extern u64
DrawScene(const render_data* RenderData, const view_data* ViewData);

extern void
DeleteRenderData(render_data* RenderData);

extern bool
DEBUGOutputRenderShaderAssembly(const render_data* RenderData, FILE* OutFile);

#endif

