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


extern render_data*
CreateRenderData(const svo* const Svo, const view_data* const ViewData);


extern void
UpdateRenderData(const svo* const Svo, render_data* RenderDataOut);


extern u64
DrawScene(const render_data* const RenderData,
          const view_data* const ViewData);


extern void
DeleteRenderData(render_data* RenderData);


extern bool
DEBUGOutputRenderShaderAssembly(const render_data* const RenderData,
                                FILE* OutFile);

#endif
