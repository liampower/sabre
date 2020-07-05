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


extern "C" render_data*
CreateRenderData(const svo* const Svo, const view_data* const ViewData);


extern "C" void
UpdateRenderData(const svo* const Svo, render_data* RenderDataOut);


extern "C" u64
DrawScene(const render_data* const RenderData,
          const view_data* const ViewData);


extern "C" void
DeleteRenderData(render_data* RenderData);


extern "C" bool
DEBUGOutputRenderShaderAssembly(const render_data* const RenderData,
                                FILE* OutFile);

#endif
