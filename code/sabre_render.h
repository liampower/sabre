#ifndef SABRE_RENDER_H
#define SABRE_RENDER_H


struct sbr_render_data;

struct sbr_view_data
{
    int ScreenWidth;
    int ScreenHeight;

    // TODO(Liam): Tighten type safety here.
    float* CamTransform;
    float* CamPos;
};

extern "C" sbr_render_data*
SBR_CreateRenderData(const sbr_svo* const Tree,
                     const sbr_view_data* const ViewData);


extern "C" void
SBR_UpdateRenderData(const sbr_svo* const Tree, sbr_render_data* RenderData);


extern "C" void
SBR_DrawScene(const sbr_render_data* const RenderData, 
              const sbr_view_data* const ViewData);

extern "C" void
SBR_DeleteRenderData(sbr_render_data* RenderData);


extern "C" bool
DEBUGOutputRenderShaderAssembly(const sbr_render_data* const RenderData,
                                FILE* OutFile);

#endif
