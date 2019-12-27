#include "sabre_data.h"



extern const char* const MainVertexCode = R"GLSL(
#version 450 core

layout (location = 0) in vec2 Position;

out vec2 UV;

void main()
{
    gl_Position = vec4(Position, 0.0, 1.0);
    UV = Position.xy;
}

)GLSL";

extern const char* const MainFragmentCode = R"GLSL(
#version 450 core

out vec4 FragCr;

uniform sampler2D OutputTextureUniform;

in vec2 UV;

void main()
{
    FragCr = texture(OutputTextureUniform, UV);
}

)GLSL";


extern const char* const RaycasterComputeKernel = R"GLSL(
#version 450 core

layout (local_size_x = 2, local_size_y = 2) in;

layout (rgba32f, binding = 0) uniform image2D OutputImgUniform;

uniform uint MaxDepthUniform;

void main()
{
    ivec2 PixelCoords = ivec2(gl_GlobalInvocationID.xy);
    imageStore(OutputImgUniform, PixelCoords, vec4(1.0, 0.0, 0.0, 1.0));
}
)GLSL";

