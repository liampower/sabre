#include "sabre_data.h"

extern const char* const RaycasterComputeKernel = R"GLSL(
#version 450 core

layout (local_size_x = 2, local_size_y = 2) in;

void main()
{
}
)GLSL";


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

uniform sampler2D RenderedTexture;

in vec2 UV;

void main()
{
    FragCr = texture(RenderedTexture, UV);
}

)GLSL";

