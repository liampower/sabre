#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>

#define SHADER_COUNT 3

#define ARRAY_COUNT(A) (sizeof(A) / sizeof(A[0]))



struct shader
{
    const char* const FileName;
    const char* const CodeName;
};

static shader Shaders[SHADER_COUNT] = {
    { "code/cs_main.gl", "RaycasterComputeKernel" },
    { "code/vs_main.gl",  "MainVertexCode" },
    { "code/fs_main.gl",  "MainFragmentCode" },
};


static bool
ProcessShader(const shader* Shader, FILE* OutFile)
{
    printf("Processing shader %s %s\n", Shader->CodeName, Shader->FileName);
    FILE* ShaderFile = fopen(Shader->FileName, "rb");

    if (ShaderFile)
    {
        fseek(ShaderFile, 0, SEEK_END);
        long FileSize = ftell(ShaderFile);
        rewind(ShaderFile);

        unsigned char* Data = (unsigned char*)malloc(FileSize + 1);

        fread(Data, FileSize, 1, ShaderFile);
        Data[FileSize] = '\0';

        fprintf(OutFile, "static const char* const %s = R\"GLSL(\n", Shader->CodeName);
        fwrite(Data, FileSize, 1, OutFile);
        fprintf(OutFile, ")GLSL\";\n\n");

        fclose(ShaderFile);

        return true;
    }
    else
    {
        printf("Can't open shader %s\n", Shader->CodeName);

        return false;
    }
}

static void
OutputPrologue(FILE* OutFile)
{
    fprintf(OutFile, "#pragma once\n");
}


extern int
main(int ArgCount, const char** Args)
{
    FILE* OutFile = fopen("code/data.h", "wb+");
    OutputPrologue(OutFile);
    printf("BEGIN\n");

    for (int ShaderIndex = 0; ShaderIndex < 3; ++ShaderIndex)
    {
        if (false == ProcessShader(&Shaders[ShaderIndex], OutFile))
        {
            return EXIT_FAILURE;
        }
    }

    fprintf(OutFile, "\n");

    fclose(OutFile);
}

