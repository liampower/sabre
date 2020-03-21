#include <glad/glad.h>
#include <assert.h>

#include "sabre.h"
#include "sabre_svo.h"
#include "sabre_render.h"
#include "sabre_data.h"

static constexpr uint SABRE_WORK_SIZE_X = 512;
static constexpr uint SABRE_WORK_SIZE_Y = 512;

typedef GLuint  gl_uint;
typedef GLint   gl_int;
typedef GLsizei gl_isize;

// Holds the contextual data required to render a SVO voxel
// scene with OpenGL.
struct sbr_render_data
{
    gl_uint CanvasShader; // Shader program ID for canvas
    gl_uint CanvasVAO;    // VAO holding canvas vertices
    gl_uint CanvasVBO;    // VBO holding canvas vertices

    gl_uint SvoBuffer;    // Buffer for the SVO voxel data
    gl_uint FarPtrBuffer; // Buffer for the SVO far-ptr data
    gl_uint RenderShader; // Shader ID of the raycaster CS
    gl_uint RenderImage;  // GL texture ID of the render output image

    gl_int ViewMatUniformLocation; // Location of view matrix uniform
    gl_int ViewPosUniformLocation; // Location of view position uniform
};


struct svo_buffers
{
    union { gl_uint SvoBuffer, VAO; };
    union { gl_uint FarPtrBuffer, VBO; };
};

// Vertices of a full-screen quad which we 
// render to using a compute shader.
static const f32 GlobalCanvasVerts[12] = {
    -1.0f, -1.0f,
    -1.0f,  1.0f,
     1.0f,  1.0f,

     1.0f,  1.0f,
     1.0f, -1.0f,
    -1.0f, -1.0f,
};

static svo_buffers
UploadCanvasVertices(void)
{
    svo_buffers Result = { };
    gl_uint VAO, VBO;

    glGenBuffers(1, &VBO);
    glGenVertexArrays(1, &VAO);

    assert(VAO);
    assert(VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, 12*sizeof(f32), GlobalCanvasVerts, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);

    Result.VAO = VAO;
    Result.VBO = VBO;

    return Result;
}


static gl_uint
CompileComputeShader(const char* const ComputeShaderCode)
{
    gl_uint ShaderID = glCreateShader(GL_COMPUTE_SHADER);
    gl_uint ProgramID = glCreateProgram();
    gl_int  Success = 0;

    glShaderSource(ShaderID, 1, &ComputeShaderCode, nullptr);
    glCompileShader(ShaderID);
    glGetShaderiv(ShaderID, GL_COMPILE_STATUS, &Success);

    if (0 == Success)
    {
        char Log[512] = { 0 };
        glGetShaderInfoLog(ShaderID, 512, nullptr, Log);
        fprintf(stdout, "Failed to compile compute shader\nLog is: %s\n", Log);

        return 0;
    }

    glAttachShader(ProgramID, ShaderID);
    glLinkProgram(ProgramID);

    Success = 0;
    glGetProgramiv(ProgramID, GL_LINK_STATUS, &Success);

    if (0 == Success)
    {
        fprintf(stderr, "Failed to link compute shader program\n");
        return 0;
    }

    glDeleteShader(ShaderID);

    return ProgramID;
}

static gl_uint
CompileShader(const char* VertSrc, const char* FragSrc)
{
    gl_uint VertShader = 0;
    gl_uint FragShader = 0;
    gl_uint Program    = 0;
    gl_int  Success    = 0;

    VertShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(VertShader, 1, &VertSrc, nullptr);
    glCompileShader(VertShader);
    glGetShaderiv(VertShader, GL_COMPILE_STATUS, &Success);
    
    if (0 == Success)
    {
        char Log[512] = { 0 };
        glGetShaderInfoLog(VertShader, 512, nullptr, Log);
        fprintf(stderr, "Failed to compile vertex shader\n%s", Log);
        return 0;
    }

    Success = false;
    FragShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(FragShader, 1, &FragSrc, nullptr);
    glCompileShader(FragShader);
    glGetShaderiv(FragShader, GL_COMPILE_STATUS, &Success);

    if (0 == Success)
    {
        char Log[512] = { 0 };
        glGetShaderInfoLog(FragShader, 512, nullptr, Log);
        fprintf(stderr, "Failed to compile fragment shader\n%s", Log);
        return 0;
    }

    Success = false;
    Program = glCreateProgram();
    glAttachShader(Program, VertShader);
    glAttachShader(Program, FragShader);
    glLinkProgram(Program);
    glGetProgramiv(Program, GL_LINK_STATUS, &Success);

    if (0 == Success)
    {
        fprintf(stderr, "Failed to link shader program\n");
        return 0;
    }

    glDeleteShader(VertShader);
    glDeleteShader(FragShader);

    return Program;
}

static inline void
SetRenderUniformData(const svo* const Tree, sbr_render_data* const RenderData)
{
    glUseProgram(RenderData->RenderShader);

    glUniform1i(glGetUniformLocation(RenderData->RenderShader, "OuputImgUniform"), RenderData->RenderImage);
    glUniform1ui(glGetUniformLocation(RenderData->RenderShader, "MaxDepthUniform"), Tree->MaxDepth);
    glUniform1ui(glGetUniformLocation(RenderData->RenderShader, "ScaleExponentUniform"), Tree->ScaleExponent);
    glUniform1ui(glGetUniformLocation(RenderData->RenderShader, "BlockCountUniform"), Tree->UsedBlockCount);
    glUniform1ui(glGetUniformLocation(RenderData->RenderShader, "EntriesPerBlockUniform"), SVO_ENTRIES_PER_BLOCK);
    glUniform1ui(glGetUniformLocation(RenderData->RenderShader, "FarPtrsPerBlockUniform"), SVO_FAR_PTRS_PER_BLOCK);
    glUniform1ui(glGetUniformLocation(RenderData->RenderShader, "BiasUniform"), Tree->Bias);
    glUniform1f(glGetUniformLocation(RenderData->RenderShader, "InvBiasUniform"), Tree->InvBias);
}

static inline gl_uint
CreateRenderImage(int ImgWidth, int ImgHeight)
{
    gl_uint OutputTexture;
    glGenTextures(1, &OutputTexture);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, OutputTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, ImgWidth, ImgHeight, 0, GL_RGBA, GL_FLOAT, nullptr);
    glBindImageTexture(0, OutputTexture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    
    return OutputTexture;
}

static svo_buffers
UploadOctreeBlockData(const svo* const Svo)
{
    svo_buffers RenderData = { };
    gl_uint SvoBuffer, FarPtrBuffer;

    // TODO(Liam): Look into combining these allocations
    glGenBuffers(1, &SvoBuffer);
    glGenBuffers(1, &FarPtrBuffer);

    // FIXME(Liam): BIG cleanup here needed - may need to bite the bullet
    // and just have the nodes be typedef'd into U32s anyway.
    static_assert(sizeof(svo_node) == sizeof(GLuint), "Node size must == GLuint size!");

    if (SvoBuffer && FarPtrBuffer)
    {
        usize SvoBlockDataSize = sizeof(svo_node) * SVO_ENTRIES_PER_BLOCK;
        // TODO(Liam): Waste here on the last block
        usize MaxSvoDataSize = SvoBlockDataSize * Svo->UsedBlockCount;

        usize FarPtrBlockDataSize = sizeof(far_ptr) * SVO_FAR_PTRS_PER_BLOCK;
        usize MaxFarPtrDataSize = FarPtrBlockDataSize * Svo->UsedBlockCount;

        // Allocate space for the far ptr buffer
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, FarPtrBuffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, MaxFarPtrDataSize, nullptr, GL_DYNAMIC_COPY);
        far_ptr* GPUFarPtrBuffer = (far_ptr*)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);

        // Allocate space for the node data buffer
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, SvoBuffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, MaxSvoDataSize, nullptr, GL_DYNAMIC_COPY);
        GLuint* GPUSvoBuffer = (GLuint*)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);

        // Reset SSBO buffer binding
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);


        // TODO(Liam): Don't send all the blocks to the renderer! Use only a subset.
        svo_block* CurrentBlk = Svo->RootBlock;
        usize NextSvoDataOffset = 0;
        usize NextFarPtrDataOffset = 0;

        while (CurrentBlk)
        {
            assert(NextSvoDataOffset + (CurrentBlk->NextFreeSlot*sizeof(svo_node)) <= MaxSvoDataSize);

            memcpy(GPUSvoBuffer + NextSvoDataOffset, CurrentBlk->Entries, CurrentBlk->NextFreeSlot * sizeof(svo_node));
            NextSvoDataOffset += CurrentBlk->NextFreeSlot;

            memcpy(GPUFarPtrBuffer + NextFarPtrDataOffset, CurrentBlk->FarPtrs, SVO_FAR_PTRS_PER_BLOCK * sizeof(far_ptr)); 
            NextFarPtrDataOffset += SVO_FAR_PTRS_PER_BLOCK;//CurrentBlk->NextFarPtrSlot;

            CurrentBlk = CurrentBlk->Next;
        }

        // Unmap both buffers
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, FarPtrBuffer);
        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, SvoBuffer);
        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
        
        // Associate buffers with shader slots
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, SvoBuffer);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, FarPtrBuffer);
    }

    RenderData.SvoBuffer = SvoBuffer;
    RenderData.FarPtrBuffer = FarPtrBuffer;

    return RenderData;
}

extern "C" void
DrawSvoRenderData(const sbr_render_data* const RenderData, const sbr_view_data* const ViewData)
{
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, RenderData->SvoBuffer);
    glUseProgram(RenderData->RenderShader);
    glUniformMatrix3fv(RenderData->ViewMatUniformLocation, 1, GL_TRUE, ViewData->CamTransform);
    glUniform3fv(RenderData->ViewPosUniformLocation, 1, ViewData->CamPos);
    glDispatchCompute(SABRE_WORK_SIZE_X/8, SABRE_WORK_SIZE_Y/8, 1);

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    // Draw the canvas
    glUseProgram(RenderData->CanvasShader);
    glBindVertexArray(RenderData->CanvasVAO);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, RenderData->RenderImage);

    glBindBuffer(GL_ARRAY_BUFFER, RenderData->CanvasVBO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}

extern "C" sbr_render_data*
CreateSvoRenderData(const svo* const Tree, const sbr_view_data* const ViewData)
{
    sbr_render_data* RenderData = (sbr_render_data*) calloc(1, sizeof(sbr_render_data));

    RenderData->RenderShader = CompileComputeShader(RaycasterComputeKernel);
    if (0 == RenderData->RenderShader)
    {
        fprintf(stderr, "Failed to compile compute shader\n");

        DeleteSvoRenderData(RenderData);
        return nullptr;
    }

    RenderData->CanvasShader = CompileShader(MainVertexCode, MainFragmentCode);
    if (0 == RenderData->RenderShader)
    {
        fprintf(stderr, "Failed to compile compute shader\n");

        DeleteSvoRenderData(RenderData);
        return nullptr;
    }

    RenderData->RenderImage = CreateRenderImage(ViewData->ScreenWidth, ViewData->ScreenHeight);
    SetRenderUniformData(Tree, RenderData);

    svo_buffers CanvasBuffers = UploadCanvasVertices();
    if (0 == CanvasBuffers.VAO || 0 == CanvasBuffers.VBO)
    {
        fprintf(stderr, "Failed to upload canvas vertices\n");

        DeleteSvoRenderData(RenderData);
        return nullptr;
    }

    RenderData->CanvasVAO = CanvasBuffers.VAO;
    RenderData->CanvasVBO = CanvasBuffers.VBO;

    svo_buffers SvoBuffers = UploadOctreeBlockData(Tree);
    if (0 == SvoBuffers.SvoBuffer || 0 == SvoBuffers.FarPtrBuffer)
    {
        DeleteSvoRenderData(RenderData);
        return nullptr;
    }

    RenderData->SvoBuffer = SvoBuffers.SvoBuffer;
    RenderData->FarPtrBuffer = SvoBuffers.FarPtrBuffer;

    RenderData->ViewMatUniformLocation = glGetUniformLocation(RenderData->RenderShader, "ViewMatrixUniform");
    RenderData->ViewPosUniformLocation = glGetUniformLocation(RenderData->RenderShader, "ViewPosUniform");

    return RenderData;
}

extern "C" void
DeleteSvoRenderData(sbr_render_data* RenderData)
{
    if (RenderData)
    {
        glUseProgram(0);
        glBindVertexArray(0);

        glDeleteProgram(RenderData->RenderShader);
        glDeleteProgram(RenderData->CanvasShader);

        glDeleteBuffers(1, &RenderData->SvoBuffer);
        glDeleteBuffers(1, &RenderData->FarPtrBuffer);

        glDeleteTextures(1, &RenderData->RenderImage);
        glDeleteVertexArrays(1, &RenderData->CanvasVAO);

        free(RenderData);
    }
}

extern "C" void
UpdateSvoRenderData(const svo* const Svo, sbr_render_data* const RenderDataOut)
{
    usize SvoBlockDataSize = sizeof(svo_node) * SVO_ENTRIES_PER_BLOCK;
    // TODO(Liam): Waste here on the last block
    usize MaxSvoDataSize = SvoBlockDataSize * Svo->UsedBlockCount;

    usize FarPtrBlockDataSize = sizeof(far_ptr) * SVO_FAR_PTRS_PER_BLOCK;
    usize MaxFarPtrDataSize = FarPtrBlockDataSize * Svo->UsedBlockCount;

    // Allocate space for the far ptr buffer
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, RenderDataOut->FarPtrBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, MaxFarPtrDataSize, nullptr, GL_DYNAMIC_COPY);
    far_ptr* GPUFarPtrBuffer = (far_ptr*)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);

    // Allocate space for the node data buffer
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, RenderDataOut->SvoBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, MaxSvoDataSize, nullptr, GL_DYNAMIC_COPY);
    GLuint* GPUSvoBuffer = (GLuint*)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);

    // Reset SSBO buffer binding
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);


    // TODO(Liam): Don't send all the blocks to the renderer! Use only a subset.
    svo_block* CurrentBlk = Svo->RootBlock;
    usize NextSvoDataOffset = 0;
    usize NextFarPtrDataOffset = 0;

    while (CurrentBlk)
    {
        assert(NextSvoDataOffset + (CurrentBlk->NextFreeSlot*sizeof(svo_node)) <= MaxSvoDataSize);

        memcpy(GPUSvoBuffer + NextSvoDataOffset, CurrentBlk->Entries, CurrentBlk->NextFreeSlot * sizeof(svo_node));
        NextSvoDataOffset += CurrentBlk->NextFreeSlot;

        memcpy(GPUFarPtrBuffer + NextFarPtrDataOffset, CurrentBlk->FarPtrs, SVO_FAR_PTRS_PER_BLOCK * sizeof(far_ptr)); 
        NextFarPtrDataOffset += SVO_FAR_PTRS_PER_BLOCK;

        CurrentBlk = CurrentBlk->Next;
    }

    // Unmap both buffers
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, RenderDataOut->FarPtrBuffer);
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, RenderDataOut->SvoBuffer);
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
}
