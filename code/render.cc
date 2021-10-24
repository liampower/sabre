#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <cstring>

#include <glad/glad.h>

#include "sabre.hh"
#include "svo.hh"
#include "render.hh"
#include "vecmath.hh"

using gl_uint = GLuint;
using gl_int = GLint;
using gl_sizei = GLsizei;
using gl_enum = GLenum;
using gl_u64 = GLuint64;

using namespace vm;

// Fixed-size for shader error log buffers.
static constexpr usize SHADER_LOG_BUFFER_SIZE = 1024U;

// Dimensions of the render kernel work groups.
static constexpr uint WORK_SIZE_X = 1024U;
static constexpr uint WORK_SIZE_Y = 768U;

// Dimensions of the coarse-pass beams in pixels.
static constexpr int BEAM_WIDTH_PX = 8;
static constexpr int BEAM_HEIGHT_PX = 8;

// Maximum size (in bytes) of the SVO SSBO GPU memory buffer
static constexpr usize MAX_SVO_SSBO_SIZE = 1024ULL*1024ULL*128ULL;

// The actual memory used for the hashmap buffer is
// HTABLE_SLOT_COUNT * sizeof(htable_entry). This is usually 8 bytes.
static constexpr usize HTABLE_SLOT_COUNT = 1024ULL*1024ULL*8ULL;

// Byte pattern used to indicate the hashtable null (aka empty) key.
// Since this is used as an argument to memset, only the unsigned char
// conversion of this value is used, so don't try changing it!
static constexpr int HTABLE_NULL_KEY_BYTE = 0xFF;

// Maximum size of compute shader workgroup as defined by the OpenGL spec.
static constexpr usize MAX_WORK_GROUP_SIZE = 65535ULL;


enum cs_bindings
{
    BIND_RENDER_TEX = 0,
    BIND_BEAM_TEX = 1,
};

enum htable_cs_bindings
{
    HTableOutputBufferBinding = 0,
    HTableInputBufferBinding = 1,
};

enum htable_uniforms
{
    HTableUniformTableSize = 2,
    HTableUniformOffset = 3,
};


struct gl_timer
{
    uint    Front;
    gl_uint Q[2];
};

// Holds the contextual data required to render a SVO voxel scene with OpenGL
struct render_data
{
    gl_uint CanvasVAO;    // VAO holding canvas vertices
    gl_uint CanvasVBO;    // VBO holding canvas vertices

    gl_uint SvoBuffer;    // Buffer for the SVO voxel data
    gl_uint FarPtrBuffer; // Buffer for the SVO far-ptr data

    gl_uint RenderImage;  // GL texture ID of the render output image
    gl_uint BeamImage;    // Texture holding coarse distance image for beam optimisation

    gl_int ViewMatUniformLocation; // Location of view matrix uniform
    gl_int ViewPosUniformLocation; // Location of view position uniform
    gl_int IsCoarsePassUniformLocation;

    gl_uint HasherShader; // Compute shader to construct the leaf hashtable
    gl_uint RenderShader; // Shader ID of the raycaster CS
    gl_uint CanvasShader; // Shader program ID for canvas
    gl_uint HTableInputBuffer;
    gl_uint HTableOutputBuffer;

    gl_timer Timer;
};


struct buffer_pair
{
    union { gl_uint SvoBuffer, VAO, InputBuffer; };
    union { gl_uint FarPtrBuffer, VBO, OutputBuffer; };
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


static inline gl_timer
CreateGPUTimer(void)
{
    gl_timer Timer = {};

    glGenQueries(2, Timer.Q);
    glQueryCounter(Timer.Q[Timer.Front], GL_TIME_ELAPSED);

    return Timer;
}

static inline void
DeleteGPUTimer(gl_timer* Timer)
{
    glDeleteQueries(2, Timer->Q);
}

static inline u64
GetGPUTimeElapsed(gl_timer* Timer)
{
    gl_u64 Time = 0;
    glGetQueryObjectui64v(Timer->Q[Timer->Front], GL_QUERY_RESULT, &Time);

    // Swap active query (0 -> 1, 1 -> 0)
    Timer->Front ^= 1;

    return static_cast<u64>(Time);
}


static inline void
BeginTimerQuery(const gl_timer* Timer)
{
    glBeginQuery(GL_TIME_ELAPSED, Timer->Q[Timer->Front]);
}

static inline void
EndTimerQuery(const gl_timer* const Timer)
{
    glEndQuery(GL_TIME_ELAPSED);
}

static buffer_pair
CreateLeafDataHashTable(const render_data* const RenderData, 
                        const attrib_data* const Data,
                        usize Count)
{
    gl_uint DataBuffers[2] = { 0 };
    const usize HTableDataSize = (HTABLE_SLOT_COUNT * sizeof(attrib_data));

    glGenBuffers(2, DataBuffers);

    if (Count > 0 && nullptr != Data)
    {

        // Upload the input leaf data key-value pairs into the leaf buffer
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, DataBuffers[0]);
        glBufferData(GL_SHADER_STORAGE_BUFFER, Count*sizeof(attrib_data), nullptr, GL_DYNAMIC_COPY);
        attrib_data* GPUDataBuffer = (attrib_data*)glMapBuffer(GL_SHADER_STORAGE_BUFFER,
                                                               GL_WRITE_ONLY);
        assert(GPUDataBuffer);
        std::memcpy(GPUDataBuffer, Data, Count*sizeof(attrib_data));
        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, DataBuffers[1]);
        glBufferData(GL_SHADER_STORAGE_BUFFER, HTableDataSize, nullptr, GL_DYNAMIC_COPY);
        attrib_data* GPUHTableBuffer = (attrib_data*) glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);
        assert(GPUDataBuffer);
        std::memset(GPUHTableBuffer, HTABLE_NULL_KEY_BYTE, HTableDataSize);
        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

        glUseProgram(RenderData->HasherShader);
        glUniform1ui(HTableUniformTableSize, HTABLE_SLOT_COUNT);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, HTableInputBufferBinding, DataBuffers[0]);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, HTableOutputBufferBinding, DataBuffers[1]);

        LogInfo("Begin voxel hash table construction");

        Count = Minimum(Count, HTABLE_SLOT_COUNT);
        usize Remaining = Count;
        while (Remaining > 0)
        {
            usize WorkGroupCount = Minimum(MAX_WORK_GROUP_SIZE, Remaining);

            glUniform1ui(HTableUniformOffset, Count - Remaining);
            glDispatchCompute(WorkGroupCount, 1, 1);
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

            Remaining -= WorkGroupCount;
        }
    }

    return buffer_pair{ {DataBuffers[0]}, {DataBuffers[1]} };
}


static buffer_pair
UploadCanvasVertices(void)
{
    buffer_pair Result = { };
    gl_uint VAO, VBO;

    glGenBuffers(1, &VBO);
    glGenVertexArrays(1, &VAO);

    assert(VAO);
    assert(VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER,
                 12*sizeof(f32),
                 GlobalCanvasVerts,
                 GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);

    Result.VAO = VAO;
    Result.VBO = VBO;

    return Result;
}


static buffer_pair
UploadSvoBlockData(const svo* const Svo)
{
    buffer_pair Buffers = { };
    usize NodeBlkSize = (SBR_NODES_PER_BLK * sizeof(svo_node));
    usize FarPtrBlkSize = (SBR_FAR_PTRS_PER_BLK * sizeof(far_ptr));
    usize BlkSize = NodeBlkSize + FarPtrBlkSize;

    usize MaxViewableBlkCount = (MAX_SVO_SSBO_SIZE / BlkSize);
    usize ViewableBlkCount = Minimum(MaxViewableBlkCount, (usize)Svo->UsedBlockCount);

    if (ViewableBlkCount < static_cast<usize>(Svo->UsedBlockCount))
    {
        std::fprintf(stderr, "[WARNING] SVO block data exceeds max GPU buffer size\n");
    }

    usize NodeBufferSize = NodeBlkSize * ViewableBlkCount;
    usize FarPtrBufferSize = FarPtrBlkSize * ViewableBlkCount;

    // 0: Svo nodes buffer
    // 1: svo far ptr buffer
    gl_uint BlkBuffers[2] = { 0, 0 };
    glGenBuffers(2, BlkBuffers);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, BlkBuffers[1]);
    glBufferData(GL_SHADER_STORAGE_BUFFER,
                 FarPtrBufferSize,
                 nullptr,
                 GL_DYNAMIC_COPY);
    far_ptr* GPUFarPtrBuffer = (far_ptr*)glMapBuffer(GL_SHADER_STORAGE_BUFFER,
                                                     GL_WRITE_ONLY);


    glBindBuffer(GL_SHADER_STORAGE_BUFFER, BlkBuffers[0]);
    glBufferData(GL_SHADER_STORAGE_BUFFER, NodeBufferSize, nullptr, GL_DYNAMIC_COPY);
    svo_node* GPUNodeBuffer = (svo_node*)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);

    if (BlkBuffers[0] && BlkBuffers[1])
    {
        svo_block* CurrentBlk = Svo->RootBlock;
        usize NextSvoDataOffset = 0;
        usize NextFarPtrDataOffset = 0;

        for (usize BlkIndex = 0; BlkIndex < ViewableBlkCount; ++BlkIndex)
        {
            std::memcpy(GPUNodeBuffer + NextSvoDataOffset, CurrentBlk->Entries, CurrentBlk->NextFreeSlot * sizeof(svo_node));
            NextSvoDataOffset += CurrentBlk->NextFreeSlot;

            std::memcpy(GPUFarPtrBuffer + NextFarPtrDataOffset, CurrentBlk->FarPtrs, SBR_FAR_PTRS_PER_BLK * sizeof(far_ptr)); 
            NextFarPtrDataOffset += SBR_FAR_PTRS_PER_BLK;

            if (CurrentBlk->Next)
            {
                CurrentBlk = CurrentBlk->Next;
            }
            else
            {
                break;
            }
        }
    }

    // Unmap both buffers
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, BlkBuffers[1]);
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, BlkBuffers[0]);
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    
    // Associate buffers with shader slots
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, BlkBuffers[0]);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, BlkBuffers[1]);

    Buffers.SvoBuffer = BlkBuffers[0]; 
    Buffers.FarPtrBuffer = BlkBuffers[1];

    return Buffers;
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
        char Log[SHADER_LOG_BUFFER_SIZE] = { 0 };
        glGetShaderInfoLog(ShaderID, SHADER_LOG_BUFFER_SIZE, nullptr, Log);
        std::fprintf(stdout, "Failed to compile compute shader\n%s\n", Log);

        return 0;
    }

    glAttachShader(ProgramID, ShaderID);
    glLinkProgram(ProgramID);

    Success = 0;
    glGetProgramiv(ProgramID, GL_LINK_STATUS, &Success);

    if (0 == Success)
    {
        LogError("Failed to link shader");
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
        char Log[SHADER_LOG_BUFFER_SIZE] = { 0 };
        glGetShaderInfoLog(VertShader, SHADER_LOG_BUFFER_SIZE, nullptr, Log);
        std::fprintf(stderr, "%s\n", VertSrc);
        std::fprintf(stderr, "Failed to compile vertex shader\n%s", Log);
        return 0;
    }

    Success = false;
    FragShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(FragShader, 1, &FragSrc, nullptr);
    glCompileShader(FragShader);
    glGetShaderiv(FragShader, GL_COMPILE_STATUS, &Success);

    if (0 == Success)
    {
        char Log[SHADER_LOG_BUFFER_SIZE] = { 0 };
        glGetShaderInfoLog(FragShader, SHADER_LOG_BUFFER_SIZE, nullptr, Log);
        std::fprintf(stderr, "Failed to compile fragment shader\n%s", Log);
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
        std::fprintf(stderr, "Failed to link shader program\n");
        return 0;
    }

    glDeleteShader(VertShader);
    glDeleteShader(FragShader);

    return Program;
}


static inline void
SetUniformData(const svo* const Tree, render_data* const R)
{
    glUseProgram(R->RenderShader);

    glUniform1i(glGetUniformLocation(R->RenderShader, "OuputImgUniform"), R->RenderImage);
    glUniform1ui(glGetUniformLocation(R->RenderShader, "MaxDepthUniform"), Tree->MaxDepth);
    glUniform1ui(glGetUniformLocation(R->RenderShader, "ScaleExpUniform"), Tree->ScaleExponent);
    glUniform1ui(glGetUniformLocation(R->RenderShader, "BlockCountUniform"), Tree->UsedBlockCount);
    glUniform1ui(glGetUniformLocation(R->RenderShader, "EntriesPerBlockUniform"), SBR_NODES_PER_BLK);
    glUniform1ui(glGetUniformLocation(R->RenderShader, "FarPtrsPerBlockUniform"), SBR_FAR_PTRS_PER_BLK);
    glUniform1ui(glGetUniformLocation(R->RenderShader, "BiasUniform"), Tree->Bias.Scale);
    glUniform1f(glGetUniformLocation(R->RenderShader, "InvBiasUniform"), Tree->Bias.InvScale);
    glUniform1ui(glGetUniformLocation(R->RenderShader, "TableSizeUniform"), HTABLE_SLOT_COUNT);

    printf("Inv Bias: %f\n", (f64)Tree->Bias.InvScale);
    printf("Bias Scale: %u\n", 1U << Tree->Bias.Scale);
    printf("Bias: %u\n", Tree->Bias.Scale);
}


static inline gl_uint
CreateRenderImage(int ImgW, int ImgH)
{
    gl_uint OutTex;
    glGenTextures(1, &OutTex);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, OutTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 
                 0,
                 GL_RGBA8,
                 ImgW,
                 ImgH,
                 0,
                 GL_BGRA,
                 GL_UNSIGNED_BYTE,
                 nullptr);

    glBindImageTexture(BIND_RENDER_TEX,
                       OutTex,
                       0,
                       GL_FALSE,
                       0,
                       GL_WRITE_ONLY,
                       GL_RGBA8);
    
    return OutTex;
}

static inline gl_uint
CreateBeamDistanceImage(int RenderWidth, int RenderHeight)
{
    gl_uint Texture;
    glGenTextures(1, &Texture);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, Texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_R32F,
                 RenderWidth/BEAM_WIDTH_PX,
                 RenderHeight/BEAM_HEIGHT_PX,
                 0,
                 GL_RED,
                 GL_FLOAT,
                 nullptr);

    glBindImageTexture(BIND_BEAM_TEX,
                       Texture,
                       0,
                       GL_FALSE,
                       0,
                       GL_WRITE_ONLY,
                       GL_R32F);

    return Texture;
}


static buffer_pair
UploadOctreeBlockData(const svo* const Svo)
{
    buffer_pair RenderData = { };
    gl_uint SvoBuffer, FarPtrBuffer;

    // TODO(Liam): Look into combining these allocations
    glGenBuffers(1, &SvoBuffer);
    glGenBuffers(1, &FarPtrBuffer);

    // FIXME(Liam): BIG cleanup here needed - may need to bite the bullet
    // and just have the nodes be typedef'd into U32s anyway.
    static_assert(sizeof(svo_node) == sizeof(gl_uint), "Node size must == GLuint size!");

    if (SvoBuffer && FarPtrBuffer)
    {
        usize SvoBlockDataSize = sizeof(svo_node) * SBR_NODES_PER_BLK;
        // TODO(Liam): Waste here on the last block
        usize MaxSvoDataSize = SvoBlockDataSize * Svo->UsedBlockCount;

        usize FarPtrBlockDataSize = sizeof(far_ptr) * SBR_FAR_PTRS_PER_BLK;
        usize MaxFarPtrDataSize = FarPtrBlockDataSize * Svo->UsedBlockCount;

        // Allocate space for the far ptr buffer
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, FarPtrBuffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, MaxFarPtrDataSize, nullptr, GL_DYNAMIC_COPY);
        far_ptr* GPUFarPtrBuffer = (far_ptr*)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);

        // Allocate space for the node data buffer
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, SvoBuffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, MaxSvoDataSize, nullptr, GL_DYNAMIC_COPY);
        gl_uint* GPUSvoBuffer = (gl_uint*)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);

        // Reset SSBO buffer binding
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);


        // TODO(Liam): Don't send all the blocks to the renderer! Use only a subset.
        svo_block* CurrentBlk = Svo->RootBlock;
        usize NextSvoDataOffset = 0;
        usize NextFarPtrDataOffset = 0;

        while (CurrentBlk)
        {
            assert(NextSvoDataOffset + (CurrentBlk->NextFreeSlot*sizeof(svo_node)) <= MaxSvoDataSize);

            std::memcpy(GPUSvoBuffer + NextSvoDataOffset, CurrentBlk->Entries, CurrentBlk->NextFreeSlot * sizeof(svo_node));
            NextSvoDataOffset += CurrentBlk->NextFreeSlot;

            std::memcpy(GPUFarPtrBuffer + NextFarPtrDataOffset, CurrentBlk->FarPtrs, SBR_FAR_PTRS_PER_BLK * sizeof(far_ptr)); 
            NextFarPtrDataOffset += SBR_FAR_PTRS_PER_BLK;

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


extern u64
DrawScene(const render_data* const RenderData, const view_data* const ViewData)
{
    BeginTimerQuery(&RenderData->Timer);
    glUseProgram(RenderData->RenderShader);
    glUniformMatrix3fv(RenderData->ViewMatUniformLocation,
                       1,
                       GL_TRUE,
                       ViewData->CamTransform);
    glUniform3fv(RenderData->ViewPosUniformLocation, 1, ViewData->CamPos);

    glUniform1ui(RenderData->IsCoarsePassUniformLocation, 1);
    glDispatchCompute(WORK_SIZE_X/64, WORK_SIZE_Y/64, 1);

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    glUniform1ui(RenderData->IsCoarsePassUniformLocation, 0);
    glDispatchCompute(WORK_SIZE_X/8, WORK_SIZE_Y / 8, 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    EndTimerQuery(&RenderData->Timer);

    // Draw the canvas
    glUseProgram(RenderData->CanvasShader);
    glBindVertexArray(RenderData->CanvasVAO);

    glBindTexture(GL_TEXTURE_2D, RenderData->RenderImage);

    glBindBuffer(GL_ARRAY_BUFFER, RenderData->CanvasVBO);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    return GetGPUTimeElapsed(const_cast<gl_timer*>(&RenderData->Timer));
}


extern render_data*
CreateRenderData(const svo* Scene,
                 const view_data* ViewData,
                 const shader_data* Shaders)
{
    render_data* Data = (render_data*)std::calloc(1, sizeof(render_data));
    if (nullptr == Data)
    {
        std::fprintf(stderr, "Failed to allocate render data\n");
        return nullptr;
    }

    Data->RenderShader = CompileComputeShader(Shaders->Code[SHADER_RENDER_CS]);
    if (0 == Data->RenderShader)
    {
        std::fprintf(stderr, "Failed to compile compute shader\n");

        DeleteRenderData(Data);
        return nullptr;
    }
    TraceOK("Compiled compute shader");

    Data->CanvasShader = CompileShader(Shaders->Code[SHADER_MAIN_VS],
                                       Shaders->Code[SHADER_MAIN_FS]);
    if (0 == Data->CanvasShader)
    {
        std::fprintf(stderr, "Failed to compile canvas shader\n");

        DeleteRenderData(Data);
        return nullptr;
    }
    TraceOK("Compiled canvas shader");

    Data->HasherShader = CompileComputeShader(Shaders->Code[SHADER_HASHER_CS]);
    if (0 == Data->HasherShader)
    {
        std::fprintf(stderr, "Failed to compile hashtable builder shader\n");
        DeleteRenderData(Data);

        return nullptr;
    }
    TraceOK("Compiled hasher compute shader");

    buffer_pair HTableBuffers = CreateLeafDataHashTable(Data,
                                                        Scene->AttribData.data(),
                                                        Scene->AttribData.size());

    Data->HTableInputBuffer = HTableBuffers.InputBuffer;
    Data->HTableOutputBuffer = HTableBuffers.OutputBuffer;


    Data->BeamImage = CreateBeamDistanceImage(ViewData->ScreenWidth, ViewData->ScreenHeight);
    Data->RenderImage = CreateRenderImage(ViewData->ScreenWidth, ViewData->ScreenHeight);
    assert(Data->BeamImage);
    SetUniformData(Scene, Data);

    buffer_pair CanvasBuffers = UploadCanvasVertices();
    if (0 == CanvasBuffers.VAO || 0 == CanvasBuffers.VBO)
    {
        std::fprintf(stderr, "Failed to upload canvas vertices\n");

        DeleteRenderData(Data);
        return nullptr;
    }

    Data->CanvasVAO = CanvasBuffers.VAO;
    Data->CanvasVBO = CanvasBuffers.VBO;

    buffer_pair SceneBuffers = UploadSvoBlockData(Scene);
    if (0 == SceneBuffers.SvoBuffer || 0 == SceneBuffers.FarPtrBuffer)
    {
        DeleteRenderData(Data);
        return nullptr;
    }

    Data->SvoBuffer = SceneBuffers.SvoBuffer;
    Data->FarPtrBuffer = SceneBuffers.FarPtrBuffer;

    Data->ViewMatUniformLocation = glGetUniformLocation(Data->RenderShader, "ViewMatrixUniform");
    Data->ViewPosUniformLocation = glGetUniformLocation(Data->RenderShader, "ViewPosUniform");
    Data->IsCoarsePassUniformLocation = glGetUniformLocation(Data->RenderShader, "IsCoarsePassUniform");

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, Data->SvoBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, Data->HTableOutputBuffer);
    glActiveTexture(GL_TEXTURE0);

    /*FILE* OutFile = fopen("data/cs.nvasm", "wb");
    assert(OutFile);
    DEBUGOutputRenderShaderAssembly(Data, OutFile);
    fclose(OutFile);*/

    Data->Timer = CreateGPUTimer();

    return Data;
}


extern void
DeleteRenderData(render_data* RenderData)
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

        glBindTexture(GL_TEXTURE_3D, 0);

        DeleteGPUTimer(&RenderData->Timer);
    }

    std::free(RenderData);
}


extern void
UpdateRenderScene(const svo* Scene, render_data* const RenderDataOut)
{
    usize SvoBlockDataSize = sizeof(svo_node) * SBR_NODES_PER_BLK;
    // TODO(Liam): Waste here on the last block
    usize MaxSvoDataSize = SvoBlockDataSize * Scene->UsedBlockCount;

    usize FarPtrBlockDataSize = sizeof(far_ptr) * SBR_FAR_PTRS_PER_BLK;
    usize MaxFarPtrDataSize = FarPtrBlockDataSize * Scene->UsedBlockCount;

    // Allocate space for the far ptr buffer
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, RenderDataOut->FarPtrBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, MaxFarPtrDataSize, nullptr, GL_DYNAMIC_COPY);
    far_ptr* GPUFarPtrBuffer = (far_ptr*)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);

    // Allocate space for the node data buffer
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, RenderDataOut->SvoBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, MaxSvoDataSize, nullptr, GL_DYNAMIC_COPY);
    gl_uint* GPUSvoBuffer = (gl_uint*)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);

    // Reset SSBO buffer binding
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    // TODO(Liam): Don't send all the blocks to the renderer! Use only a subset.
    svo_block* CurrentBlk = Scene->RootBlock;
    usize NextSvoDataOffset = 0;
    usize NextFarPtrDataOffset = 0;

    while (CurrentBlk)
    {
        assert(NextSvoDataOffset + (CurrentBlk->NextFreeSlot*sizeof(svo_node)) <= MaxSvoDataSize);

        std::memcpy(GPUSvoBuffer + NextSvoDataOffset, CurrentBlk->Entries, CurrentBlk->NextFreeSlot * sizeof(svo_node));
        NextSvoDataOffset += CurrentBlk->NextFreeSlot;

        std::memcpy(GPUFarPtrBuffer + NextFarPtrDataOffset, CurrentBlk->FarPtrs, SBR_FAR_PTRS_PER_BLK * sizeof(far_ptr)); 
        NextFarPtrDataOffset += SBR_FAR_PTRS_PER_BLK;

        CurrentBlk = CurrentBlk->Next;
    }

    // Unmap both buffers
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, RenderDataOut->FarPtrBuffer);
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, RenderDataOut->SvoBuffer);
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
}

extern bool
UpdateRenderShaders(const svo* Scene, const shader_data* Shaders, u32 Changed, render_data* Out)
{
    // Updating either the vertex or fragment shader requires recompiling both.
    // TODO(Liam): Can we just recompile one and then re-link?
    if (Changed & (SHADER_MAIN_VS | SHADER_MAIN_FS))
    {
        gl_uint Program = CompileShader(Shaders->Code[SHADER_MAIN_VS],
                                        Shaders->Code[SHADER_MAIN_FS]);

        if (0 == Program)
        {
            std::fprintf(stderr, "Failed to compile shader\n");
            return false;
        }

        glDeleteProgram(Out->CanvasShader);
        Out->CanvasShader = Program;
    }

    if (Changed & (SHADER_RENDER_CS))
    {
        gl_uint Program = CompileComputeShader(Shaders->Code[SHADER_RENDER_CS]);
        if (0 == Program)
        {
            std::fprintf(stderr, "Failed to compile shader\n");
            return false;
        }

        glDeleteProgram(Out->RenderShader);
        Out->RenderShader = Program;

        // TODO(Liam): Optimise this
        SetUniformData(Scene, Out);
    }

    if (Changed & (SHADER_HASHER_CS))
    {
        gl_uint Program = CompileComputeShader(Shaders->Code[SHADER_HASHER_CS]);
        if (0 == Program)
        {
            std::fprintf(stderr, "Failed to compile shader\n");
            return false;
        }

        glDeleteProgram(Out->HasherShader);
        Out->HasherShader = Program;
        
        SetUniformData(Scene, Out);
    }

    return true;
}


extern bool
DEBUGOutputRenderShaderAssembly(const render_data* RenderData, FILE* OutFile)
{
    gl_sizei DataLength = 0;
    gl_enum BinFormats[64];
    gl_int AsmLength = 0;

    glGetProgramiv(RenderData->RenderShader, GL_PROGRAM_BINARY_LENGTH, &AsmLength);
    byte* ShaderAssembly = (byte*)std::malloc((usize)AsmLength);
    glGetProgramBinary(RenderData->RenderShader, AsmLength, &DataLength, BinFormats, ShaderAssembly);

    std::fwrite(ShaderAssembly, sizeof(byte), (usize)DataLength, OutFile);

    std::free(ShaderAssembly);

    // TODO(Liam): Error checking
    return true;
}

