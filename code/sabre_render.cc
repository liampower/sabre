#include <glad/glad.h>
#include <assert.h>
#include <map>
#include <unordered_map>

#include "sabre.h"
#include "sabre_svo.h"
#include "sabre_render.h"
#include "sabre_data.h"

static constexpr uint WORK_SIZE_X = 512U;
static constexpr uint WORK_SIZE_Y = 512U;

typedef u64 morton_key;
extern morton_key EncodeMorton3(uvec3 V);

struct uvec3_hash
{
public:
    size_t operator()(const uvec3& Element) const{
        return EncodeMorton3(Element);
    }
};


typedef GLuint gl_uint;
typedef GLint  gl_int;
typedef GLsizei gl_sizei;
typedef GLenum gl_enum;

enum cs_bindings
{
    BIND_RENDER_TEX = 0,
    BIND_NORMALS_TEX = 1
};

enum htable_cs_bindings
{
    HTableOutputBufferBinding = 0,
    HTableInputBufferBinding = 1,
};

// Holds the contextual data required to render a SVO voxel
// scene with OpenGL.
struct render_data
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

    gl_uint MapTexture;
    gl_uint ColourTexture;

    gl_uint HTableBuilderShader; // Compute shader to construct the leaf hashtable
    gl_uint HTableInputBuffer;
    gl_uint HTableOutputBuffer;
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


static gl_uint
CreateLeafDataHashTable(render_data* RenderData, const attrib_data* const Data, usize Count)
{
    const usize MaxHTableBufferSize = 8*1024*1024;
    gl_uint DataBuffers[2] = { 0 };

    glGenBuffers(2, DataBuffers);

    { // Upload the input leaf data key-value pairs into the leaf buffer
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, DataBuffers[0]);
        glBufferData(GL_SHADER_STORAGE_BUFFER, Count*sizeof(packed_snorm3), nullptr, GL_DYNAMIC_COPY);
        packed_snorm3* GPUDataBuffer = (packed_snorm3*)glMapBuffer(GL_SHADER_STORAGE_BUFFER,
                                                         GL_WRITE_ONLY);
    
        memcpy(GPUDataBuffer, Data, Count*sizeof(packed_snorm3));
        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    }

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, DataBuffers[1]);
    glBufferData(GL_SHADER_STORAGE_BUFFER, MaxHTableBufferSize*sizeof(gl_uint), nullptr, GL_DYNAMIC_COPY);
    gl_uint* GPUHTableBuffer = (gl_uint*) glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);
    memset(GPUHTableBuffer, 0xFFFFFFFF, MaxHTableBufferSize*sizeof(gl_uint));
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);


    glUseProgram(RenderData->HTableBuilderShader);
    glUniform1ui(glGetUniformLocation(RenderData->HTableBuilderShader, "TableSizeUniform"), MaxHTableBufferSize);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, HTableInputBufferBinding, DataBuffers[0]);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, HTableOutputBufferBinding, DataBuffers[1]);

    // Kick the table builder kernel
    glDispatchCompute(50, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    RenderData->HTableInputBuffer = DataBuffers[0];
    RenderData->HTableOutputBuffer = DataBuffers[1];

    return 0;
}

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

static svo_buffers
UploadSvoBlockData(const svo* const Svo)
{
    svo_buffers Buffers = { };
    usize NodeBlkSize = (SBR_NODES_PER_BLK * sizeof(svo_node));
    usize FarPtrBlkSize = (SBR_FAR_PTRS_PER_BLK * sizeof(far_ptr));
    usize BlkSize = NodeBlkSize + FarPtrBlkSize;

    // The minimum SSBO size guaranteed by the implementation is 128MiB. To be
    // safe (and to avoid gratuitous memory hogging) we allocate 16MiB for the
    // view buffer.
    usize MaxSSBOSize = 16777216ULL;
    usize MaxViewableBlkCount = (MaxSSBOSize / BlkSize);
    usize ViewableBlkCount = Min(MaxViewableBlkCount, Svo->UsedBlockCount);
    usize NodeBufferSize = NodeBlkSize * ViewableBlkCount;
    usize FarPtrBufferSize = FarPtrBlkSize * ViewableBlkCount;

    // 0: Svo nodes buffer
    // 1: svo far ptr buffer
    gl_uint BlkBuffers[2] = { 0, 0 };
    glGenBuffers(2, BlkBuffers);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, BlkBuffers[1]);
    glBufferData(GL_SHADER_STORAGE_BUFFER, FarPtrBufferSize, nullptr, GL_DYNAMIC_COPY);
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
            memcpy(GPUNodeBuffer + NextSvoDataOffset, CurrentBlk->Entries, CurrentBlk->NextFreeSlot * sizeof(svo_node));
            NextSvoDataOffset += CurrentBlk->NextFreeSlot;

            memcpy(GPUFarPtrBuffer + NextFarPtrDataOffset, CurrentBlk->FarPtrs, SBR_FAR_PTRS_PER_BLK * sizeof(far_ptr)); 
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
SetUniformData(const svo* const Tree, render_data* const RenderData)
{
    glUseProgram(RenderData->RenderShader);

    glUniform1i(glGetUniformLocation(RenderData->RenderShader, "OuputImgUniform"), RenderData->RenderImage);
    glUniform1ui(glGetUniformLocation(RenderData->RenderShader, "MaxDepthUniform"), Tree->MaxDepth);
    glUniform1ui(glGetUniformLocation(RenderData->RenderShader, "ScaleExponentUniform"), Tree->ScaleExponent);
    glUniform1ui(glGetUniformLocation(RenderData->RenderShader, "BlockCountUniform"), Tree->UsedBlockCount);
    glUniform1ui(glGetUniformLocation(RenderData->RenderShader, "EntriesPerBlockUniform"), SBR_NODES_PER_BLK);
    glUniform1ui(glGetUniformLocation(RenderData->RenderShader, "FarPtrsPerBlockUniform"), SBR_FAR_PTRS_PER_BLK);
    glUniform1ui(glGetUniformLocation(RenderData->RenderShader, "BiasUniform"), Tree->Bias.Scale);
    glUniform1f(glGetUniformLocation(RenderData->RenderShader, "InvBiasUniform"), Tree->Bias.InvScale);
    glUniform1i(glGetUniformLocation(RenderData->RenderShader, "MapDataUniform"), 1);
    glUniform1i(glGetUniformLocation(RenderData->RenderShader, "ColourDataUniform"), 0);

    printf("Inv Bias: %f\n", (f64)Tree->Bias.InvScale);
    printf("Bias Scale: %u\n", 1U << Tree->Bias.Scale);
    printf("Bias: %u\n", Tree->Bias.Scale);
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
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, ImgWidth, ImgHeight, 0, GL_RGBA, GL_FLOAT, nullptr);

    glBindImageTexture(BIND_RENDER_TEX, OutputTexture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);
    
    return OutputTexture;
}


struct tex_page
{
    packed_snorm3* Data;

    /*tex_page(usize PageSizeX, usize PageSizeY, usize PageSizeZ)
    {
        Data = (packed_snorm3*) malloc(PageSizeX*PageSizeY*PageSizeZ);
    }*/
};

static std::unordered_map<uvec3, tex_page, uvec3_hash>
PartitionLeafDataToBuckets(const std::vector<std::pair<uvec3, u32>>& LeafData, usize PageSizeX, usize PageSizeY, usize PageSizeZ)
{
    std::unordered_map<uvec3, tex_page, uvec3_hash> Pages;
    const uvec3 PageSize = uvec3(PageSizeX, PageSizeY, PageSizeZ);

    for (auto It = LeafData.begin(); It != LeafData.end(); ++It)
    {
        auto Element = *It;

        // Subtexture coords
        uvec3 Bucket = Element.first / PageSize;
        uvec3 PageCoords = Element.first % PageSize;
        usize DataIndex = (PageSizeX*PageSizeY*PageCoords.Z) + (PageSizeX*PageCoords.Y) + PageCoords.X;
        assert(DataIndex < (PageSizeX*PageSizeY*PageSizeZ));

        // Key already exists
        if (Pages.find(Bucket) != Pages.end())
        {
            tex_page* Page = &Pages.find(Bucket)->second;

            Page->Data[DataIndex] = Element.second;
        }
        else
        {
            tex_page Page = {};

            // I really hate this, but can't risk distribution page sizes being different
            Page.Data = (packed_snorm3*) malloc(PageSizeX*PageSizeY*PageSizeZ*sizeof(packed_snorm3));
            Page.Data[DataIndex] = Element.second;
            Pages.insert(std::make_pair(Bucket, Page));
        }
    }

    return Pages;
}


static gl_uint
UploadLeafDataSparse(std::vector<std::pair<uvec3, packed_snorm3>> Data, int AttachmentIndex)
{
    gl_uint MapTexture;
    glCreateTextures(GL_TEXTURE_3D, 1, &MapTexture);

    if (MapTexture)
    {
        glActiveTexture(GL_TEXTURE0 + AttachmentIndex);
        glBindTexture(GL_TEXTURE_3D, MapTexture);

        // Read the page sizes
        gl_int PageSizeX, PageSizeY, PageSizeZ;
		glGetInternalformativ(GL_TEXTURE_3D, GL_RGBA8_SNORM, GL_VIRTUAL_PAGE_SIZE_X_ARB, 1, &PageSizeX);
		glGetInternalformativ(GL_TEXTURE_3D, GL_RGBA8_SNORM, GL_VIRTUAL_PAGE_SIZE_Y_ARB, 1, &PageSizeY);
		glGetInternalformativ(GL_TEXTURE_3D, GL_RGBA8_SNORM, GL_VIRTUAL_PAGE_SIZE_Z_ARB, 1, &PageSizeZ);
        assert(PageSizeX > 0 && PageSizeY > 0 && PageSizeZ > 0);
        fprintf(stderr, "Page size: %d %d %d\n", PageSizeX, PageSizeY, PageSizeZ);

        std::unordered_map<uvec3, tex_page, uvec3_hash> Pages = PartitionLeafDataToBuckets(Data, PageSizeX, PageSizeY, PageSizeZ);

        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

        // Enable sparse texture storage
		glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_SPARSE_ARB, GL_TRUE);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        glTexStorage3D(GL_TEXTURE_3D, 1, GL_RGBA8_SNORM, 2048, 2048, 2048);

        gl_int MaxTextureSize;
        glGetIntegerv(GL_MAX_SPARSE_3D_TEXTURE_SIZE_ARB, &MaxTextureSize);

        size_t MaxPageCount = ((size_t)MaxTextureSize*(size_t)MaxTextureSize*(size_t)MaxTextureSize);
        MaxPageCount /= ((size_t)PageSizeX*(size_t)PageSizeY*(size_t)PageSizeZ);

        printf("Max Pages: %zu\n", MaxPageCount);

        size_t PageCount = 0;
        MaxPageCount = 1024;
        for (auto It = Pages.begin(); It != Pages.end(); ++It)
        {
            // Too many pages; avoid exhausting the GPU memory
            if (PageCount == MaxPageCount) break;

            auto Element = *It;
            gl_int PageX = (gl_int)Element.first.X*PageSizeX;
            gl_int PageY = (gl_int)Element.first.Y*PageSizeY;
            gl_int PageZ = (gl_int)Element.first.Z*PageSizeZ;

            // Mark the page containing this point as backed by physical
            // memory.
            glTexPageCommitmentARB(GL_TEXTURE_3D,
                                   0,
                                   PageX,
                                   PageY,
                                   PageZ,
                                   PageSizeX,
                                   PageSizeY,
                                   PageSizeZ,
                                   GL_TRUE);

            // Upload the leaf data into the subtexture page
            glTexSubImage3D(GL_TEXTURE_3D,
                            0,
                            PageX,
                            PageY,
                            PageZ,
                            PageSizeX,
                            PageSizeY,
                            PageSizeZ,
                            GL_RGBA,
                            GL_BYTE,
                            Element.second.Data);

           free(Element.second.Data);
           ++PageCount;
        }
        
        printf("Used cell count %zu\n", PageCount);

        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    }

    return MapTexture;
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

            memcpy(GPUSvoBuffer + NextSvoDataOffset, CurrentBlk->Entries, CurrentBlk->NextFreeSlot * sizeof(svo_node));
            NextSvoDataOffset += CurrentBlk->NextFreeSlot;

            memcpy(GPUFarPtrBuffer + NextFarPtrDataOffset, CurrentBlk->FarPtrs, SBR_FAR_PTRS_PER_BLK * sizeof(far_ptr)); 
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


extern "C" void
DrawScene(const render_data* const RenderData, const view_data* const ViewData)
{
    glUseProgram(RenderData->RenderShader);
    glUniformMatrix3fv(RenderData->ViewMatUniformLocation, 1, GL_TRUE, ViewData->CamTransform);
    glUniform3fv(RenderData->ViewPosUniformLocation, 1, ViewData->CamPos);
    glDispatchCompute(WORK_SIZE_X/8, WORK_SIZE_Y/8, 1);

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    // Draw the canvas
    glUseProgram(RenderData->CanvasShader);
    glBindVertexArray(RenderData->CanvasVAO);

    glBindTexture(GL_TEXTURE_2D, RenderData->RenderImage);

    glBindBuffer(GL_ARRAY_BUFFER, RenderData->CanvasVBO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
}


extern "C" render_data*
CreateRenderData(const svo* const Tree, const view_data* const ViewData)
{
    render_data* RenderData = (render_data*) calloc(1, sizeof(render_data));
    if (nullptr == RenderData)
    {
        fprintf(stderr, "Failed to allocate render data\n");
        return nullptr;
    }

    RenderData->RenderShader = CompileComputeShader(RaycasterComputeKernel);
    if (0 == RenderData->RenderShader)
    {
        fprintf(stderr, "Failed to compile compute shader\n");

        DeleteRenderData(RenderData);
        return nullptr;
    }

    RenderData->CanvasShader = CompileShader(MainVertexCode, MainFragmentCode);
    if (0 == RenderData->RenderShader)
    {
        fprintf(stderr, "Failed to compile canvas shader\n");

        DeleteRenderData(RenderData);
        return nullptr;
    }

    RenderData->HTableBuilderShader = CompileComputeShader(HasherComputeKernel);
    if (0 == RenderData->HTableBuilderShader)
    {
        fprintf(stderr, "Failed to compile hashtable builder compute shader\n");
        DeleteRenderData(RenderData);

        return nullptr;
    }

    CreateLeafDataHashTable(RenderData, Tree->Normals.data(), Tree->Normals.size());


    RenderData->RenderImage = CreateRenderImage(ViewData->ScreenWidth, ViewData->ScreenHeight);
    SetUniformData(Tree, RenderData);

    svo_buffers CanvasBuffers = UploadCanvasVertices();
    if (0 == CanvasBuffers.VAO || 0 == CanvasBuffers.VBO)
    {
        fprintf(stderr, "Failed to upload canvas vertices\n");

        DeleteRenderData(RenderData);
        return nullptr;
    }

    RenderData->CanvasVAO = CanvasBuffers.VAO;
    RenderData->CanvasVBO = CanvasBuffers.VBO;

    RenderData->ColourTexture = UploadLeafDataSparse(Tree->Colours, 0);
    //RenderData->MapTexture = UploadLeafDataSparse(Tree->Normals, 1);


    /*if (0 == RenderData->MapTexture)
    {
        DeleteRenderData(RenderData);
        return nullptr;
    }*/

    if (0 == RenderData->ColourTexture)
    {
        DeleteRenderData(RenderData);
        return nullptr;
    }

    svo_buffers SvoBuffers = UploadSvoBlockData(Tree);
    if (0 == SvoBuffers.SvoBuffer || 0 == SvoBuffers.FarPtrBuffer)
    {
        DeleteRenderData(RenderData);
        return nullptr;
    }

    RenderData->SvoBuffer = SvoBuffers.SvoBuffer;
    RenderData->FarPtrBuffer = SvoBuffers.FarPtrBuffer;


    RenderData->ViewMatUniformLocation = glGetUniformLocation(RenderData->RenderShader, "ViewMatrixUniform");
    RenderData->ViewPosUniformLocation = glGetUniformLocation(RenderData->RenderShader, "ViewPosUniform");

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, RenderData->SvoBuffer);
    glActiveTexture(GL_TEXTURE0);

    return RenderData;
}

extern "C" void
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

        glDeleteTextures(1, &RenderData->MapTexture);
        glDeleteTextures(1, &RenderData->ColourTexture);

        free(RenderData);
    }
}


extern "C" void
UpdateRenderData(const svo* const Svo, render_data* const RenderDataOut)
{
    usize SvoBlockDataSize = sizeof(svo_node) * SBR_NODES_PER_BLK;
    // TODO(Liam): Waste here on the last block
    usize MaxSvoDataSize = SvoBlockDataSize * Svo->UsedBlockCount;

    usize FarPtrBlockDataSize = sizeof(far_ptr) * SBR_FAR_PTRS_PER_BLK;
    usize MaxFarPtrDataSize = FarPtrBlockDataSize * Svo->UsedBlockCount;

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
    svo_block* CurrentBlk = Svo->RootBlock;
    usize NextSvoDataOffset = 0;
    usize NextFarPtrDataOffset = 0;

    while (CurrentBlk)
    {
        assert(NextSvoDataOffset + (CurrentBlk->NextFreeSlot*sizeof(svo_node)) <= MaxSvoDataSize);

        memcpy(GPUSvoBuffer + NextSvoDataOffset, CurrentBlk->Entries, CurrentBlk->NextFreeSlot * sizeof(svo_node));
        NextSvoDataOffset += CurrentBlk->NextFreeSlot;

        memcpy(GPUFarPtrBuffer + NextFarPtrDataOffset, CurrentBlk->FarPtrs, SBR_FAR_PTRS_PER_BLK * sizeof(far_ptr)); 
        NextFarPtrDataOffset += SBR_FAR_PTRS_PER_BLK;

        CurrentBlk = CurrentBlk->Next;
    }

    // Unmap both buffers
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, RenderDataOut->FarPtrBuffer);
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, RenderDataOut->SvoBuffer);
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
}


extern "C" bool
DEBUGOutputRenderShaderAssembly(const render_data* const RenderData, FILE* OutFile)
{
    gl_sizei DataLength = 0;
    gl_enum BinFormats[64];
    gl_int AsmLength = 0;

    glGetProgramiv(RenderData->RenderShader, GL_PROGRAM_BINARY_LENGTH, &AsmLength);
    byte* ShaderAssembly = (byte*)malloc((usize)AsmLength);
    glGetProgramBinary(RenderData->RenderShader, AsmLength, &DataLength, BinFormats, ShaderAssembly);

    fwrite(ShaderAssembly, sizeof(byte), (usize)DataLength, OutFile);

    free(ShaderAssembly);

    // TODO(Liam): Error checking
    return true;
}



