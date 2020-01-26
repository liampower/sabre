#include "sabre.h"
#include "sabre_math.h"
#include "sabre_svo.h"

#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include <queue>

#include "rply.h"

#include "sabre_svo.cc"

struct node_ctx
{
    svo_node* Node;
    svo_oct   Oct;
    u32       Depth;
    u32       Scale;
    vec3      Centre;
};

static bool
GeometryWithinNode(vec3 Min, vec3 Max, std::vector<vec3>& Vertices)
{
    for (vec3 V : Vertices)
    {
        if (Any(GreaterThan(V, Min)) || Any(LessThan(V, Max))) return true;
    }

    return false;
}

static svo*
AllocateEmptySvo(void)
{
    svo* Tree = (svo*)calloc(1, sizeof(svo));

    if (Tree)
    {
        Tree->RootBlock = (svo_block*)calloc(1, sizeof(svo_block));
        Tree->CurrentBlock = Tree->RootBlock;
        Tree->UsedBlockCount = 1;
        Tree->ScaleExponent = 5;
        Tree->MaxDepth = 4;

        return Tree;
    }
    else
    {
        return nullptr;
    }
}

static_assert(sizeof(vec3) == 3*sizeof(float), "Sizeof(vec3) must equal 3*sizeof(float)");

struct poly
{
    vec3 V[3];
};

struct mesh
{
    usize TriangleCount;
    usize VertexCount;

    usize Nv;
    usize Nt;

    f32*  VertexData;
    poly* TriangleData;
};

static int
VertexCallback(p_ply_argument Arg)
{
    mesh* Mesh;
    ply_get_argument_user_data(Arg, NULL, &Mesh);

    Mesh->VertexData[Mesh->Nv] = ply_get_argument_value(Arg);
    ++Mesh->Nv;

    return 1;
}

static int
TriangleCallback(p_ply_argument Arg)
{
    long Length, ValueIndex;
    ply_get_argument_property(Arg, NULL, &Length, &ValueIndex);

    mesh* Mesh;
    ply_get_argument_user_data(Arg, NULL, &Mesh);
}

static mesh*
LoadMeshFile(const char* const FilePath)
{
    p_ply PlyFile = ply_open(FilePath, NULL, 0, NULL); 

    if (PlyFile)
    {
        long VertexCount = 0;
        long TriangleCount = 0;

        mesh* Mesh = (mesh*)calloc(1, sizeof(mesh));

        ply_read_header(PlyFile);
        VertexCount = ply_set_read_cb(PlyFile, "vertex", "x", &VertexCallback, Mesh);

    }
    else
    {
        fprintf(stderr, "Failed to open PLY file\n");

        return nullptr;
    }
}


static svo*
BuildSvoFromPlyFile(const char* const FilePath)
{
    mesh* Mesh = LoadMeshFile(FilePath);
    
    svo* Svo = AllocateEmptySvo();


    // BFS
    std::queue<node_ctx> Queue;

    u32 SvoScale = 1 << Svo->ScaleExponent;
    vec3 RootCentre = vec3(SvoScale >> 1);
    vec3 Rad = vec3(SvoScale >> 1);
    vec3 RootMin = RootCentre - Rad;
    vec3 RootMax = RootCentre + Rad;

    bool _Ignored;
    svo_node* RootNode = AllocateNode(Svo, &_Ignored);

    if (GeometryWithinNode(RootMin, RootMax, Vtx))
    {
        // Push root node
        node_ctx RootCtx = { RootNode, OCT_C000, 0, SvoScale, RootCentre };
        Queue.push(RootCtx);

        fprintf(stdout, "Geometry within bounds\n");

        while (false == Queue.empty())
        {
            node_ctx CurrentCtx = Queue.front();
            u32 Scale = CurrentCtx.Scale >> 1;

            for (u32 Oct = 0; Oct < 8; ++Oct)
            {
                vec3 OctCentre = GetNodeCentrePosition((svo_oct)Oct, Scale, CurrentCtx.Centre);
                vec3 Rad = vec3(Scale >> 1);
                vec3 NodeMin = OctCentre - Rad;
                vec3 NodeMax = OctCentre + Rad;

                if (GeometryWithinNode(NodeMin, NodeMax, Vtx))
                {
                    if (CurrentCtx.Depth < Svo->MaxDepth - 1)
                    {
                        SetOctantOccupied((svo_oct)Oct, VOXEL_PARENT, CurrentCtx.Node);

                        // Geometry present
                        svo_node* Child = AllocateNode(Svo, &_Ignored);

                        node_ctx ChildCtx = { Child, (svo_oct)Oct, CurrentCtx.Depth + 1, Scale >> 1, OctCentre };
                        
                        ptrdiff_t ChildOffset = Child - Svo->CurrentBlock->Entries;
                        SetNodeChildPointer((u16)ChildOffset, false, Svo, CurrentCtx.Node);

                        Queue.push(ChildCtx);
                    }
                    else
                    {
                        SetOctantOccupied((svo_oct)Oct, VOXEL_LEAF, CurrentCtx.Node);
                    }

                }
            }

            Queue.pop();
                
        }
    }
    else
    {
        printf("WARNING: No geometry present\n");
    }


    In.close();

    return Svo;

}


int
main(int ArgCount, const char** Args)
{

    BuildSvoFromPlyFile("C:/Users/Liam/sabre/data/TestModels/sphere_r16.ply");
}
