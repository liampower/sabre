#version 450 core

layout (local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

#define EMPTY_KEY 0xFFFFFFFF
#define MAX_STEPS 16

struct entry
{
    uint Key;
    uint PackedNormal;
    uint PackedColour;
};

layout (std430, binding = 0) restrict buffer htable_out
{
    entry Entries[];
} HTableOutBuffer;

layout (std430, binding = 1) readonly restrict buffer data_in
{
    entry Entries[];
} DataInputBuffer;

layout (location = 2) uniform uint TableSizeUniform;
layout (location = 3) uniform uint OffsetUniform;

uvec4 ComputeHashes(uint Key)
{
    const uvec4 S0 = uvec4(16, 18, 17, 16);
    const uvec4 S1 = uvec4(15, 16, 15, 16);
    const uvec4 S2 = uvec4(16, 17, 16, 15);

    const uvec4 C0 = uvec4(0x7feb352d, 0xa136aaad, 0x24f4d2cd, 0xe2d0d4cb);
    const uvec4 C1 = uvec4(0x846ca68b, 0x9f6d62d7, 0x1ba3b969, 0x3c6ad939);

    uvec4 K = uvec4(Key);
    K ^= K >> S0;
    K *= C0;
    K ^= K >> S1;
    K *= C1;
    K ^= K >> S2;
    return K % (TableSizeUniform - 1);
}

void main()
{
    uint ThreadID = gl_GlobalInvocationID.x;
    entry InputPair = DataInputBuffer.Entries[OffsetUniform + ThreadID];

    uint Slot = ComputeHashes(InputPair.Key).x;

    int Step;
    for (Step = 0; Step < MAX_STEPS; ++Step)
    {
        InputPair.Key = atomicExchange(HTableOutBuffer.Entries[Slot].Key, InputPair.Key);
        InputPair.PackedNormal = atomicExchange(HTableOutBuffer.Entries[Slot].PackedNormal, InputPair.PackedNormal);
        InputPair.PackedColour = atomicExchange(HTableOutBuffer.Entries[Slot].PackedColour, InputPair.PackedColour);
        if (EMPTY_KEY != InputPair.Key)
        {
            uvec4 Hashes = ComputeHashes(InputPair.Key);

            if (Slot == Hashes.x) Slot = Hashes.y;
            else if (Slot == Hashes.y) Slot = Hashes.z;
            else if (Slot == Hashes.z) Slot = Hashes.w;
            else if (Slot == Hashes.w) Slot = Hashes.x;
        }
        else
        {
            break;
        }
    }

}

