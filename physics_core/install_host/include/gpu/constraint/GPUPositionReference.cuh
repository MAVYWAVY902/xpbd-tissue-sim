#ifndef __GPU_POSITION_REFERENCE_CUH
#define __GPU_POSITION_REFERENCE_CUH

struct GPUPositionReference
{
    float inv_mass; // inverse mass of the vertex
    int index; // vertex index
};

#endif // __GPU_POSITION_REFERENCE_CUH