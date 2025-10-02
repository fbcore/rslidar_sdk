
#ifndef CUDA_VOXEL_GRID_CUH
#define CUDA_VOXEL_GRID_CUH

#include "cuda_point_types.cuh"
#include <vector>

// Host-side function to launch the CUDA kernel for voxel grid downsampling
cudaError_t voxelGridDownsampleGPU(
    CudaPointXYZI* d_input_cloud,
    size_t num_input_points,
    float leaf_size,
    CudaPointXYZI** d_output_cloud,
    size_t* num_output_points);

#endif // CUDA_VOXEL_GRID_CUH
