
#ifndef CUDA_TRANSFORM_MERGE_CUH
#define CUDA_TRANSFORM_MERGE_CUH

#include "cuda_point_types.cuh"
#include <vector>
#include <memory>

// Host-side function to launch the CUDA kernel for transforming and merging point clouds
cudaError_t transformAndMergeGPU(
    const std::vector<CudaPointXYZI*>& d_input_clouds,
    const std::vector<size_t>& h_input_counts,
    const std::vector<CudaMatrix4f>& h_transforms,
    CudaPointXYZI* d_output_cloud,
    size_t total_output_points);

#endif // CUDA_TRANSFORM_MERGE_CUH
