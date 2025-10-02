
#ifndef CUDA_ROI_FILTER_CUH
#define CUDA_ROI_FILTER_CUH

#include "cuda_point_types.cuh"
#include <vector>

// Struct to pass ROI filter configurations to CUDA
struct CudaRoiFilterConfig
{
    float min_x, max_x, min_y, max_y, min_z, max_z;
    int type; // 0 for positive, 1 for negative
};

// Host-side function to launch the CUDA kernel for ROI filtering
cudaError_t roiFilterGPU(
    CudaPointXYZI* d_input_cloud,
    size_t num_input_points,
    const CudaRoiFilterConfig* d_filter_configs,
    size_t num_filter_configs,
    CudaPointXYZI** d_output_cloud,
    size_t* num_output_points);

#endif // CUDA_ROI_FILTER_CUH
