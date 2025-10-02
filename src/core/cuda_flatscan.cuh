
#ifndef CUDA_FLATSCAN_CUH
#define CUDA_FLATSCAN_CUH

#include "cuda_point_types.cuh"
#include <vector>

// Host-side function to launch the CUDA kernel for LaserScan generation
cudaError_t generateFlatScanGPU(
    CudaPointXYZI* d_input_cloud,
    size_t num_input_points,
    const CudaLaserScanParams& params,
    float** d_ranges_output); // Output device pointer for ranges

#endif // CUDA_FLATSCAN_CUH
