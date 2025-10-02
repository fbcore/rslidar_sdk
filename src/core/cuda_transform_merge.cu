#include "cuda_transform_merge.cuh"
#include <cuda_runtime.h>

// CUDA kernel to transform and merge points
__global__ void transformAndMergeKernel(
    CudaPointXYZI** d_input_clouds,
    size_t* d_input_counts,
    CudaMatrix4f* d_transforms,
    size_t num_lidars,
    CudaPointXYZI* d_output_cloud)
{
    size_t global_idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Find which lidar this global_idx belongs to
    size_t current_offset = 0;
    for (size_t lidar_idx = 0; lidar_idx < num_lidars; ++lidar_idx)
    {
        size_t lidar_points = d_input_counts[lidar_idx];
        if (global_idx < current_offset + lidar_points)
        {
            // This point belongs to lidar_idx
            size_t local_idx = global_idx - current_offset;
            CudaPointXYZI p = d_input_clouds[lidar_idx][local_idx];
            CudaMatrix4f transform = d_transforms[lidar_idx];

            // Apply transform
            CudaPointXYZI transformed_p = transform.transform(p);

            // Write to output
            d_output_cloud[global_idx] = transformed_p;
            return;
        }
        current_offset += lidar_points;
    }
}

// Host-side function to launch the CUDA kernel
cudaError_t transformAndMergeGPU(
    const std::vector<CudaPointXYZI*>& h_input_clouds,
    const std::vector<size_t>& h_input_counts,
    const std::vector<CudaMatrix4f>& h_transforms,
    CudaPointXYZI* d_output_cloud,
    size_t total_output_points)
{
    // Copy host vectors to device
    CudaPointXYZI** d_input_clouds;
    cudaMalloc((void**)&d_input_clouds, h_input_clouds.size() * sizeof(CudaPointXYZI*));
    cudaMemcpy(d_input_clouds, h_input_clouds.data(), h_input_clouds.size() * sizeof(CudaPointXYZI*), cudaMemcpyHostToDevice);

    size_t* d_input_counts_ptr;
    cudaMalloc((void**)&d_input_counts_ptr, h_input_counts.size() * sizeof(size_t));
    cudaMemcpy(d_input_counts_ptr, h_input_counts.data(), h_input_counts.size() * sizeof(size_t), cudaMemcpyHostToDevice);

    CudaMatrix4f* d_transforms_ptr;
    cudaMalloc((void**)&d_transforms_ptr, h_transforms.size() * sizeof(CudaMatrix4f));
    cudaMemcpy(d_transforms_ptr, h_transforms.data(), h_transforms.size() * sizeof(CudaMatrix4f), cudaMemcpyHostToDevice);

    // Launch kernel
    const int BLOCK_SIZE = 256;
    int num_blocks = (total_output_points + BLOCK_SIZE - 1) / BLOCK_SIZE;
    transformAndMergeKernel<<<num_blocks, BLOCK_SIZE>>>(d_input_clouds, d_input_counts_ptr, d_transforms_ptr, h_input_clouds.size(), d_output_cloud);

    // Check for CUDA errors
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess)
    {
        fprintf(stderr, "CUDA kernel launch failed: %s\n", cudaGetErrorString(err));
    }

    // Free device memory for temporary arrays
    cudaFree(d_input_clouds);
    cudaFree(d_input_counts_ptr);
    cudaFree(d_transforms_ptr);

    return err;
}
