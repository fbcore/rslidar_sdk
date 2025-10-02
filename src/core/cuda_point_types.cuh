
#ifndef CUDA_POINT_TYPES_CUH
#define CUDA_POINT_TYPES_CUH

#include <cstddef>

// Point structure for GPU memory
struct CudaPointXYZI
{
    float x, y, z, intensity;
};

// Simple 4x4 matrix structure for CUDA kernels
struct CudaMatrix4f
{
    float data[16]; // Column-major order, like Eigen

    // Host-side conversion from Eigen::Matrix4f
    __host__ void fromEigen(const float* eigen_data)
    {
        for (int i = 0; i < 16; ++i)
        {
            data[i] = eigen_data[i];
        }
    }

    // Device-side multiplication with a point
    __device__ CudaPointXYZI transform(const CudaPointXYZI& p) const
    {
        CudaPointXYZI res;
        res.x = data[0] * p.x + data[4] * p.y + data[8] * p.z + data[12];
        res.y = data[1] * p.x + data[5] * p.y + data[9] * p.z + data[13];
        res.z = data[2] * p.x + data[6] * p.y + data[10] * p.z + data[14];
        res.intensity = p.intensity; // Intensity usually doesn't change with geometric transform
        return res;
    }
};

#endif // CUDA_POINT_TYPES_CUH
