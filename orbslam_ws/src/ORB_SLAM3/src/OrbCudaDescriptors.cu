// Optional descriptor sampling for ORB-SLAM3. Build via integration/run_orb_slam_gpu.py.
// Detection/orientation/blur and all SLAM optimization remain on the CPU.
#include <cuda_runtime.h>
#include <cstddef>

namespace {
struct Buffers {
    unsigned char *image = nullptr, *descriptors = nullptr;
    float* points = nullptr;
    int* pattern = nullptr;
    size_t image_capacity = 0, point_capacity = 0;
    ~Buffers() { cudaFree(image); cudaFree(descriptors); cudaFree(points); cudaFree(pattern); }
};

__global__ void describe(const unsigned char* image, int width, const float* points,
                         int count, const int* pattern, unsigned char* output)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= count * 32) return;
    const float* point = points + (i / 32) * 4;
    const int* pairs = pattern + (i % 32) * 32;
    int center = int(point[1]) * width + int(point[0]);
    unsigned char value = 0;
    for (int bit = 0; bit < 8; ++bit) {
        const int* p = pairs + bit * 4;
        int y0 = __float2int_rn(p[0] * point[3] + p[1] * point[2]);
        int x0 = __float2int_rn(p[0] * point[2] - p[1] * point[3]);
        int y1 = __float2int_rn(p[2] * point[3] + p[3] * point[2]);
        int x1 = __float2int_rn(p[2] * point[2] - p[3] * point[3]);
        value |= (image[center + y0 * width + x0] < image[center + y1 * width + x1]) << bit;
    }
    output[i] = value;
}
}

extern "C" const char* orb_cuda_probe()
{
    int count = 0;
    cudaError_t error = cudaGetDeviceCount(&count);
    if (error != cudaSuccess) return cudaGetErrorString(error);
    return count ? nullptr : "no CUDA devices";
}

extern "C" const char* orb_cuda_descriptors(const unsigned char* image, int rows, int cols,
    size_t stride, const float* points, int count, const int* pattern, unsigned char* output)
{
    if (!image || !points || !pattern || !output || rows <= 0 || cols <= 0 || count <= 0 || stride < size_t(cols))
        return "invalid descriptor inputs";
    // Per extraction thread: stereo extractors cannot overwrite each other's buffers.
    static thread_local Buffers buffers;
    cudaError_t error;
#define CUDA_CHECK(call) if ((error = (call)) != cudaSuccess) return cudaGetErrorString(error)
    if (size_t(rows) * cols > buffers.image_capacity) {
        CUDA_CHECK(cudaFree(buffers.image)); buffers.image = nullptr; buffers.image_capacity = 0;
        CUDA_CHECK(cudaMalloc(&buffers.image, size_t(rows) * cols));
        buffers.image_capacity = size_t(rows) * cols;
    }
    if (size_t(count) > buffers.point_capacity) {
        CUDA_CHECK(cudaFree(buffers.points)); buffers.points = nullptr; buffers.point_capacity = 0;
        CUDA_CHECK(cudaFree(buffers.descriptors)); buffers.descriptors = nullptr;
        CUDA_CHECK(cudaMalloc(&buffers.points, size_t(count) * 4 * sizeof(float)));
        CUDA_CHECK(cudaMalloc(&buffers.descriptors, size_t(count) * 32));
        buffers.point_capacity = count;
    }
    if (!buffers.pattern) CUDA_CHECK(cudaMalloc(&buffers.pattern, 1024 * sizeof(int)));
    CUDA_CHECK(cudaMemcpy2D(buffers.image, cols, image, stride, cols, rows, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(buffers.points, points, size_t(count) * 4 * sizeof(float), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(buffers.pattern, pattern, 1024 * sizeof(int), cudaMemcpyHostToDevice));
    describe<<<(count * 32 + 127) / 128, 128>>>(buffers.image, cols, buffers.points, count, buffers.pattern, buffers.descriptors);
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaMemcpy(output, buffers.descriptors, size_t(count) * 32, cudaMemcpyDeviceToHost));
    return nullptr;
#undef CUDA_CHECK
}
