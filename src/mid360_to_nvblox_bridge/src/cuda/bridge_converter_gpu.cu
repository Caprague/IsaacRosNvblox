// SPDX-License-Identifier: Apache-2.0
// CUDA Implementation for Mid360 Bridge - Host-side wrapper

#include "mid360_to_nvblox_bridge/cuda/bridge_converter_gpu.hpp"
#include "mid360_to_nvblox_bridge/cuda/bridge_kernels.cuh"
#include <cuda_runtime.h>
#include <stdexcept>
#include <string>

namespace mid360_bridge {
namespace cuda {

// Helper macro for CUDA error checking
#define CUDA_CHECK(call) \
  do { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
      throw std::runtime_error( \
        std::string("CUDA error at ") + __FILE__ + ":" + \
        std::to_string(__LINE__) + " - " + \
        cudaGetErrorString(err)); \
    } \
  } while(0)

// ============================================================================
// GPU Memory Manager (RAII wrapper)
// ============================================================================
template<typename T>
class DeviceBuffer {
public:
  DeviceBuffer() : data_(nullptr), size_(0), capacity_(0) {}

  explicit DeviceBuffer(size_t size) : data_(nullptr), size_(0), capacity_(0) {
    resize(size);
  }

  ~DeviceBuffer() {
    if (data_) {
      cudaFree(data_);
    }
  }

  DeviceBuffer(DeviceBuffer&& other) noexcept
    : data_(other.data_), size_(other.size_), capacity_(other.capacity_) {
    other.data_ = nullptr;
    other.size_ = 0;
    other.capacity_ = 0;
  }

  DeviceBuffer& operator=(DeviceBuffer&& other) noexcept {
    if (this != &other) {
      if (data_) cudaFree(data_);
      data_ = other.data_;
      size_ = other.size_;
      capacity_ = other.capacity_;
      other.data_ = nullptr;
      other.size_ = 0;
      other.capacity_ = 0;
    }
    return *this;
  }

  DeviceBuffer(const DeviceBuffer&) = delete;
  DeviceBuffer& operator=(const DeviceBuffer&) = delete;

  void resize(size_t new_size) {
    if (new_size > capacity_) {
      if (data_) {
        cudaFree(data_);
      }
      CUDA_CHECK(cudaMalloc(&data_, new_size * sizeof(T)));
      capacity_ = new_size;
    }
    size_ = new_size;
  }

  void setZero() {
    if (data_ && size_ > 0) {
      CUDA_CHECK(cudaMemset(data_, 0, size_ * sizeof(T)));
    }
  }

  void copyFromHost(const T* host_data, size_t count) {
    resize(count);
    CUDA_CHECK(cudaMemcpy(data_, host_data, count * sizeof(T),
                          cudaMemcpyHostToDevice));
  }

  void copyToHost(T* host_data, size_t count) const {
    CUDA_CHECK(cudaMemcpy(host_data, data_, count * sizeof(T),
                          cudaMemcpyDeviceToHost));
  }

  T* data() { return data_; }
  const T* data() const { return data_; }
  size_t size() const { return size_; }

private:
  T* data_;
  size_t size_;
  size_t capacity_;
};

// ============================================================================
// BridgeConverterGPU::Impl - holds all CUDA-dependent members
// ============================================================================
struct BridgeConverterGPU::Impl {
  VirtualLidarConfig config;
  size_t grid_size;

  DeviceBuffer<float> points_x_buffer;
  DeviceBuffer<float> points_y_buffer;
  DeviceBuffer<float> points_z_buffer;
  DeviceBuffer<GridCellGPU> grid_buffer;
  DeviceBuffer<GridCellGPU> grid_buffer_temp;
  DeviceBuffer<int> valid_count_buffer;

  cudaStream_t stream;

  explicit Impl(const VirtualLidarConfig& cfg)
    : config(cfg),
      grid_size(static_cast<size_t>(cfg.width) * cfg.height),
      grid_buffer(grid_size),
      grid_buffer_temp(grid_size),
      valid_count_buffer(1)
  {
    CUDA_CHECK(cudaStreamCreate(&stream));
  }

  ~Impl() {
    cudaStreamDestroy(stream);
  }

  void fillHoles(int iterations) {
    for (int i = 0; i < iterations; ++i) {
      dim3 block(16, 16);
      dim3 grid(
          (config.width + block.x - 1) / block.x,
          (config.height + block.y - 1) / block.y);

      fillHolesKernel<<<grid, block, 0, stream>>>(
          grid_buffer.data(), grid_buffer_temp.data(),
          config.width, config.height);

      // Swap buffers
      std::swap(grid_buffer, grid_buffer_temp);
    }
  }
};

// ============================================================================
// BridgeConverterGPU implementation
// ============================================================================
BridgeConverterGPU::BridgeConverterGPU(const VirtualLidarConfig& config)
  : impl_(std::make_unique<Impl>(config))
{
}

BridgeConverterGPU::~BridgeConverterGPU() = default;

void BridgeConverterGPU::convertPointcloud(
    const std::vector<float>& input_x,
    const std::vector<float>& input_y,
    const std::vector<float>& input_z,
    std::vector<float>& output_x,
    std::vector<float>& output_y,
    std::vector<float>& output_z,
    AggregationMethod aggregation,
    int hole_fill_iterations)
{
  const int num_points = static_cast<int>(input_x.size());

  // Upload point data
  impl_->points_x_buffer.copyFromHost(input_x.data(), num_points);
  impl_->points_y_buffer.copyFromHost(input_y.data(), num_points);
  impl_->points_z_buffer.copyFromHost(input_z.data(), num_points);

  // Reset grid
  impl_->grid_buffer.setZero();
  impl_->valid_count_buffer.setZero();

  // Kernel launch config
  const int block_size = 256;
  const int grid_dim = (num_points + block_size - 1) / block_size;

  // Dispatch to appropriate kernel based on aggregation method
  if (aggregation == AggregationMethod::MEAN) {
    pointsToGridKernel<<<grid_dim, block_size, 0, impl_->stream>>>(
        impl_->points_x_buffer.data(), impl_->points_y_buffer.data(),
        impl_->points_z_buffer.data(),
        num_points, impl_->config, impl_->grid_buffer.data(),
        impl_->valid_count_buffer.data());
  } else {
    pointsToGridMinMaxKernel<<<grid_dim, block_size, 0, impl_->stream>>>(
        impl_->points_x_buffer.data(), impl_->points_y_buffer.data(),
        impl_->points_z_buffer.data(),
        num_points, impl_->config, impl_->grid_buffer.data(),
        aggregation);
  }

  // Launch aggregation kernel
  const int agg_grid_dim = (static_cast<int>(impl_->grid_size) + block_size - 1) / block_size;
  aggregateGridKernel<<<agg_grid_dim, block_size, 0, impl_->stream>>>(
      impl_->grid_buffer.data(), static_cast<int>(impl_->grid_size), aggregation);

  // Fill holes
  impl_->fillHoles(hole_fill_iterations);

  // Reconstruct pointcloud
  dim3 recon_block(16, 16);
  dim3 recon_grid(
      (impl_->config.width + recon_block.x - 1) / recon_block.x,
      (impl_->config.height + recon_block.y - 1) / recon_block.y);

  // Output buffers
  output_x.resize(impl_->grid_size);
  output_y.resize(impl_->grid_size);
  output_z.resize(impl_->grid_size);

  DeviceBuffer<float> out_x_buf(impl_->grid_size);
  DeviceBuffer<float> out_y_buf(impl_->grid_size);
  DeviceBuffer<float> out_z_buf(impl_->grid_size);

  reconstructPointcloudKernel<<<recon_grid, recon_block, 0, impl_->stream>>>(
      impl_->grid_buffer.data(), impl_->config,
      out_x_buf.data(), out_y_buf.data(), out_z_buf.data());

  CUDA_CHECK(cudaStreamSynchronize(impl_->stream));

  // Download results
  out_x_buf.copyToHost(output_x.data(), impl_->grid_size);
  out_y_buf.copyToHost(output_y.data(), impl_->grid_size);
  out_z_buf.copyToHost(output_z.data(), impl_->grid_size);
}

int BridgeConverterGPU::getValidPointCount()
{
  int count = 0;
  impl_->valid_count_buffer.copyToHost(&count, 1);
  return count;
}

// ============================================================================
// Utility functions
// ============================================================================
bool isCudaAvailable()
{
  int device_count = 0;
  cudaError_t err = cudaGetDeviceCount(&device_count);
  return (err == cudaSuccess && device_count > 0);
}

std::string getCudaDeviceInfo()
{
  cudaDeviceProp prop;
  int device = 0;
  cudaError_t err = cudaGetDevice(&device);
  if (err != cudaSuccess) {
    return "CUDA device not available";
  }
  err = cudaGetDeviceProperties(&prop, device);
  if (err != cudaSuccess) {
    return "Failed to get device properties";
  }
  return std::string(prop.name) + " | SM " +
         std::to_string(prop.major) + "." + std::to_string(prop.minor) +
         " | " + std::to_string(prop.totalGlobalMem / (1024*1024)) + " MB";
}

} // namespace cuda
} // namespace mid360_bridge
