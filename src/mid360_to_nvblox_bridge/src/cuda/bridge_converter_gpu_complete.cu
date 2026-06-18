// SPDX-License-Identifier: Apache-2.0
// CUDA Implementation - COMPLETE with Async Stream Support

#include "mid360_to_nvblox_bridge/cuda/bridge_kernels.cuh"
#include <cuda_runtime.h>
#include <vector>
#include <memory>
#include <stdexcept>

namespace mid360_bridge {
namespace cuda {

// ============================================================================
// Enhanced GPU Memory Manager with Async Support
// ============================================================================
template<typename T>
class DeviceBuffer {
public:
  DeviceBuffer() : data_(nullptr), size_(0), capacity_(0) {}
  
  explicit DeviceBuffer(size_t size) : size_(0), capacity_(0) {
    resize(size);
  }
  
  ~DeviceBuffer() {
    if (data_) {
      cudaFree(data_);
    }
  }
  
  DeviceBuffer(const DeviceBuffer&) = delete;
  DeviceBuffer& operator=(const DeviceBuffer&) = delete;
  
  void resize(size_t new_size) {
    if (new_size > capacity_) {
      if (data_) {
        cudaFree(data_);
      }
      checkCudaErrors(cudaMalloc(&data_, new_size * sizeof(T)));
      capacity_ = new_size;
    }
    size_ = new_size;
  }
  
  // ✅ 异步版本 - 使用stream
  void setZeroAsync(cudaStream_t stream) {
    if (data_ && size_ > 0) {
      checkCudaErrors(cudaMemsetAsync(data_, 0, size_ * sizeof(T), stream));
    }
  }
  
  // ✅ 异步拷贝 Host→Device
  void copyFromHostAsync(const T* host_data, size_t count, cudaStream_t stream) {
    resize(count);
    checkCudaErrors(cudaMemcpyAsync(data_, host_data, count * sizeof(T), 
                                   cudaMemcpyHostToDevice, stream));
  }
  
  // ✅ 异步拷贝 Device→Host
  void copyToHostAsync(T* host_data, size_t count, cudaStream_t stream) const {
    checkCudaErrors(cudaMemcpyAsync(host_data, data_, count * sizeof(T), 
                                   cudaMemcpyDeviceToHost, stream));
  }
  
  T* data() { return data_; }
  const T* data() const { return data_; }
  size_t size() const { return size_; }
  
private:
  T* data_;
  size_t size_;
  size_t capacity_;
  
  void checkCudaErrors(cudaError_t err) {
    if (err != cudaSuccess) {
      throw std::runtime_error(
        std::string("CUDA error: ") + cudaGetErrorString(err));
    }
  }
};

// ============================================================================
// Pinned Memory Buffer for Fast Transfers
// ============================================================================
template<typename T>
class PinnedHostBuffer {
public:
  PinnedHostBuffer() : data_(nullptr), capacity_(0) {}
  
  explicit PinnedHostBuffer(size_t size) : data_(nullptr), capacity_(0) {
    resize(size);
  }
  
  ~PinnedHostBuffer() {
    if (data_) {
      cudaFreeHost(data_);
    }
  }
  
  void resize(size_t new_size) {
    if (new_size > capacity_) {
      if (data_) {
        cudaFreeHost(data_);
      }
      checkCudaErrors(cudaMallocHost(&data_, new_size * sizeof(T)));
      capacity_ = new_size;
    }
  }
  
  T* data() { return data_; }
  const T* data() const { return data_; }
  
private:
  T* data_;
  size_t capacity_;
  
  void checkCudaErrors(cudaError_t err) {
    if (err != cudaSuccess) {
      throw std::runtime_error(
        std::string("CUDA error: ") + cudaGetErrorString(err));
    }
  }
};

// ============================================================================
// Complete GPU Converter Implementation with Async Stream
// ============================================================================
class BridgeConverterGPU {
public:
  explicit BridgeConverterGPU(const VirtualLidarConfig& config)
    : config_(config)
    , grid_size_(config.width * config.height)
    , stream_(nullptr)
  {
    // ✅ 创建独立CUDA Stream
    checkCudaErrors(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking));
    
    // 预分配GPU buffer
    grid_buffer_.resize(grid_size_);
    grid_buffer_temp_.resize(grid_size_);
    valid_count_buffer_.resize(1);
    
    // 预分配typical大小（会自动增长）
    const size_t initial_capacity = 100000;
    points_x_buffer_.resize(initial_capacity);
    points_y_buffer_.resize(initial_capacity);
    points_z_buffer_.resize(initial_capacity);
    
    // ✅ 预分配Pinned Memory for faster transfers
    pinned_input_x_.resize(initial_capacity);
    pinned_input_y_.resize(initial_capacity);
    pinned_input_z_.resize(initial_capacity);
    pinned_output_x_.resize(grid_size_);
    pinned_output_y_.resize(grid_size_);
    pinned_output_z_.resize(grid_size_);
  }
  
  ~BridgeConverterGPU() {
    if (stream_) {
      cudaStreamSynchronize(stream_);
      cudaStreamDestroy(stream_);
    }
  }
  
  // ✅ 完整的异步转换实现
  void convertPointcloud(
      const std::vector<float>& input_x,
      const std::vector<float>& input_y,
      const std::vector<float>& input_z,
      std::vector<float>& output_x,
      std::vector<float>& output_y,
      std::vector<float>& output_z,
      AggregationMethod aggregation = AggregationMethod::MEAN,
      int hole_fill_iterations = 2)
  {
    const size_t num_input_points = input_x.size();
    
    // ===== Stage 1: Async Upload to GPU =====
    // ✅ 先拷贝到pinned memory（可以并行进行）
    if (pinned_input_x_.capacity() < num_input_points) {
      pinned_input_x_.resize(num_input_points);
      pinned_input_y_.resize(num_input_points);
      pinned_input_z_.resize(num_input_points);
    }
    
    std::copy(input_x.begin(), input_x.end(), pinned_input_x_.data());
    std::copy(input_y.begin(), input_y.end(), pinned_input_y_.data());
    std::copy(input_z.begin(), input_z.end(), pinned_input_z_.data());
    
    // ✅ 异步上传（3个拷贝可以pipeline）
    points_x_buffer_.copyFromHostAsync(pinned_input_x_.data(), num_input_points, stream_);
    points_y_buffer_.copyFromHostAsync(pinned_input_y_.data(), num_input_points, stream_);
    points_z_buffer_.copyFromHostAsync(pinned_input_z_.data(), num_input_points, stream_);
    
    // ===== Stage 2: Clear Grid (Async) =====
    grid_buffer_.setZeroAsync(stream_);
    valid_count_buffer_.setZeroAsync(stream_);
    
    // ===== Stage 3: Point-to-Grid Kernel (Async) =====
    const int threads_per_block = 256;
    const int num_blocks = (num_input_points + threads_per_block - 1) / threads_per_block;
    
    // ✅ Kernel在stream中异步执行
    pointsToGridKernel<<<num_blocks, threads_per_block, 0, stream_>>>(
        points_x_buffer_.data(),
        points_y_buffer_.data(),
        points_z_buffer_.data(),
        num_input_points,
        config_,
        grid_buffer_.data(),
        valid_count_buffer_.data()
    );
    checkCudaErrors(cudaGetLastError());
    
    // ===== Stage 4: Aggregation Kernel (Async) =====
    const int grid_blocks = (grid_size_ + threads_per_block - 1) / threads_per_block;
    
    aggregateGridKernel<<<grid_blocks, threads_per_block, 0, stream_>>>(
        grid_buffer_.data(),
        grid_size_,
        aggregation
    );
    checkCudaErrors(cudaGetLastError());
    
    // ===== Stage 5: Hole Filling (Async, Iterative) =====
    if (hole_fill_iterations > 0) {
      fillHoles(hole_fill_iterations);
    }
    
    // ===== Stage 6: Reconstruction Kernel (Async) =====
    dim3 block_dim(16, 16);
    dim3 grid_dim(
        (config_.width + block_dim.x - 1) / block_dim.x,
        (config_.height + block_dim.y - 1) / block_dim.y
    );
    
    DeviceBuffer<float> out_x(grid_size_);
    DeviceBuffer<float> out_y(grid_size_);
    DeviceBuffer<float> out_z(grid_size_);
    
    reconstructPointcloudKernel<<<grid_dim, block_dim, 0, stream_>>>(
        grid_buffer_.data(),
        config_,
        out_x.data(),
        out_y.data(),
        out_z.data()
    );
    checkCudaErrors(cudaGetLastError());
    
    // ===== Stage 7: Async Download from GPU =====
    output_x.resize(grid_size_);
    output_y.resize(grid_size_);
    output_z.resize(grid_size_);
    
    // ✅ 异步下载（3个拷贝可以pipeline）
    out_x.copyToHostAsync(pinned_output_x_.data(), grid_size_, stream_);
    out_y.copyToHostAsync(pinned_output_y_.data(), grid_size_, stream_);
    out_z.copyToHostAsync(pinned_output_z_.data(), grid_size_, stream_);
    
    // ✅ 等待所有异步操作完成
    checkCudaErrors(cudaStreamSynchronize(stream_));
    
    // 从pinned memory拷贝到output vector
    std::copy(pinned_output_x_.data(), pinned_output_x_.data() + grid_size_, output_x.begin());
    std::copy(pinned_output_y_.data(), pinned_output_y_.data() + grid_size_, output_y.begin());
    std::copy(pinned_output_z_.data(), pinned_output_z_.data() + grid_size_, output_z.begin());
  }
  
  int getValidPointCount() {
    int count = 0;
    checkCudaErrors(cudaMemcpyAsync(&count, valid_count_buffer_.data(), 
                                   sizeof(int), cudaMemcpyDeviceToHost, stream_));
    checkCudaErrors(cudaStreamSynchronize(stream_));
    return count;
  }
  
private:
  // ✅ Ping-pong hole filling with async
  void fillHoles(int iterations) {
    dim3 block_dim(16, 16);
    dim3 grid_dim(
        (config_.width + block_dim.x - 1) / block_dim.x,
        (config_.height + block_dim.y - 1) / block_dim.y
    );
    
    GridCellGPU* src = grid_buffer_.data();
    GridCellGPU* dst = grid_buffer_temp_.data();
    
    for (int iter = 0; iter < iterations; ++iter) {
      // ✅ 在stream中异步执行
      fillHolesKernel<<<grid_dim, block_dim, 0, stream_>>>(
          src, dst, config_.width, config_.height
      );
      checkCudaErrors(cudaGetLastError());
      
      std::swap(src, dst);
    }
    
    // 确保最终结果在grid_buffer_中
    if (iterations % 2 == 1) {
      checkCudaErrors(cudaMemcpyAsync(
          grid_buffer_.data(),
          grid_buffer_temp_.data(),
          grid_size_ * sizeof(GridCellGPU),
          cudaMemcpyDeviceToDevice,
          stream_
      ));
    }
  }
  
  void checkCudaErrors(cudaError_t err) {
    if (err != cudaSuccess) {
      throw std::runtime_error(
        std::string("CUDA error: ") + cudaGetErrorString(err));
    }
  }
  
  VirtualLidarConfig config_;
  size_t grid_size_;
  
  // GPU buffers
  DeviceBuffer<float> points_x_buffer_;
  DeviceBuffer<float> points_y_buffer_;
  DeviceBuffer<float> points_z_buffer_;
  DeviceBuffer<GridCellGPU> grid_buffer_;
  DeviceBuffer<GridCellGPU> grid_buffer_temp_;
  DeviceBuffer<int> valid_count_buffer_;
  
  // ✅ Pinned host buffers for faster transfers
  PinnedHostBuffer<float> pinned_input_x_;
  PinnedHostBuffer<float> pinned_input_y_;
  PinnedHostBuffer<float> pinned_input_z_;
  PinnedHostBuffer<float> pinned_output_x_;
  PinnedHostBuffer<float> pinned_output_y_;
  PinnedHostBuffer<float> pinned_output_z_;
  
  // ✅ 独立CUDA Stream
  cudaStream_t stream_;
};

// ============================================================================
// Utility Functions
// ============================================================================
inline bool isCudaAvailable() {
  int device_count = 0;
  cudaError_t err = cudaGetDeviceCount(&device_count);
  return (err == cudaSuccess && device_count > 0);
}

inline std::string getCudaDeviceInfo() {
  int device;
  cudaGetDevice(&device);
  
  cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, device);
  
  return std::string("CUDA Device: ") + prop.name + 
         ", Compute: " + std::to_string(prop.major) + "." + std::to_string(prop.minor) +
         ", Memory: " + std::to_string(prop.totalGlobalMem / (1024*1024)) + " MB" +
         ", Async Engines: " + std::to_string(prop.asyncEngineCount);
}

} // namespace cuda
} // namespace mid360_bridge
