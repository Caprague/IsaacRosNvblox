#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <cmath>
#include <thread>
#include <mutex>

using namespace std::chrono_literals;

class ImuHorizontalAlignNode : public rclcpp::Node
{
public:
  ImuHorizontalAlignNode() : Node("imu_horizontal_align_node")
  {
    RCLCPP_INFO(this->get_logger(), "IMU Horizontal Alignment Node is starting...");
    
    // 声明并获取参数
    this->declare_parameter<double>("start_delay", 0.0);
    this->declare_parameter<std::string>("imu_topic", "/camera/imu");   // 相机IMU话题
    this->declare_parameter<std::string>("base_imu_topic", "");         // 底盘IMU话题，空则禁用双IMU标定
    this->declare_parameter<double>("transform_publish_rate", 100.0);
    this->declare_parameter<bool>("use_static_tf_broadcaster", false);  // true=发布/tf_static，false=按频率发布/tf
    this->declare_parameter<double>("camera_base_x", 0.34);             // 相机在底盘坐标系中的平移X
    this->declare_parameter<double>("camera_base_y", 0.0);              // 相机在底盘坐标系中的平移Y
    this->declare_parameter<double>("camera_base_z", 0.09);             // 相机在底盘坐标系中的平移Z
    
    this->start_delay_ = this->get_parameter("start_delay").as_double();
    this->imu_topic_ = this->get_parameter("imu_topic").as_string();
    this->base_imu_topic_ = this->get_parameter("base_imu_topic").as_string();
    this->transform_publish_rate_ = this->get_parameter("transform_publish_rate").as_double();
    this->use_static_tf_broadcaster_ = this->get_parameter("use_static_tf_broadcaster").as_bool();
    this->camera_base_translation_ = Eigen::Vector3d(
      this->get_parameter("camera_base_x").as_double(),
      this->get_parameter("camera_base_y").as_double(),
      this->get_parameter("camera_base_z").as_double()
    );
    
    // IMU数据收集参数
    this->collecting_data_ = false;
    this->collection_duration_ = 3.0;  // 收集3秒数据
    this->min_valid_samples_ = 100;
    
    // 初始化对齐四元数
    this->alignment_quaternion_.setIdentity();
    this->alignment_quaternion_set_ = false;
    
    // 初始化相机-底盘相对旋转四元数
    this->camera_base_quaternion_.setIdentity();
    this->camera_base_quaternion_set_ = false;
    
    // 创建TF广播器
    if (this->use_static_tf_broadcaster_) {
      this->static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
      RCLCPP_INFO(this->get_logger(), "Using static TF broadcaster (/tf_static)");
    } else {
      this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      RCLCPP_INFO(this->get_logger(), "Using dynamic TF broadcaster (/tf), publish rate: %.2f Hz", this->transform_publish_rate_);
    }
    
    // 订阅相机IMU数据
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    this->imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        this->imu_topic_,
        qos,
        std::bind(&ImuHorizontalAlignNode::cameraImuCallback, this, std::placeholders::_1)
    );
    
    // 订阅底盘IMU数据（如果配置了话题）
    if (!this->base_imu_topic_.empty()) {
      this->base_imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
          this->base_imu_topic_,
          qos,
          std::bind(&ImuHorizontalAlignNode::baseImuCallback, this, std::placeholders::_1)
      );
      RCLCPP_INFO(this->get_logger(), "Dual-IMU mode enabled, base IMU topic: %s", this->base_imu_topic_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Dual-IMU mode disabled (base_imu_topic not set), using hardcoded camera-base rotation");
    }
    
    // 延时启动，避免在IMU数据稳定前计算
    RCLCPP_INFO(this->get_logger(), "Delaying IMU horizontal alignment by %.2f seconds...", this->start_delay_);
    std::this_thread::sleep_for(std::chrono::duration<double>(this->start_delay_));
    
    // 开始收集数据
    this->start_data_collection();
  }
  
private:
  void start_data_collection()
  {
    this->collecting_data_ = true;
    RCLCPP_INFO(this->get_logger(), "Starting to collect IMU data for %.2f seconds...", this->collection_duration_);
    this->start_time_ = this->get_clock()->now();
    
    // 创建定时器来检查数据收集是否完成
    this->timer_ = this->create_wall_timer(
        100ms,
        std::bind(&ImuHorizontalAlignNode::timer_callback, this)
    );
  }
  
  void cameraImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (this->collecting_data_)
    {
      Eigen::Vector3d accel;
      accel[0] = msg->linear_acceleration.x;
      accel[1] = msg->linear_acceleration.y;
      accel[2] = msg->linear_acceleration.z;
      std::lock_guard<std::mutex> lock(this->samples_mutex_);
      this->camera_accel_samples_.push_back(accel);
    }
  }
  
  void baseImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (this->collecting_data_)
    {
      Eigen::Vector3d accel;
      accel[0] = msg->linear_acceleration.x;
      accel[1] = msg->linear_acceleration.y;
      accel[2] = msg->linear_acceleration.z;
      std::lock_guard<std::mutex> lock(this->samples_mutex_);
      this->base_accel_samples_.push_back(accel);
    }
  }
  
  void timer_callback()
  {
    // 检查是否收集了足够的时间
    auto elapsed_time = (this->get_clock()->now() - this->start_time_).seconds();
    if (this->collecting_data_ && elapsed_time > this->collection_duration_)
    {
      this->collecting_data_ = false;
      
      size_t cam_count, base_count;
      {
        std::lock_guard<std::mutex> lock(this->samples_mutex_);
        cam_count = this->camera_accel_samples_.size();
        base_count = this->base_accel_samples_.size();
      }
      RCLCPP_INFO(this->get_logger(), "Collected %zu camera IMU samples, %zu base IMU samples",
                  cam_count, base_count);
      
      // 处理收集的数据
      this->process_imu_data();
      
      // 停止数据收集定时器
      this->timer_->cancel();
    }
  }

  void process_imu_data()
  {
      std::vector<Eigen::Vector3d> camera_samples, base_samples;
      {
        std::lock_guard<std::mutex> lock(this->samples_mutex_);
        camera_samples = this->camera_accel_samples_;
        base_samples = this->base_accel_samples_;
      }
      
      if (camera_samples.size() < this->min_valid_samples_)
      {
          RCLCPP_ERROR(this->get_logger(), "Not enough camera IMU samples collected: %zu", camera_samples.size());
          return;
      }
      
      // 过滤相机IMU异常值（只保留接近重力加速度的样本）
      std::vector<Eigen::Vector3d> valid_camera_samples;
      for (const auto& sample : camera_samples)
      {
          double norm = sample.norm();
          if (norm >= 9.0 && norm <= 10.0)
          {
              valid_camera_samples.push_back(sample);
          }
      }
      
      if (valid_camera_samples.size() < this->min_valid_samples_)
      {
          RCLCPP_ERROR(this->get_logger(), "Not enough valid camera IMU samples after filtering: %zu", valid_camera_samples.size());
          return;
      }
      
      // 计算相机平均加速度向量
      Eigen::Vector3d avg_camera_accel = Eigen::Vector3d::Zero();
      for (const auto& sample : valid_camera_samples)
      {
          avg_camera_accel += sample;
      }
      avg_camera_accel /= valid_camera_samples.size();
      
      // 归一化平均加速度向量，得到重力方向
      Eigen::Vector3d measured_gravity = avg_camera_accel.normalized();
      
      RCLCPP_INFO(this->get_logger(), "Camera average accelerometer: [%.3f, %.3f, %.3f]", 
                  avg_camera_accel[0], avg_camera_accel[1], avg_camera_accel[2]);
      RCLCPP_INFO(this->get_logger(), "Camera measured gravity direction: [%.3f, %.3f, %.3f]", 
                  measured_gravity[0], measured_gravity[1], measured_gravity[2]);
      
      // Realsense D435i相机坐标系定义：
      // X轴: 向右
      // Y轴: 向下  
      // Z轴: 向前 (从相机视角看)
      
      // 从重力向量计算相机相对于水平面的倾斜
      double roll_imu = std::atan2(measured_gravity[1], measured_gravity[2]); // 绕X轴的倾斜
      double pitch_imu = std::atan2(-measured_gravity[0], 
                                    std::sqrt(measured_gravity[1]*measured_gravity[1] + 
                                            measured_gravity[2]*measured_gravity[2])); // 绕Y轴的倾斜
      
      RCLCPP_INFO(this->get_logger(), "Calculated IMU roll: %.2f°, pitch: %.2f°", 
                  roll_imu * 180.0 / M_PI, pitch_imu * 180.0 / M_PI);
      
      // 计算从测量重力方向到目标重力方向的旋转
      Eigen::Vector3d target_gravity(0.0, -1.0, 0.0);
      
      Eigen::Vector3d rotation_axis = measured_gravity.cross(target_gravity);
      double rotation_axis_norm = rotation_axis.norm();
      
      Eigen::Quaterniond q_temp;
      
      if (rotation_axis_norm < 1e-6) {
          if (measured_gravity.dot(target_gravity) > 0) {
              q_temp.setIdentity();
              RCLCPP_INFO(this->get_logger(), "IMU already aligned with target gravity direction");
          } else {
              q_temp = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
              RCLCPP_WARN(this->get_logger(), "IMU is upside down, applying 180-degree rotation around Z axis");
          }
      } else {
          rotation_axis.normalize();
          double cos_angle = measured_gravity.dot(target_gravity);
          cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
          double rotation_angle = std::acos(cos_angle);
          
          q_temp = Eigen::AngleAxisd(rotation_angle, rotation_axis);
      }
      
      // 获取临时四元数的欧拉角
      Eigen::Vector3d euler_angles = q_temp.toRotationMatrix().eulerAngles(0, 1, 2); // RPY
      double roll_deg = euler_angles[0] * 180.0 / M_PI;
      double pitch_deg = euler_angles[1] * 180.0 / M_PI;
      double yaw_deg = euler_angles[2] * 180.0 / M_PI;
      
      RCLCPP_INFO(this->get_logger(), "Original alignment quaternion - RPY: [%.2f°, %.2f°, %.2f°]", 
                  roll_deg, pitch_deg, yaw_deg);
      
      // 交换roll和pitch角度值，然后重新构建四元数
      double corrected_roll_deg = yaw_deg;
      double corrected_pitch_deg = (roll_imu * 180.0 / M_PI) < -90.0 ? roll_deg:(360.0-roll_deg);
      double corrected_yaw_deg = pitch_deg;
      
      RCLCPP_INFO(this->get_logger(), "Corrected RPY after swap - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", 
                  corrected_roll_deg, corrected_pitch_deg, corrected_yaw_deg);
      
      // 将角度转换回弧度
      double corrected_roll_rad = corrected_roll_deg * M_PI / 180.0;
      double corrected_pitch_rad = corrected_pitch_deg * M_PI / 180.0;
      double corrected_yaw_rad = corrected_yaw_deg * M_PI / 180.0;
      
      // 从修正后的欧拉角创建新的四元数
      Eigen::AngleAxisd rollAngle(corrected_roll_rad, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(corrected_pitch_rad, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(corrected_yaw_rad, Eigen::Vector3d::UnitZ());
      
      Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle; // ZYX顺序
      
      // 验证最终的四元数
      Eigen::Vector3d final_euler_angles = q.toRotationMatrix().eulerAngles(0, 1, 2); // RPY
      double final_roll_deg = final_euler_angles[0] * 180.0 / M_PI;
      double final_pitch_deg = final_euler_angles[1] * 180.0 / M_PI;
      double final_yaw_deg = final_euler_angles[2] * 180.0 / M_PI;
      
      RCLCPP_INFO(this->get_logger(), "Final alignment quaternion after swap - RPY: [%.2f°, %.2f°, %.2f°]", 
                  final_roll_deg, final_pitch_deg, final_yaw_deg);
      
      // 验证校正效果：应用旋转后重力方向应该接近期望值
      Eigen::Vector3d corrected_gravity = q * measured_gravity;
      RCLCPP_INFO(this->get_logger(), "After alignment, gravity direction will be: [%.3f, %.3f, %.3f]", 
                  corrected_gravity[0], corrected_gravity[1], corrected_gravity[2]);
      
      // 检查Z轴方向（相机前方方向），确保它在水平面上
      Eigen::Vector3d original_z_axis(0.0, 0.0, 1.0);
      Eigen::Vector3d transformed_z_axis = q * original_z_axis;
      RCLCPP_INFO(this->get_logger(), "After alignment, Z axis direction: [%.3f, %.3f, %.3f]", 
                  transformed_z_axis[0], transformed_z_axis[1], transformed_z_axis[2]);
      
      this->alignment_quaternion_ = q;
      this->alignment_quaternion_set_ = true;
      
      RCLCPP_INFO(this->get_logger(), "IMU alignment completed successfully with RPY swap correction");
      
      // ---- 双IMU: 计算相机到底盘的相对旋转 ----
      if (!this->base_imu_topic_.empty()) {
          computeCameraBaseRotation(measured_gravity, base_samples);
      }
      
      // 创建定时器定期发布变换
      this->create_transform_publish_timer();
  }
  
  /// 双IMU重力对齐：计算 camera_link → base_link 旋转
  ///
  /// 原理：两个IMU测量同一个物理重力向量，但表达在各自的坐标系中。
  ///   g_base = R_base_cam * g_cam
  /// 使用Triad方法，以重力为第一对向量，以"前进方向在水平面的投影"为第二对向量，
  /// 零偏航约束（相机朝前=底盘朝前），可完整求解 R_base_cam。
  ///
  /// 坐标系定义：
  ///   相机(X-right, Y-down, Z-forward) → 相机前进方向 = Z轴
  ///   底盘(X-forward, Y-left, Z-up)    → 底盘前进方向 = X轴
  void computeCameraBaseRotation(const Eigen::Vector3d& g_cam,
                                 const std::vector<Eigen::Vector3d>& base_samples)
  {
      // 过滤底盘IMU异常值
      std::vector<Eigen::Vector3d> valid_base_samples;
      for (const auto& sample : base_samples)
      {
          double norm = sample.norm();
          if (norm >= 9.0 && norm <= 10.0)
          {
              valid_base_samples.push_back(sample);
          }
      }
      
      if (valid_base_samples.size() < this->min_valid_samples_)
      {
          RCLCPP_WARN(this->get_logger(), 
              "Not enough valid base IMU samples: %zu. Falling back to hardcoded camera-base rotation.",
              valid_base_samples.size());
          return;
      }
      
      // 计算底盘平均加速度
      Eigen::Vector3d avg_base_accel = Eigen::Vector3d::Zero();
      for (const auto& sample : valid_base_samples)
      {
          avg_base_accel += sample;
      }
      avg_base_accel /= valid_base_samples.size();
      
      Eigen::Vector3d g_base = avg_base_accel.normalized();
      
      RCLCPP_INFO(this->get_logger(), "Base average accelerometer: [%.3f, %.3f, %.3f]", 
                  avg_base_accel[0], avg_base_accel[1], avg_base_accel[2]);
      RCLCPP_INFO(this->get_logger(), "Base measured gravity direction: [%.3f, %.3f, %.3f]", 
                  g_base[0], g_base[1], g_base[2]);
      
      // ---- Triad方法计算 R_base_cam ----
      
      // 第一对向量：重力方向
      Eigen::Vector3d v1_cam = g_cam;
      Eigen::Vector3d v1_base = g_base;
      
      // 第二对向量：前进方向在重力垂直平面上的投影
      // 相机前进方向 = Z轴 (0,0,1)
      // 底盘前进方向 = X轴 (1,0,0)
      Eigen::Vector3d cam_forward(0.0, 0.0, 1.0);
      Eigen::Vector3d base_forward(1.0, 0.0, 0.0);
      
      // 投影到重力垂直平面
      Eigen::Vector3d v2_cam = cam_forward - cam_forward.dot(v1_cam) * v1_cam;
      Eigen::Vector3d v2_base = base_forward - base_forward.dot(v1_base) * v1_base;
      
      double v2_cam_norm = v2_cam.norm();
      double v2_base_norm = v2_base.norm();
      
      if (v2_cam_norm < 1e-6 || v2_base_norm < 1e-6)
      {
          RCLCPP_WARN(this->get_logger(), 
              "Forward direction too aligned with gravity, cannot compute relative rotation");
          return;
      }
      
      v2_cam.normalize();
      v2_base.normalize();
      
      // 第三对向量：叉积构造正交基
      Eigen::Vector3d v3_cam = v1_cam.cross(v2_cam);
      Eigen::Vector3d v3_base = v1_base.cross(v2_base);
      
      // 构造旋转矩阵: 列向量为正交基
      Eigen::Matrix3d M_cam, M_base;
      M_cam.col(0) = v1_cam;  M_cam.col(1) = v2_cam;  M_cam.col(2) = v3_cam;
      M_base.col(0) = v1_base; M_base.col(1) = v2_base; M_base.col(2) = v3_base;
      
      // R_base_cam: 将相机坐标系向量变换到底盘坐标系
      // v_base = R_base_cam * v_cam
      // v_base = M_base * M_cam^T * v_cam (因为M* e_i = v_i)
      Eigen::Matrix3d R_base_cam = M_base * M_cam.transpose();
      
      // 验证: R_base_cam * g_cam ≈ g_base
      Eigen::Vector3d verify = R_base_cam * g_cam;
      double gravity_error = (verify - g_base).norm();
      RCLCPP_INFO(this->get_logger(), 
          "Dual-IMU rotation verification - R*g_cam: [%.4f, %.4f, %.4f], g_base: [%.4f, %.4f, %.4f], error: %.6f",
          verify[0], verify[1], verify[2], g_base[0], g_base[1], g_base[2], gravity_error);
      
      // 转换为四元数
      Eigen::Quaterniond q_base_cam(R_base_cam);
      q_base_cam.normalize();
      
      // 输出相对旋转的欧拉角
      Eigen::Vector3d rel_euler = R_base_cam.eulerAngles(0, 1, 2);
      RCLCPP_INFO(this->get_logger(), "Camera→Base relative rotation - RPY: [%.2f°, %.2f°, %.2f°]",
                  rel_euler[0] * 180.0 / M_PI, rel_euler[1] * 180.0 / M_PI, rel_euler[2] * 180.0 / M_PI);
      
      // 验证：应用旋转后，相机前进方向在底盘坐标系中应该接近底盘前进方向
      Eigen::Vector3d cam_z_in_base = R_base_cam * cam_forward;
      RCLCPP_INFO(this->get_logger(), "Camera Z-axis in base frame: [%.3f, %.3f, %.3f] (expected ~[1,0,0])",
                  cam_z_in_base[0], cam_z_in_base[1], cam_z_in_base[2]);
      
      this->camera_base_quaternion_ = q_base_cam;
      this->camera_base_quaternion_set_ = true;
      
      RCLCPP_INFO(this->get_logger(), "Dual-IMU camera-base rotation calibration completed successfully");
  }

  void create_transform_publish_timer()
  {
    if (this->use_static_tf_broadcaster_) {
      RCLCPP_INFO(this->get_logger(), "Publishing static transforms once");
      this->publish_transform_callback();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Creating transform publish timer with rate %.2f Hz", this->transform_publish_rate_);
    double publish_period = 1.0 / this->transform_publish_rate_;
    this->transform_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(publish_period),
        std::bind(&ImuHorizontalAlignNode::publish_transform_callback, this)
    );
  }
  
  void publish_transform_callback()
  {
    if (this->alignment_quaternion_set_)
    {
      this->publish_horizontal_transform(this->alignment_quaternion_);
      this->publish_camera_inverse_transform();
      this->publish_lidar_transform();
    }
  }
  
  void send_transform(const geometry_msgs::msg::TransformStamped& transform)
  {
    if (this->use_static_tf_broadcaster_) {
      this->static_tf_broadcaster_->sendTransform(transform);
    } else {
      this->tf_broadcaster_->sendTransform(transform);
    }
  }

  void publish_horizontal_transform(const Eigen::Quaterniond& quaternion)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "odom_horizontal";  // 父坐标系
    transform.child_frame_id = "map";  // 子坐标系
    
    // 平移部分设为零
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    
    // 设置旋转四元数
    transform.transform.rotation.w = quaternion.w();
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    
    // 发布动态变换
    this->send_transform(transform);
  }

  /// 发布 camera_link → base_link 变换
  ///
  /// TF语义: p_camera = R * p_base + t
  ///   R = R_cam_base = R_base_cam^T (从底盘到相机的旋转)
  ///   t = t_cam_base = -R_cam_base * t_base_cam (底盘原点在相机坐标系中的位置)
  ///
  /// 其中 t_base_cam (相机在底盘坐标系中的位置) 由参数 camera_base_x/y/z 提供。
  ///
  /// 双IMU模式: R_base_cam 由重力对齐自动计算
  /// 回退模式: 使用硬编码的旋转四元数
  void publish_camera_inverse_transform()
  {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id = "camera_link";
      transform.child_frame_id = "base_link";
      
      Eigen::Quaterniond q_cam_base;     // R_cam_base: base→camera旋转
      Eigen::Vector3d t_cam_base;        // 底盘原点在相机坐标系中的位置
      
      if (this->camera_base_quaternion_set_)
      {
          // 双IMU模式：使用自动计算的旋转
          Eigen::Quaterniond q_base_cam = this->camera_base_quaternion_;
          q_cam_base = q_base_cam.inverse();  // R_cam_base = R_base_cam^T
          t_cam_base = -(q_cam_base.toRotationMatrix()) * this->camera_base_translation_;
      }
      else
      {
          // 回退模式：先把 camera_link 姿态校正到水平基准（camera_horizontal_link），
          // 再在该水平坐标系下按固定负向位移反推 base_link。
          //
          // alignment_quaternion_ 为 odom_horizontal -> map(=camera_link) 的旋转，
          // 故其逆可视作 camera_link -> camera_horizontal_link 的旋转。
          const Eigen::Quaterniond q_horizontal_camera = this->alignment_quaternion_.inverse();
          q_cam_base = q_horizontal_camera;
          q_cam_base.normalize();

          // 在水平坐标系下，base 相对 camera 的位移取固定值 -camera_base_translation_。
          // 需要表达为 camera_link 坐标下平移：t_cam_base = R_cam_horizontal * t_horizontal_base。
          const Eigen::Vector3d t_horizontal_base = -this->camera_base_translation_;
          t_cam_base = q_cam_base.toRotationMatrix() * t_horizontal_base;

          RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Fallback camera->base horizontal correction: q_cam_base=[w=%.4f, x=%.4f, y=%.4f, z=%.4f], t_cam_base=[%.4f, %.4f, %.4f]",
            q_cam_base.w(), q_cam_base.x(), q_cam_base.y(), q_cam_base.z(),
            t_cam_base.x(), t_cam_base.y(), t_cam_base.z());
      }
      
      transform.transform.translation.x = t_cam_base[0];
      transform.transform.translation.y = t_cam_base[1];
      transform.transform.translation.z = t_cam_base[2];
      
      transform.transform.rotation.w = q_cam_base.w();
      transform.transform.rotation.x = q_cam_base.x();
      transform.transform.rotation.y = q_cam_base.y();
      transform.transform.rotation.z = q_cam_base.z();
      
      this->send_transform(transform);
  }

  void publish_lidar_transform()
  {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id = "base_link";
      transform.child_frame_id = "utlidar_lidar";
      
      // 设置平移：x+0.28945, z-0.04682, 其余为0
      transform.transform.translation.x = 0.28945;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = -0.04682;
      
      // 设置旋转：使用提供的四元数 (w,x,y,z) = (0.13132, 0, 0.99134, 0)
      transform.transform.rotation.w = 0.13132;
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.99134;
      transform.transform.rotation.z = 0.0;
      
      this->send_transform(transform);
  }
  
  // 参数
  double start_delay_;
  std::string imu_topic_;
  std::string base_imu_topic_;
  double transform_publish_rate_;
  bool use_static_tf_broadcaster_;
  Eigen::Vector3d camera_base_translation_;  // 相机在底盘坐标系中的位置
  
  // IMU数据收集
  bool collecting_data_;
  double collection_duration_;
  int min_valid_samples_;
  std::vector<Eigen::Vector3d> camera_accel_samples_;
  std::vector<Eigen::Vector3d> base_accel_samples_;
  rclcpp::Time start_time_;
  std::mutex samples_mutex_;
  
  // 对齐四元数（odom_horizontal → map）
  Eigen::Quaterniond alignment_quaternion_;
  bool alignment_quaternion_set_;
  
  // 相机到底盘的相对旋转四元数（R_base_cam）
  Eigen::Quaterniond camera_base_quaternion_;
  bool camera_base_quaternion_set_;
  
  // 定时器和回调
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr base_imu_subscription_;
  
  // TF广播器
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuHorizontalAlignNode>();
  while (rclcpp::ok())
  {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
