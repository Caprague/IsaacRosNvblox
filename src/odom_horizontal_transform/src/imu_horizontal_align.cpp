#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <cmath>
#include <thread>

using namespace std::chrono_literals;

class ImuHorizontalAlignNode : public rclcpp::Node
{
public:
  ImuHorizontalAlignNode() : Node("imu_horizontal_align_node")
  {
    RCLCPP_INFO(this->get_logger(), "IMU Horizontal Alignment Node is starting...");
    
    // 声明并获取参数
    this->declare_parameter<double>("start_delay", 0.0);
    this->declare_parameter<std::string>("imu_topic", "/camera/imu");
    this->declare_parameter<double>("transform_publish_rate", 100.0);
    
    this->start_delay_ = this->get_parameter("start_delay").as_double();
    this->imu_topic_ = this->get_parameter("imu_topic").as_string();
    this->transform_publish_rate_ = this->get_parameter("transform_publish_rate").as_double();
    
    // IMU数据收集参数
    this->collecting_data_ = false;
    this->collection_duration_ = 3.0;  // 收集3秒数据
    this->min_valid_samples_ = 10;
    
    // 初始化对齐四元数
    this->alignment_quaternion_.setIdentity();
    this->alignment_quaternion_set_ = false;
    
    // 创建动态TF广播器
    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // 订阅IMU数据
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)); // 保持历史10条消息
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort); // 设置为尽力传输
    this->imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        this->imu_topic_,
        qos,
        std::bind(&ImuHorizontalAlignNode::imu_callback, this, std::placeholders::_1)
    );
    
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
  
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // 数据收集
    if (this->collecting_data_)
    {
      // 收集加速度计数据
      Eigen::Vector3d accel;
      accel[0] = msg->linear_acceleration.x;
      accel[1] = msg->linear_acceleration.y;
      accel[2] = msg->linear_acceleration.z;
      this->accel_samples_.push_back(accel);
    }
  }
  
  void timer_callback()
  {
    // 检查是否收集了足够的时间
    auto elapsed_time = (this->get_clock()->now() - this->start_time_).seconds();
    if (this->collecting_data_ && elapsed_time > this->collection_duration_)
    {
      this->collecting_data_ = false;
      RCLCPP_INFO(this->get_logger(), "Collected %zu IMU samples", this->accel_samples_.size());
      
      // 处理收集的数据
      this->process_imu_data();
      
      // 停止数据收集定时器
      this->timer_->cancel();
    }
  }

void process_imu_data()
{
    if (this->accel_samples_.size() < this->min_valid_samples_)
    {
        RCLCPP_ERROR(this->get_logger(), "Not enough IMU samples collected: %zu", this->accel_samples_.size());
        return;
    }
    
    // 过滤异常值（只保留接近重力加速度的样本）
    std::vector<Eigen::Vector3d> valid_samples;
    for (const auto& sample : this->accel_samples_)
    {
        double norm = sample.norm();
        if (norm >= 9.0 && norm <= 10.0)
        {
            valid_samples.push_back(sample);
        }
    }
    
    if (valid_samples.size() < this->min_valid_samples_)
    {
        RCLCPP_ERROR(this->get_logger(), "Not enough valid IMU samples after filtering: %zu", valid_samples.size());
        return;
    }
    
    // 计算平均加速度向量
    Eigen::Vector3d avg_accel = Eigen::Vector3d::Zero();
    for (const auto& sample : valid_samples)
    {
        avg_accel += sample;
    }
    avg_accel /= valid_samples.size();
    
    // 归一化平均加速度向量，得到重力方向（向下）
    Eigen::Vector3d measured_gravity = avg_accel.normalized();
    
    RCLCPP_INFO(this->get_logger(), "Average accelerometer data: [%.3f, %.3f, %.3f]", 
                avg_accel[0], avg_accel[1], avg_accel[2]);
    RCLCPP_INFO(this->get_logger(), "Measured gravity direction: [%.3f, %.3f, %.3f]", 
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
    
    // 计算从测量重力方向到目标重力方向的旋转
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
    double corrected_roll_deg = yaw_deg;  // 原来的pitch值作为新的roll值
    double corrected_pitch_deg = (roll_imu * 180.0 / M_PI) < -90.0 ? roll_deg:(360.0-roll_deg);  // 原来的roll值作为新的pitch值
    double corrected_yaw_deg = pitch_deg;     // yaw值保持不变
    
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
    
    // 创建定时器定期发布变换
    this->create_transform_publish_timer();
}

  void create_transform_publish_timer()
  {
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
      // this->publish_camera_transform();
      this->publish_camera_inverse_transform();
      this->publish_lidar_transform();
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
    this->tf_broadcaster_->sendTransform(transform);
  }

  void publish_camera_transform()
  {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id = "base_link";    // 父坐标系
      transform.child_frame_id = "camera_link";  // 子坐标系
      
      // 设置平移：x+0.34, z+0.09, 其余为0
      transform.transform.translation.x = 0.34;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = 0.09;
      
      // 设置旋转：单位四元数（无旋转）
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
      transform.transform.rotation.w = 1.0;
      
      // 发布变换
      this->tf_broadcaster_->sendTransform(transform);
  }

  void publish_camera_inverse_transform()
  {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id = "camera_link";  // 父坐标系现在是camera_link
      transform.child_frame_id = "base_link";    // 子坐标系现在是base_link
      
      // 逆变换的平移：原平移取反，即 x = -0.34, z = -0.09
      transform.transform.translation.x = -0.34;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = -0.09;
      
      // 逆变换的旋转：原旋转四元数取共轭（对于单位四元数，共轭即逆）
      // 原旋转为单位四元数 (w=1, x=0, y=0, z=0)，其共轭（逆）不变
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = -0.11;
      transform.transform.rotation.z = 0.0;
      transform.transform.rotation.w = 1.0;
      
      // 发布逆变换
      this->tf_broadcaster_->sendTransform(transform);
  }

  void publish_lidar_transform()
  {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id = "base_link";       // 父坐标系
      transform.child_frame_id = "utlidar_lidar";     // 子坐标系
      
      // 设置平移：x+0.28945, z-0.04682, 其余为0
      transform.transform.translation.x = 0.28945;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = -0.04682;
      
      // 设置旋转：使用提供的四元数 (w,x,y,z) = (0.13132, 0, 0.99134, 0)
      // 对应欧拉角 (roll, pitch, yaw) = (-180.0°, 15.091°, -180.0°)
      transform.transform.rotation.w = 0.13132;
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.99134;
      transform.transform.rotation.z = 0.0;
      
      // 发布变换
      this->tf_broadcaster_->sendTransform(transform);
  }
  
  // 参数
  double start_delay_;
  std::string imu_topic_;
  double transform_publish_rate_;
  
  // IMU数据收集
  bool collecting_data_;
  double collection_duration_;
  int min_valid_samples_;
  std::vector<Eigen::Vector3d> accel_samples_;
  rclcpp::Time start_time_;
  
  // 对齐四元数
  Eigen::Quaterniond alignment_quaternion_;
  bool alignment_quaternion_set_;
  
  // 定时器和回调
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  
  // TF广播器
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
