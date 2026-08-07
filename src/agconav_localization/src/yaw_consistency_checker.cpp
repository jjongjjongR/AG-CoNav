#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <memory>
#include <string>

using std::placeholders::_1;

class YawConsistencyChecker : public rclcpp::Node
{
public:
  YawConsistencyChecker() : Node("yaw_consistency_checker")
  {
    // Declare parameters
    this->declare_parameter("yaw_diff_threshold_rad", 0.1);
    this->declare_parameter("covariance_threshold", 0.05);
    this->declare_parameter("check_rate_hz", 1.0);
    this->declare_parameter("odom_topic", "odom");
    this->declare_parameter("imu_topic", "imu");
    this->declare_parameter("status_topic", "yaw_check_status");

    // Get parameters
    yaw_diff_threshold_rad_ = this->get_parameter("yaw_diff_threshold_rad").as_double();
    covariance_threshold_ = this->get_parameter("covariance_threshold").as_double();
    check_rate_hz_ = this->get_parameter("check_rate_hz").as_double();
    
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string status_topic = this->get_parameter("status_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Starting YawConsistencyChecker with threshold: %.3f rad, cov: %.3f", 
                yaw_diff_threshold_rad_, covariance_threshold_);

    // Subscriptions
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10), std::bind(&YawConsistencyChecker::odom_callback, this, _1));
      
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::QoS(10), std::bind(&YawConsistencyChecker::imu_callback, this, _1));

    // Publisher
    status_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
      status_topic, rclcpp::QoS(1));

    // Timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / check_rate_hz_));
    timer_ = this->create_wall_timer(
      timer_period, std::bind(&YawConsistencyChecker::timer_callback, this));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (last_odom_ && msg->header.stamp.sec < last_odom_->header.stamp.sec) {
      RCLCPP_WARN(this->get_logger(), "Time jump detected. Resetting initial offset.");
      initial_offset_set_ = false;
    }
    last_odom_ = msg;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = msg;
  }

  double get_yaw_from_quaternion(double x, double y, double z, double w)
  {
    tf2::Quaternion q(x, y, z, w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  double normalize_angle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void timer_callback()
  {
    diagnostic_msgs::msg::DiagnosticStatus status_msg;
    status_msg.name = this->get_namespace() + std::string("/") + this->get_name();

    if (!last_odom_ || !last_imu_) {
      status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status_msg.message = "Waiting for Odom and IMU data";
      status_pub_->publish(status_msg);
      return;
    }

    // Check covariances
    double odom_yaw_cov = last_odom_->pose.covariance[35]; // 6x6 matrix, index 35 is yaw variance
    double imu_yaw_cov = last_imu_->orientation_covariance[8]; // 3x3 matrix, index 8 is yaw variance

    // Allow a special case where covariance might be exactly 0 (if sensor doesn't report it) but normally we check if it exceeds threshold
    if (odom_yaw_cov > covariance_threshold_ || imu_yaw_cov > covariance_threshold_) {
      status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status_msg.message = "Covariance too high; alignment pending";
      status_pub_->publish(status_msg);
      return;
    }

    double odom_yaw = get_yaw_from_quaternion(
      last_odom_->pose.pose.orientation.x,
      last_odom_->pose.pose.orientation.y,
      last_odom_->pose.pose.orientation.z,
      last_odom_->pose.pose.orientation.w);

    double imu_yaw = get_yaw_from_quaternion(
      last_imu_->orientation.x,
      last_imu_->orientation.y,
      last_imu_->orientation.z,
      last_imu_->orientation.w);

    message_count_++;
    if (message_count_ < 5) {
      status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status_msg.message = "Stabilizing signals...";
      status_pub_->publish(status_msg);
      return;
    }

    if (!initial_offset_set_) {
      initial_yaw_offset_ = normalize_angle(imu_yaw - odom_yaw);
      initial_offset_set_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial yaw offset set to: %.3f rad", initial_yaw_offset_);
    }

    double diff = std::abs(normalize_angle(imu_yaw - odom_yaw - initial_yaw_offset_));

    if (diff > yaw_diff_threshold_rad_) {
      status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status_msg.message = "Yaw inconsistency exceeds threshold";
      
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "yaw_diff";
      kv.value = std::to_string(diff);
      status_msg.values.push_back(kv);
    } else {
      status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status_msg.message = "Yaw consistent";
    }

    status_pub_->publish(status_msg);
  }

  double yaw_diff_threshold_rad_;
  double covariance_threshold_;
  double check_rate_hz_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_;

  bool initial_offset_set_ = false;
  double initial_yaw_offset_ = 0.0;
  int message_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YawConsistencyChecker>());
  rclcpp::shutdown();
  return 0;
}
