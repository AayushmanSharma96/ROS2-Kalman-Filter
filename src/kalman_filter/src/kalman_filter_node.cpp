#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "KalmanFilter.hpp"

using namespace std::chrono_literals;

class KFNode : public rclcpp::Node
{
public:
  KFNode() : Node("kalman_filter_node"),
             kf_(6, 6)   // example: 6-D state, 6-D measurement
  {
    // Dummy model; replace with your own
    KalmanFilter::Matrix A = KalmanFilter::Matrix::Identity(6,6);
    kf_.setModel(A, {}, KalmanFilter::Matrix::Identity(6,6),
                 KalmanFilter::Matrix::Identity(6,6)*1e-3,
                 KalmanFilter::Matrix::Identity(6,6)*1e-2);
    kf_.init(KalmanFilter::Vector::Zero(6),
             KalmanFilter::Matrix::Identity(6,6));

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      [this](sensor_msgs::msg::Imu::SharedPtr msg)
      {
        KalmanFilter::Vector z(6);
        z << msg->linear_acceleration.x,
             msg->linear_acceleration.y,
             msg->linear_acceleration.z,
             msg->angular_velocity.x,
             msg->angular_velocity.y,
             msg->angular_velocity.z;

        kf_.predict();          // no control input
        kf_.update(z);

        RCLCPP_INFO(get_logger(),
                    "Updated state: [%.3f %.3f %.3f  ...]",
                    kf_.state()(0), kf_.state()(1), kf_.state()(2));

        publish_state(msg->header.stamp);
      });

    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/kf/pose", 10);
  }

private:
  void publish_state(const rclcpp::Time& stamp)
  {
    const auto& x = kf_.state();
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = "map";
    out.pose.position.x = x(0);
    out.pose.position.y = x(1);
    out.pose.position.z = x(2);
    // orientation left zero for brevity
    pub_->publish(out);
  }

  KalmanFilter kf_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KFNode>());
  rclcpp::shutdown();
  return 0;
}
