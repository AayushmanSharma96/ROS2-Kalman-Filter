#include <chrono>
#include <std_msgs/msg/float64.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "EKF.hpp"

using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;

class CarEKFNode : public rclcpp::Node {
public:
    CarEKFNode()
    : Node("car_ekf_node"), ekf_(4, 3)
    {
        double dt = 0.02; // f_sampling = 50 Hz

        // EKF system model: x = [x, y, theta, v], u = [a, omega]
        auto f = [dt](const Vector& x, const Vector& u) {
            Vector xp = x;
            xp(0) += x(3) * std::cos(x(2)) * dt;
            xp(1) += x(3) * std::sin(x(2)) * dt;
            xp(2) += u(1) * dt;
            xp(3) += u(0) * dt;
            return xp;
        };
        auto F = [dt](const Vector& x, const Vector&) {
            Matrix J = Matrix::Identity(4,4);
            J(0,2) = -x(3) * std::sin(x(2)) * dt;
            J(0,3) =  std::cos(x(2)) * dt;
            J(1,2) =  x(3) * std::cos(x(2)) * dt;
            J(1,3) =  std::sin(x(2)) * dt;
            return J;
        };
        auto h = [](const Vector& x) {
            Vector z(1);
            z(0) = x(3);   // velocity
            return z;
        };
        auto H = [](const Vector& x) {
            Matrix J = Matrix::Zero(1,4);
            J(0,3) = 1.0;  // d(v)/d(v)
            return J;
        };

        Matrix Q = Matrix::Identity(4,4) * 1e-3;
        Matrix R = Matrix::Identity(1,1) * 1e-2;
        ekf_.setModel(f, h, F, H, Q, R);

        Vector x0(4);
        x0 << 0.0, 0.0, 0.0, 1.0;
        Matrix P0 = Matrix::Identity(4,4) * 0.1;
        ekf_.init(x0, P0);

        // Subscriptions
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                last_a_ = msg->linear_acceleration.x;
                last_omega_ = msg->angular_velocity.z;
                have_imu_ = true;
                step();
            });

        encoder_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/encoder/speed", 10,
            [this](std_msgs::msg::Float64::SharedPtr msg) {
                last_v_ = msg->data;
                have_enc_ = true;
                step();
            });

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/kf/pose", 10);
    }

private:
    void step() {
        if (!(have_imu_ && have_enc_)) return;
        
        // Reset for next measurement cycle
        have_imu_ = false;
        have_enc_ = false;

        // Predict
        Vector u(2); u << last_a_, last_omega_;
        ekf_.predict(u);

        // Update
        Vector z(1); z << last_v_;
        ekf_.update(z);

        // Publish
        auto x = ekf_.state();
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = "map";
        pose.pose.position.x = x(0);
        pose.pose.position.y = x(1);
        pose.pose.position.z = 0.0;
        pose_pub_->publish(pose);

        RCLCPP_INFO(get_logger(), "x=%.2f y=%.2f th=%.2f v=%.2f", x(0), x(1), x(2), x(3));
    }

    ExtendedKalmanFilter ekf_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr encoder_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    double last_a_ = 0, last_omega_ = 0, last_v_ = 0;
    bool have_imu_ = false, have_enc_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarEKFNode>());
    rclcpp::shutdown();
    return 0;
}
