#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class PID : public rclcpp::Node {
public:
    PID() : Node("pid_node") {
        pose_topic_ = declare_parameter<std::string>("pose_topic", "/kf/pose");
        lookahead_distance_ = declare_parameter<double>("lookahead_distance", 1); 
        v_ref_ = declare_parameter<double>("v_ref", 1.0);

        // Longitudinal PID parameters
        Kp_v_ = declare_parameter<double>("Kp_v", 1.0);
        Ki_v_ = declare_parameter<double>("Ki_v", 0.2);
        integral_max_v_ = declare_parameter<double>("integral_max_v", 5.0);

        // Lateral PID parameters
        Kpsi_ = declare_parameter<double>("Kpsi", 1.5);
        Ky_ = declare_parameter<double>("Ky", 0.6);

        // Actuator limits
        a_max_ = declare_parameter<double>("a_max", 10.0);
        omega_max_ = declare_parameter<double>("omega_max", 10.0);

        dt_ = declare_parameter<double>("dt", 0.02);    // 50 Hz sampling rate
        
        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, rclcpp::SensorDataQoS(),
            std::bind(&PID::poseCallback, this, std::placeholders::_1));

        sub_path_ = create_subscription<nav_msgs::msg::Path>(
            "/path", 10,
            std::bind(&PID::pathCallback, this, std::placeholders::_1));

        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
            std::bind(&PID::timerCallback, this));

        RCLCPP_INFO(get_logger(), "PID Controller running | Pose topic: %s, Lookahead distance: %.2f, Reference velocity: %.2f",
                    pose_topic_.c_str(), lookahead_distance_, v_ref_);
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped &msg) {
        const double x = msg.pose.position.x;
        const double y = msg.pose.position.y;
        const double theta = yawFrom(msg.pose.orientation);

        // Speed estimation using FD
        const double t_now = this->now().seconds();
        if (have_pose_){
            const double dt = std::max(t_now - last_time_, 1e-6);
            if (dt > 0.0) {
                v_ = std::hypot(x - last_x_, y - last_y_) / dt;
            }
        }
            last_x_ = x;
            last_y_ = y;
            last_theta_ = theta;
            last_time_ = t_now;
            have_pose_ = true;
        }

    void pathCallback(const nav_msgs::msg::Path &msg) {
        pts_.clear();
        pts_.reserve(msg.poses.size());
        for (const auto &pose : msg.poses) {
            pts_.push_back({pose.pose.position.x, pose.pose.position.y});
        }
        have_path_ = !pts_.empty();
    }

    void timerCallback() {
        if (!have_pose_ || !have_path_) return;

        // Find the closest point on the path
        size_t idx_min = 0;
        double best = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < pts_.size(); ++i) {
            double d = distanceTo(pts_[i]);
            if (d < best) { best = d; idx_min = i; }
        }

        size_t idx_target = idx_min;
        if (pts_.size() >= 2) {
            const double Ld = std::max(lookahead_distance_, 1e-3);
            double acc_dist = 0.0;
            size_t i = idx_min;
            size_t guard = 0;                     // prevents infinite loop if Ld > path length
            while (acc_dist < Ld && guard < pts_.size()) {
                size_t j = (i + 1) % pts_.size(); // <-- wrap-around here
                acc_dist += std::hypot(pts_[j].first  - pts_[i].first,
                                    pts_[j].second - pts_[i].second);
                i = j;
                ++guard;
            }
            idx_target = i; // last index reached at/after Ld
        }
        const auto &target = pts_[idx_target];
        const double dx = target.first - last_x_;
        const double dy = target.second - last_y_;
        const double theta_target = std::atan2(dy, dx);
        const double dtheta = normalizeAngle(theta_target - last_theta_);

        // Longitudinal control
        error_v_ = v_ref_ - v_;
        integral_v_ += error_v_ * dt_;
        integral_v_ = std::clamp(integral_v_, -integral_max_v_, integral_max_v_);
        double a = Kp_v_ * error_v_ + Ki_v_ * integral_v_;
        a = std::clamp(a, -a_max_, a_max_);

        // Lateral control: Pure Pursuit
        const double v_for_omega = std::max(v_, 0.05); 
        const double Ld = std::max(lookahead_distance_, 1e-3);
        double omega = (2.0 * v_for_omega * std::sin(dtheta)) / Ld;
        omega = std::clamp(omega, -omega_max_, omega_max_);

        // Publish command
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = a;  // linear acceleration  
        cmd.angular.z = omega;
        pub_cmd_->publish(cmd);

        RCLCPP_INFO(get_logger(), "Cmd: a=%.2f omega=%.2f", cmd.linear.x, cmd.angular.z);
    }


    static double yawFrom(const geometry_msgs::msg::Quaternion &q) {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    double distanceTo(const std::pair<double, double> &pt) const {
        return std::hypot(pt.first - last_x_, pt.second - last_y_);
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    std::string pose_topic_;
    double lookahead_distance_;
    double v_ref_;
    double Kp_v_, Ki_v_, integral_max_v_;
    double Kpsi_, Ky_;
    double a_max_, omega_max_;
    double dt_;

    bool have_pose_ = false;
    bool have_path_ = false;
    double last_x_{0.0}, last_y_{0.0}, last_theta_{0.0}, last_time_{0.0};
    double v_{0.0}, error_v_{0.0}, integral_v_{0.0};
    std::vector<std::pair<double, double>> pts_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PID>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}  
