#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

class EllipsePathPublisher : public rclcpp::Node {
public:
    EllipsePathPublisher(): Node("ellipse_path_publisher"){
        path_publisher_ = create_publisher<nav_msgs::msg::Path>("/path", 10);

        double a = 6.0; // semi-major axis
        double b = 3.0; // semi-minor axis
        double center_x = 0.0; // center x
        double center_y = 0.0; // center y
        double rotation = 0.0; // rotation angle in radians
        double phase = 0.0; // phase shift in radians
        int n_points = 600; // number of points in the path

        timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            [this]{publishPath();
            });
    }
        private:
        void publishPath(){

            const double c = std::cos(rotation);
            const double s = std::sin(rotation);

            nav_msgs::msg::Path path_msg;
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = now();
            path_msg.poses.reserve(n_points);

            for (int i = 0; i < n_points; ++i) {
                double theta = 2.0 * M_PI * i / n_points + phase;
                double x = center_x + a * std::cos(theta) * std::cos(rotation) - b * std::sin(theta) * std::sin(rotation);
                double y = center_y + a * std::cos(theta) * std::sin(rotation) + b * std::sin(theta) * std::cos(rotation);

                geometry_msgs::msg::PoseStamped pos;
                pos.header = path_msg.header;
                pos.pose.position.x = x;
                pos.pose.position.y = y;
                pos.pose.position.z = 0.0; // 2D path

                path_msg.poses.push_back(pos);
            }

            path_publisher_->publish(path_msg);
            //RCLCPP_INFO(get_logger(), "Published path with %d points", n_points);
        }

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        int n_points{600};
        double a{6.0}, b{3.0}, center_x{0.0}, center_y{0.0}, rotation{0.0}, phase{0.0};
        
    };

    

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EllipsePathPublisher>());
    rclcpp::shutdown();
    return 0;
}