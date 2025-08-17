#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from scipy.spatial.transform import Rotation as R

class CarSensorSim(Node):
    def __init__(self):
        super().__init__('car_sensor_sim')
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.enc_pub = self.create_publisher(Float64, '/encoder/speed', 10)
        self.true_pose_pub = self.create_publisher(PoseStamped, '/sim/true_pose', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # f_sampling = 50 Hz

        self.x = 0.0        # x position (m)
        self.y = 0.0        # y position (m)
        self.theta = 0.0    # heading (rad)
        self.v = 1.0        # Linear velocity (m/s)
        self.a = 0.0        # Linear acceleration (m/s^2)
        self.omega = 0.1    # Angular velocity (rad/s)

        # Actuator constraints
        self.v_max = 5.0      # Maximum linear velocity (m/s)
        self.a_max = 3.0      # Maximum linear acceleration (m/s^2)
        self.omega_max = 2.0  # Maximum angular velocity (rad/s)

        self.dt = 0.02
    
    def cmd_cb(self, msg: Twist):
        # Update linear acceleration and angular velocity based on Twist message
        self.a = float(np.clip(msg.linear.x, -self.a_max, self.a_max))
        self.omega = float(np.clip(msg.angular.z, -self.omega_max, self.omega_max))

    def timer_callback(self):

        # State update : Forward Euler
        self.v += self.a * self.dt
        self.v = float(np.clip(self.v, 0.0, self.v_max))
        self.theta += self.omega * self.dt
        self.theta = float(np.clip(self.theta, -np.pi, np.pi))

        self.x += self.v * np.cos(self.theta) * self.dt
        self.y += self.v * np.sin(self.theta) * self.dt

        # Noisy IMU readings
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = self.a + np.random.normal(0, 0.05)
        imu_msg.angular_velocity.z = self.omega + np.random.normal(0, 0.01)

        # Noisy encoder (velocity) readings
        enc_msg = Float64()
        enc_msg.data = self.v + np.random.normal(0, 0.02)

        # Create publisher for IMU and encoder messages
        self.imu_pub.publish(imu_msg)
        self.enc_pub.publish(enc_msg)

        # Publish true pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0

        q = R.from_euler('z', self.theta).as_quat()  # [x, y, z, w]
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.true_pose_pub.publish(pose_msg)

        self.get_logger().info(
            f"Sim State: x={self.x:.2f} y={self.y:.2f} th={self.theta:.2f} v={self.v:.2f} a={self.a:.2f} omega={self.omega:.2f}")

def main():
    rclpy.init()
    node = CarSensorSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
