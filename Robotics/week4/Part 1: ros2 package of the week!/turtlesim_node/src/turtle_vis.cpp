#include <iostream>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

double r = 0.05;
double l = 0.1;

void compute(double vr, double vl, double& v, double& w) {
    v = (r / 2) * (vr + vl);
    w = (r / (2 * l)) * (vr - vl);
}
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("wheel_velocity_controller");
    auto publisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    double vr = 2.0;
    double vl = 1.0;
    double v, w;
    compute(vr, vl, v, w);
    std::cout << "Linear velocity: " << v << " m/s\n";
    std::cout << "Angular velocity: " << w << " rad/s\n";
    if (w > 0)
        std::cout << "Turning left\n";
    else if (w < 0)
        std::cout << "Turning right\n";
    else
        std::cout << "Moving straight\n";
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    publisher->publish(cmd);

    rclcpp::shutdown();
    return 0;
}

