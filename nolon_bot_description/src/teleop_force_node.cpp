/*
teleop_force_node.cpp
Author: Aayush Rath | Date: 25/08/2025

Implements a ROS 2 node for keyboard teleoperation of a mobile robot using
geometry_msgs::msg::TwistStamped messages. Unlike direct velocity control,
this simulates "forces and torques" like in video games:

- Keys apply acceleration instead of setting fixed velocities
- Robot gradually slows down when keys are released (friction)
- Feels smoother and more natural for teleop control

Controls:
    W: Accelerate forward
    S: Accelerate backward
    A: Apply torque left
    D: Apply torque right
    Q: Quit program
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <set>

class TeleopForceStyle : public rclcpp::Node {
public:
    TeleopForceStyle() : Node("teleop_force_node") {
        pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/mobile_base_controller/cmd_vel", 10);

        enableTerminalRawMode();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TeleopForceStyle::timerCallback, this));

        RCLCPP_INFO(this->get_logger(),
                    "Use W/A/S/D for force-like motion and Q to quit");
    }

    ~TeleopForceStyle() {
        disableTerminalRawMode();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct termios original_term_;

    // Current "state" of velocity (like physics sim)
    double linear_vel_ = 0.0;
    double angular_vel_ = 0.0;

    // Parameters for "force-style" control
    const double accel_ = 0.1;     // linear acceleration step
    const double decel_ = 0.08;    // natural deceleration
    const double max_lin_vel_ = 0.6;
    const double max_ang_vel_ = 1.5;
    const double ang_accel_ = 0.15; // angular acceleration step

    void enableTerminalRawMode() {
        tcgetattr(STDIN_FILENO, &original_term_);
        struct termios raw = original_term_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }

    void disableTerminalRawMode() {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_term_);
    }

    void timerCallback() {
        std::set<char> keys_pressed;
        char c;

        while (read(STDIN_FILENO, &c, 1) > 0) {
            if (c == 'q') {
                rclcpp::shutdown();
                return;
            }
            keys_pressed.insert(tolower(c));
        }

        // Apply forces (acceleration increments)
        if (keys_pressed.count('w')) linear_vel_ += accel_;
        if (keys_pressed.count('s')) linear_vel_ -= accel_;
        if (keys_pressed.count('a')) angular_vel_ += ang_accel_;
        if (keys_pressed.count('d')) angular_vel_ -= ang_accel_;

        // Clamp velocities
        if (linear_vel_ > max_lin_vel_) linear_vel_ = max_lin_vel_;
        if (linear_vel_ < -max_lin_vel_) linear_vel_ = -max_lin_vel_;
        if (angular_vel_ > max_ang_vel_) angular_vel_ = max_ang_vel_;
        if (angular_vel_ < -max_ang_vel_) angular_vel_ = -max_ang_vel_;

        // Apply natural deceleration (friction-like)
        applyFriction(linear_vel_, decel_);
        applyFriction(angular_vel_, decel_);

        // Publish the twist
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "base_link";
        twist_msg.twist.linear.x = linear_vel_;
        twist_msg.twist.angular.z = angular_vel_;
        pub_->publish(twist_msg);
    }

    void applyFriction(double &vel, double factor) {
        if (vel > 0.0) {
            vel -= factor;
            if (vel < 0.0) vel = 0.0;
        } else if (vel < 0.0) {
            vel += factor;
            if (vel > 0.0) vel = 0.0;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopForceStyle>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
