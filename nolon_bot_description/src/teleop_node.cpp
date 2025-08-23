/*
teleop_twiststamped_node.cpp
Author: Aayush Rath | Date: 14/08/2025

Implements a ROS 2 node for keyboard teleoperation of a mobile robot using 
geometry_msgs::msg::TwistStamped messages. The node runs in a non-blocking 
terminal mode to capture real-time keypresses and publish velocity commands.

Features:
- Publishes TwistStamped messages to `/mobile_base_controller/cmd_vel`
- Uses W/A/S/D keys for linear and angular control:
    - W: Forward
    - S: Backward
    - A: Rotate left
    - D: Rotate right
- Press Q to quit the program
- Non-blocking keyboard input via termios and fcntl
- Periodic publishing at 10 Hz using a wall timer

Usage:
1. Build this node in a ROS 2 workspace.
2. Run the node:
       ros2 run <package_name> teleop_twiststamped_node
3. Focus the terminal and control the robot with W/A/S/D keys.
4. Press Q to stop the node.

Notes:
- Requires a running ROS 2 environment with a mobile base controller subscribed to `/mobile_base_controller/cmd_vel`
- Terminal must have focus to capture keypresses
*/

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <set>

class TeleopTwistStamped : public rclcpp::Node {
    public:
        TeleopTwistStamped() : Node("teleop_twiststamped_node") {
            pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mobile_base_controller/cmd_vel", 10);
            
            enableTerminalRawMode();
            
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TeleopTwistStamped::timerCallback, this));

            RCLCPP_INFO(this->get_logger(), "Use W/A/S/D for motion and Q to quit");
        }

        ~TeleopTwistStamped() {
            disableTerminalRawMode();
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        struct termios original_term_;

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

            geometry_msgs::msg::TwistStamped twist_msg;
            twist_msg.header.stamp = this->now();
            twist_msg.header.frame_id = "base_link";

            if (keys_pressed.count('w')) twist_msg.twist.linear.x = 0.5;
            if (keys_pressed.count('s')) twist_msg.twist.linear.x = -0.5;
            if (keys_pressed.count('a')) twist_msg.twist.angular.z = 0.5;
            if (keys_pressed.count('d')) twist_msg.twist.angular.z = -0.5;

            if (twist_msg.twist.linear.x != 0.0 || twist_msg.twist.angular.z != 0.0) {
                pub_->publish(twist_msg);
            }
        }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopTwistStamped>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}