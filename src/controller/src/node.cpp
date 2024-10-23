#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "interfaces/msg/command.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

class Publisher : public rclcpp::Node
{
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
	rclcpp::Publisher<interfaces::msg::Command>::SharedPtr pub;

	void fungsi_subscribe(const sensor_msgs::msg::Joy &msg)
	{
	
    float stick_x = msg.axes[0] * 250.0;  // stik kiri, sumbu X
    float stick_y = msg.axes[1] * 250.0;  // stik kiri, sumbu Y

    float croskey_x = msg.axes[6] * 250.0;  // D-pad kiri/kanan
    float croskey_y = msg.axes[7] * 250.0;  // D-pad atas/bawah

    float depth = (msg.axes[4] + 1) * 5;  // stik kanan, sumbu y
    float yaw = msg.axes[3] * 180.0;  // Stick kanan, sumbu x
    
    float x_button = msg.buttons[2];
    float y_button = msg.buttons[3];
    float b_button = msg.buttons[1];
    float a_button = msg.buttons[0];

    float x,y;
    if (croskey_x != 0.0){
        x = croskey_x;
    } else {
        x = stick_x;
    } 

    if (croskey_y != 0.0){
        y = croskey_y;
    } else {
        y = stick_y;
    }    

    auto cmd = interfaces::msg::Command();

    cmd.x_cmd = x;
    cmd.y_cmd = y;
    cmd.depth = depth;
    cmd.yaw = yaw;
    cmd.x = x_button;
    cmd.y = y_button;
    cmd.a = a_button;
    cmd.b = b_button; 

    pub->publish(cmd);
	}

public:
	Publisher() : Node("node")
	{
		sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Publisher::fungsi_subscribe, this, _1));
		pub = this->create_publisher<interfaces::msg::Command>("cmd_vel", 10);
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Publisher>());
	rclcpp::shutdown();
	return 0;
}