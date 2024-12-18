#include <functional>
#include <memory>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace gazebo
{
    class ROV_Controller : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->world = _model->GetWorld();
            this->model = _model;

            this->node = gazebo_ros::Node::Get(_sdf);

            this->cmd_vel_sub = this->node->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&ROV_Controller::OnCmdVel, this, std::placeholders::_1));

            RCLCPP_INFO(this->node->get_logger(), "Model plugin initialized");
        }

        void OnCmdVel(const geometry_msgs::msg::Twist &msg)
        {
            ignition::math::Pose3d pose = this->model->WorldPose();

            this->model->SetLinearVel(ignition::math::Vector3d(
                msg.linear.x * cosf(pose.Rot().Yaw()) - msg.linear.y * sinf(pose.Rot().Yaw()),
                msg.linear.y * cosf(pose.Rot().Yaw()) + msg.linear.x * sinf(pose.Rot().Yaw()),
                msg.linear.z));
            this->model->SetAngularVel(ignition::math::Vector3d(msg.angular.x, msg.angular.y, msg.angular.z));
        }

    private:
        physics::WorldPtr world;
        physics::ModelPtr model;
        std::shared_ptr<rclcpp::Node> node;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    };

    GZ_REGISTER_MODEL_PLUGIN(ROV_Controller)
}

/**
 * float error(SP, PV)
 *      return SP - PV
 * 
 * float P(err, kd)
 *      return (err) * kp
 * 
 * float I(sum_err, err, ki)
 *      sum_err += err
 *      return sum_err * ki
 * 
 * float D(prev_err, err, dt, kd)
 *      d = (err - prev_err) / dt
 *      prev_err = err
 *      return d * kd
 * 
 * void OnUpdate()
 * ...
 *      yaw_err = error(SP_YAW, PV_YAW)
 *      yaw_pid = P(yaw_err, 0.1) + I(sum_yaw_err, yaw_err, 0.1) + D(prev_yaw_err, yaw_err, dt, 0.1)
 *      yaw += yaw_pid
 * ...
 */