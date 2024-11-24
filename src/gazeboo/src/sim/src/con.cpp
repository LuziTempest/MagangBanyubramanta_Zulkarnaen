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

            // Initialize PID variables
            sum_yaw_err = 0.0;
            prev_yaw_err = 0.0;
        }

        void OnCmdVel(const geometry_msgs::msg::Twist &msg)
        {
            ignition::math::Pose3d pose = this->model->WorldPose();

            // Calculate yaw error
            double yaw_sp = msg.angular.z; // Desired yaw rate
            double yaw_pv = pose.Rot().Yaw(); // Current yaw
            double yaw_err = error(yaw_sp, yaw_pv);

            // Update PID for yaw
            double yaw_pid = P(yaw_err, kp) + I(sum_yaw_err, yaw_err, ki) + D(prev_yaw_err, yaw_err, dt, kd);

            // Set linear and angular velocities with PID adjustment
            this->model->SetLinearVel(ignition::math::Vector3d(
                msg.linear.x * cosf(pose.Rot().Yaw()) - msg.linear.y * sinf(pose.Rot().Yaw()),
                msg.linear.y * cosf(pose.Rot().Yaw()) + msg.linear.x * sinf(pose.Rot().Yaw()),
                msg.linear.z));
            this->model->SetAngularVel(ignition::math::Vector3d(msg.angular.x, msg.angular.y, yaw_pid));
        }

        // PID helper functions
        double error(double sp, double pv)
        {
            return sp - pv;
        }

        double P(double err, double kp)
        {
            return err * kp;
        }

        double I(double &sum_err, double err, double ki)
        {
            sum_err += err;
            return sum_err * ki;
        }

        double D(double &prev_err, double err, double dt, double kd)
        {
            double d = (err - prev_err) / dt;
            prev_err = err;
            return d * kd;
        }

    private:
        physics::WorldPtr world;
        physics::ModelPtr model;
        std::shared_ptr<rclcpp::Node> node;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

        // PID coefficients and state
        double kp = 0.1, ki = 0.1, kd = 0.1;
        double sum_yaw_err = 0.0;
        double prev_yaw_err = 0.0;
        double dt = 0.01; // Example time step (should be updated dynamically)
    };

    GZ_REGISTER_MODEL_PLUGIN(ROV_Controller)
}
