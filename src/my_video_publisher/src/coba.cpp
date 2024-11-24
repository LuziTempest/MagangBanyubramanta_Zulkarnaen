#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

class VideoPublisher : public rclcpp::Node {
public:
    VideoPublisher() : Node("video_publisher") {
        publisher_frame_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);

        cap_.open("src/my_video_publisher/assets/fourth.mp4");
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Gagal membuka file!");
            rclcpp::shutdown();
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30), 
            std::bind(&VideoPublisher::timer_callback, this)
        );
    }
    void timer_callback() {
        cv::Mat frame;

        if (cap_.read(frame)) {
            // Tampilkan video dengan OpenCV
            cv::imshow("Original", frame);
            cv::waitKey(1);

            // Konversi frame menjadi ROS2 Image dan publish
            auto msg_frame = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            publisher_frame_->publish(*msg_frame);
        } else {
            RCLCPP_INFO(this->get_logger(), "End of video.");
            rclcpp::shutdown();
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_frame_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
