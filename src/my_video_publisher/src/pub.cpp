#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

class VideoPublisher : public rclcpp::Node {
public:
    VideoPublisher() : Node("video_publisher") {
        publisher_frame_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);
        publisher_mask_ = this->create_publisher<sensor_msgs::msg::Image>("mask", 10);

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
        int hue_min = 0, hue_max = 177;
        int sat_min = 91, sat_max = 116;
        int val_min = 0, val_max = 201;

        if (cap_.read(frame)) {
        
            cv::Mat hsv;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            
            cv::Scalar minHSV(hue_min, sat_min, val_min);
            cv::Scalar maxHSV(hue_max, sat_max, val_max);

            cv::Mat mask;
            cv::inRange(hsv, minHSV, maxHSV, mask);
            
            cv::Scalar lower_red1(0, 100, 100), upper_red1(10, 255, 255);
            cv::Scalar lower_red2(160, 100, 100), upper_red2(180, 255, 255);

            cv::Mat mask_red, mask_colored;
            cv::Mat red_mask1, red_mask2;
            cv::inRange(hsv, lower_red1, upper_red1, red_mask1);
            cv::inRange(hsv, lower_red2, upper_red2, red_mask2);
            mask_red = red_mask1 | red_mask2;

            cv::cvtColor(mask, mask_colored, cv::COLOR_GRAY2BGR);
            frame.copyTo(mask_colored, mask_red);

            // cv::imshow("Original", frame);
            // cv::imshow("Masking", mask_colored);
            cv::waitKey(1);

            if (cv::waitKey(30) == 'q') rclcpp::shutdown();;

            auto msg_frame = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            auto msg_mask = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mask_colored).toImageMsg();
            
            publisher_frame_->publish(*msg_frame);
            publisher_mask_->publish(*msg_mask);
        } else {
            RCLCPP_INFO(this->get_logger(), "End");
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_frame_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_mask_;
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