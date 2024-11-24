#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

class VideoPublisher : public rclcpp::Node {
public:
    VideoPublisher() : Node("video_publisher") {
        publisher_frame_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);

        // Buka video
        cap_.open("src/my_video_publisher/assets/fourth.mp4");
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Gagal membuka file!");
            rclcpp::shutdown();
            return;
        }

        // Setup untuk menyimpan video dalam format MP4 (menggunakan H264 codec)
        int fourcc = cv::VideoWriter::fourcc('M', 'P', '4', 'V');
        video_writer_.open("src/my_video_publisher/assets/output_video2.mp4", fourcc, 30.0, cv::Size(frame_width_, frame_height_));
        if (!video_writer_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Gagal membuka file video untuk menulis!");
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
            // Menyimpan frame ke dalam video baru dengan format MP4
            video_writer_.write(frame);

            // Konversi frame OpenCV ke pesan ROS
            auto msg_frame = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

            // Publikasi frame ke topik
            publisher_frame_->publish(*msg_frame);

            // Menampilkan frame di jendela (Opsional, untuk memutar video di layar)
            cv::imshow("Camera Frame", frame);
            cv::waitKey(1);
        } else {
            RCLCPP_INFO(this->get_logger(), "End");
            rclcpp::shutdown();
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_frame_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    cv::VideoWriter video_writer_;  // Video writer untuk menyimpan video
    int frame_width_ = 640;  // Lebar frame video
    int frame_height_ = 640; // Tinggi frame video
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
