#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

// Struktur untuk mendeteksi label dan koordinat
struct Detection
{
    std::string label;
    float confidence;
    cv::Rect box;
};

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher() : Node("video_publisher")
    {
        // Publisher untuk label dan koordinat objek
        publisher_ = this->create_publisher<std_msgs::msg::String>("object_detection", 10);

        // Inisialisasi video capture
        cap_.open("src/yolonih/assets/third.mp4");  // Gunakan video third.mp4
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file");
            return;
        }

        // Muat model YOLOv5 ONNX
        net_ = cv::dnn::readNet("src/yolonih/assets/best.onnx");  // Gunakan best.onnx (model YOLOv5)
        if (net_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YOLOv5 model");
            return;
        }

        // Proses video dalam loop
        process_video();
    }

private:
    void process_video()
    {
        cv::Mat frame;
        while (rclcpp::ok() && cap_.read(frame))
        {
            if (frame.empty())
                break;

            // Proses frame untuk deteksi objek
            std::vector<Detection> detections = detect_objects(frame);

            // Publish objek yang terdeteksi jika ada
            for (const auto &detection : detections)
            {
                if (detection.confidence > 0.5) // threshold confidence
                {
                    std_msgs::msg::String msg;
                    msg.data = "Label: " + detection.label + ", Confidence: " + std::to_string(detection.confidence) +
                               ", Location: (" + std::to_string(detection.box.x) + ", " + std::to_string(detection.box.y) + ")";
                    publisher_->publish(msg);
                }
            }

            // Tampilkan frame (optional)
            cv::imshow("Frame", frame);
            if (cv::waitKey(1) == 'q')
                break;
        }
    }

    std::vector<Detection> detect_objects(cv::Mat &frame)
    {
        std::vector<Detection> detections;

        // Pre-process the frame for detection
        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(0, 0, 0), true, false);
        net_.setInput(blob);  // Menggunakan blob untuk input

        // Forward pass
        std::vector<cv::Mat> outputs;
        net_.forward(outputs);

        // Process the outputs
        float *data = (float *)outputs[0].data;
        for (int i = 0; i < outputs[0].rows; ++i, data += outputs[0].cols)
        {
            // Confidence
            float confidence = data[4];

            if (confidence > 0.5) // threshold confidence
            {
                // Get coordinates and label
                int center_x = static_cast<int>(data[0] * frame.cols);
                int center_y = static_cast<int>(data[1] * frame.rows);
                int width = static_cast<int>(data[2] * frame.cols);
                int height = static_cast<int>(data[3] * frame.rows);

                // Get class label
                int class_id = -1;
                float max_class_score = -1;
                for (int c = 5; c < outputs[0].cols; c++)
                {
                    if (data[c] > max_class_score)
                    {
                        max_class_score = data[c];
                        class_id = c - 5;
                    }
                }

                // Store detection
                Detection d;
                std::vector<std::string> class_labels = {"Baskom", "Flare", "Celana"};
                d.label = class_labels[class_id];
                d.confidence = confidence;
                d.box = cv::Rect(center_x - width / 2, center_y - height / 2, width, height);
                detections.push_back(d);
            }
        }
        return detections;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    cv::VideoCapture cap_;
    cv::dnn::Net net_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}
