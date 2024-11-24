// #include <memory>
// #include <string>
// #include <vector>

// #include <opencv2/opencv.hpp>
// #include <openvino/openvino.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>

// // Label yang digunakan dalam model
// const char* coconame[] = { "Flare", "Baskom", "Celana", "Any" };

// // Konstanta terkait inferensi dan deteksi
// #define N_CLASSES 4 // Jumlah kelas sesuai dengan label
// #define INPUT_WIDTH 640
// #define INPUT_HEIGHT 640
// #define CONF_THRESH 0.4
// #define SCORE_THRESH 0.4
// #define NMS_THRESH 0.4

// struct Detection
// {
//     float conf;
//     int class_;
//     cv::Rect box;
// };

// class VideoSubscriberNode : public rclcpp::Node
// {
// public:
//     ov::CompiledModel compiled_model;
//     ov::InferRequest infer_request; 
//     VideoSubscriberNode() : Node("video_subscriber_node")
//     {
//         // Subscriber untuk topik kamera
//         rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
//         image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "/camera", 10,
//             std::bind(&VideoSubscriberNode::image_callback, this, std::placeholders::_1));

//         ov::Core core;
//         std::shared_ptr<ov::Model> model = core.read_model("src/yolonih/assets/best.onnx"); // ONNX / PYTORCH / OPENVINO / etc.
//         // (1, 3, 640, 640) -> (1, 84, 8400)

//         ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
//         ppp.input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR);
//         ppp.input().preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB);
//         ppp.input().model().set_layout("NCHW");
//         ppp.output(0).tensor().set_element_type(ov::element::f32);
//         model = ppp.build();

//         // Kompilasi model untuk inferensi
        
//         compiled_model = core.compile_model(model, "CPU"); // CPU / CPU_ARM / GPU / dGPU / NPU / AUTO / BATCH / HETERO
//         infer_request = compiled_model.create_infer_request();
//     }

// private:
//     void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         // Convert ROS image message to OpenCV Mat
//         cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

//         // Preprocessing
//         float width = frame.cols;
//         float height = frame.rows;
//         cv::Size new_shape = cv::Size(INPUT_WIDTH, INPUT_HEIGHT);
//         float ratio = float(new_shape.width / std::max(width, height));
//         int new_width = int(std::round(width * ratio));
//         int new_height = int(std::round(height * ratio));

//         int padding_x = new_shape.width - new_width;
//         int padding_y = new_shape.height - new_height;
//         cv::Scalar padding_color = cv::Scalar(100, 100, 100);

//         cv::Mat input_frame;
//         cv::resize(frame, input_frame, cv::Size(new_width, new_height), 0, 0, cv::INTER_AREA);
//         cv::copyMakeBorder(input_frame, input_frame, 0, padding_y, 0, padding_x, cv::BORDER_CONSTANT, padding_color);

//         // Menghitung rasio x dan y
//         float ratio_x = (float)frame.cols / (float)(input_frame.cols - padding_x);
//         float ratio_y = (float)frame.rows / (float)(input_frame.rows - padding_y);

//         // Inferensi dengan OpenVINO
//         ov::Tensor input_tensor(ov::element::u8, {1, 3, INPUT_HEIGHT, INPUT_WIDTH}, input_frame.data);
//         infer_request.set_input_tensor(input_tensor);
//         infer_request.infer();

//         // Menangani output model
//         const ov::Tensor& output_tensor = infer_request.get_output_tensor();
//         ov::Shape output_shape = output_tensor.get_shape();
//         float *detections = output_tensor.data<float>();

//         // Post-processing
//         std::vector<cv::Rect> boxes;
//         std::vector<int> class_ids;
//         std::vector<float> confidences;

//         for (int i = 0; i < output_shape[1]; i++)
//         {
//             float *detection = detections + (i * output_shape[2]);
//             float confidence = detection[4];
//             if (confidence < CONF_THRESH)
//                 continue;

//             float *class_scores = detection + 5;
//             cv::Mat scores(1, N_CLASSES, CV_32FC1, class_scores);
//             cv::Point class_id;
//             double max_class_score;
//             cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

//             if (max_class_score < SCORE_THRESH)
//                 continue;

//             float x = detection[0];
//             float y = detection[1];
//             float w = detection[2];
//             float h = detection[3];
//             float x_min = x - (w / 2);
//             float y_min = y - (h / 2);

//             confidences.push_back(confidence);
//             class_ids.push_back(class_id.x);
//             boxes.push_back(cv::Rect(x_min, y_min, w, h));
//         }

//         // NMS untuk mengurangi duplikasi deteksi
//         std::vector<int> nms_result;
//         cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESH, NMS_THRESH, nms_result);

//         std::vector<Detection> output;
//         for (int &i : nms_result)
//         {
//             Detection d;
//             d.class_ = class_ids[i];
//             d.conf = confidences[i];
//             d.box = boxes[i];
//             output.push_back(d);
//         }

//         // Menampilkan hasil
//         for (Detection &d : output)
//         {
//             cv::Rect &box = d.box;
//             box.x /= ratio_x;
//             box.y /= ratio_y;
//             box.width /= ratio_x;
//             box.height /= ratio_y;

//             cv::Scalar color = cv::Scalar(255, 0, 0); // Warna kotak (red)
//             float color_mean = cv::mean(color)[0];
//             cv::Scalar text_color = color_mean > 0.5 ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255);

//             cv::rectangle(frame, cv::Point(box.x, box.y), cv::Point(box.x + box.width, box.y + box.height), color * 255, 2);
//             int baseline = 0;
//             char text[256];
//             sprintf(text, "%s %0.1f%%", coconame[d.class_], d.conf * 100);
//             cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
//             cv::Scalar text_background_color = color * 0.7 * 255;

//             cv::rectangle(frame, cv::Rect(cv::Point(box.x, box.y), cv::Size(label_size.width, label_size.height + baseline)), text_background_color, -1);
//             cv::putText(frame, text, cv::Point(box.x, box.y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1);
//         }

//         // Menampilkan hasil di OpenCV
//         cv::imshow("Detection Results", frame);
//         cv::waitKey(1);
//     }
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<VideoSubscriberNode>());
//     rclcpp::shutdown();
//     return 0;
// }


// #include <memory>
// #include <string>
// #include <vector>

// #include <opencv2/opencv.hpp>
// #include <openvino/openvino.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>

// // Label yang digunakan dalam model
// const char* coconame[] = { "Flare", "Baskom", "Celana", "Any" };

// // Konstanta terkait inferensi dan deteksi
// #define N_CLASSES 4 // Jumlah kelas sesuai dengan label
// #define INPUT_WIDTH 640
// #define INPUT_HEIGHT 640
// #define CONF_THRESH 0.4
// #define SCORE_THRESH 0.4
// #define NMS_THRESH 0.4

// struct Detection
// {
//     float conf;
//     int class_;
//     cv::Rect box;
// };

// class VideoSubscriberNode : public rclcpp::Node
// {
// public:
//     ov::CompiledModel compiled_model;
//     ov::InferRequest infer_request; 
//     VideoSubscriberNode() : Node("video_subscriber_node")
//     {
//         // Subscriber untuk topik kamera
//         image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "/camera", 10,
//             std::bind(&VideoSubscriberNode::image_callback, this, std::placeholders::_1));

//         ov::Core core;
//         std::shared_ptr<ov::Model> model = core.read_model("src/yolonih/assets/best.onnx"); // ONNX / PYTORCH / OPENVINO / etc.
//         // (1, 3, 640, 640) -> (1, 84, 8400)

//         ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
//         ppp.input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR);
//         ppp.input().preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB);
//         ppp.input().model().set_layout("NCHW");
//         ppp.output(0).tensor().set_element_type(ov::element::f32);
//         model = ppp.build();

//         // Kompilasi model untuk inferensi
//         compiled_model = core.compile_model(model, "CPU"); // CPU / CPU_ARM / GPU / dGPU / NPU / AUTO / BATCH / HETERO
//         infer_request = compiled_model.create_infer_request();
//     }

// private:
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

//     void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         // Convert ROS image message to OpenCV Mat
//         cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

//         if (frame.empty()) {
//             RCLCPP_ERROR(this->get_logger(), "Received empty image.");
//             return;
//         }

//         // Preprocessing
//         float width = frame.cols;
//         float height = frame.rows;
//         cv::Size new_shape = cv::Size(INPUT_WIDTH, INPUT_HEIGHT);
//         float ratio = float(new_shape.width / std::max(width, height));
//         int new_width = int(std::round(width * ratio));
//         int new_height = int(std::round(height * ratio));

//         int padding_x = new_shape.width - new_width;
//         int padding_y = new_shape.height - new_height;
//         cv::Scalar padding_color = cv::Scalar(100, 100, 100);

//         cv::Mat input_frame;
//         cv::resize(frame, input_frame, cv::Size(new_width, new_height), 0, 0, cv::INTER_AREA);
//         cv::copyMakeBorder(input_frame, input_frame, 0, padding_y, 0, padding_x, cv::BORDER_CONSTANT, padding_color);

//         // Menghitung rasio x dan y
//         float ratio_x = (float)frame.cols / (float)(input_frame.cols - padding_x);
//         float ratio_y = (float)frame.rows / (float)(input_frame.rows - padding_y);

//         // Inferensi dengan OpenVINO
//         ov::Tensor input_tensor(ov::element::u8, {1, INPUT_HEIGHT, INPUT_WIDTH, 3}, input_frame.data);
//         infer_request.set_input_tensor(input_tensor);
//         infer_request.infer();

//         // Menangani output model
//         const ov::Tensor& output_tensor = infer_request.get_output_tensor();
//         ov::Shape output_shape = output_tensor.get_shape();
//         float *detections = output_tensor.data<float>();

//         // Post-processing
//         std::vector<cv::Rect> boxes;
//         std::vector<int> class_ids;
//         std::vector<float> confidences;

//         for (int i = 0; i < output_shape[1]; i++)
//         {
//             float *detection = detections + (i * output_shape[2]);
//             float confidence = detection[4];
//             if (confidence < CONF_THRESH)
//                 continue;

//             float *class_scores = detection + 5;
//             cv::Mat scores(1, N_CLASSES, CV_32FC1, class_scores);
//             cv::Point class_id;
//             double max_class_score;
//             cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

//             if (max_class_score < SCORE_THRESH)
//                 continue;

//             float x = detection[0];
//             float y = detection[1];
//             float w = detection[2];
//             float h = detection[3];
//             float x_min = x - (w / 2);
//             float y_min = y - (h / 2);

//             confidences.push_back(confidence);
//             class_ids.push_back(class_id.x);
//             boxes.push_back(cv::Rect(x_min, y_min, w, h));
//         }

//         // NMS untuk mengurangi duplikasi deteksi
//         std::vector<int> nms_result;
//         cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESH, NMS_THRESH, nms_result);

//         std::vector<Detection> output;
//         for (int &i : nms_result)
//         {
//             Detection d;
//             d.class_ = class_ids[i];
//             d.conf = confidences[i];
//             d.box = boxes[i];
//             output.push_back(d);
//         }

//         // Menampilkan hasil hanya jika label "Flare", "Baskom", atau "Celana" terdeteksi
//         for (Detection &d : output)
//         {
//             if (d.class_ == 0 || d.class_ == 1 || d.class_ == 2)  // "Flare", "Baskom", "Celana"
//             {
//                 cv::Rect &box = d.box;
//                 box.x /= ratio_x;
//                 box.y /= ratio_y;
//                 box.width /= ratio_x;
//                 box.height /= ratio_y;

//                 cv::Scalar color = cv::Scalar(255, 0, 0); // Warna kotak (red)
//                 float color_mean = cv::mean(color)[0];
//                 cv::Scalar text_color = color_mean > 0.5 ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255);

//                 cv::rectangle(frame, cv::Point(box.x, box.y), cv::Point(box.x + box.width, box.y + box.height), color * 255, 2);
//                 int baseline = 0;
//                 char text[256];
//                 sprintf(text, "%s %0.1f%%", coconame[d.class_], d.conf * 100);
//                 cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
//                 cv::Scalar text_background_color = color * 0.7 * 255;

//                 cv::rectangle(frame, cv::Rect(cv::Point(box.x, box.y), cv::Size(label_size.width, label_size.height + baseline)), text_background_color, -1);
//                 cv::putText(frame, text, cv::Point(box.x, box.y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1);
//             }
//         }

//         // Menampilkan hasil di OpenCV
//         cv::imshow("Detection Results", frame);
//         cv::waitKey(1);  // Pastikan menunggu input untuk memperbarui jendela
//     }
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<VideoSubscriberNode>());
//     rclcpp::shutdown();
//     return 0;
// }

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

// Label yang digunakan dalam model
const char* coconame[] = { "Flare", "Baskom", "Celana", "Any" };

// Konstanta terkait inferensi dan deteksi
#define N_CLASSES 4 // Jumlah kelas sesuai dengan label
#define INPUT_WIDTH 640
#define INPUT_HEIGHT 640
#define CONF_THRESH 0.4
#define SCORE_THRESH 0.4
#define NMS_THRESH 0.4

struct Detection
{
    float conf;
    int class_;
    cv::Rect box;
};

class VideoSubscriberNode : public rclcpp::Node
{
public:
    ov::CompiledModel compiled_model;
    ov::InferRequest infer_request; 
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    VideoSubscriberNode() : Node("video_subscriber_node")
    {
        // Publisher untuk topik deteksi gambar
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/detection_results", 10);

        // Subscriber untuk topik kamera
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10,
            std::bind(&VideoSubscriberNode::image_callback, this, std::placeholders::_1));

        ov::Core core;
        std::shared_ptr<ov::Model> model = core.read_model("src/yolonih/assets/best.onnx"); // ONNX / PYTORCH / OPENVINO / etc.

        ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
        ppp.input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR);
        ppp.input().preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB);
        ppp.input().model().set_layout("NCHW");
        ppp.output(0).tensor().set_element_type(ov::element::f32);
        model = ppp.build();

        // Kompilasi model untuk inferensi
        compiled_model = core.compile_model(model, "CPU"); // CPU / CPU_ARM / GPU / dGPU / NPU / AUTO / BATCH / HETERO
        infer_request = compiled_model.create_infer_request();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image message to OpenCV Mat
        cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

        if (frame.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Received empty image.");
            return;
        }

        // Preprocessing
        float width = frame.cols;
        float height = frame.rows;
        cv::Size new_shape = cv::Size(INPUT_WIDTH, INPUT_HEIGHT);
        float ratio = float(new_shape.width / std::max(width, height));
        int new_width = int(std::round(width * ratio));
        int new_height = int(std::round(height * ratio));

        int padding_x = new_shape.width - new_width;
        int padding_y = new_shape.height - new_height;
        cv::Scalar padding_color = cv::Scalar(100, 100, 100);

        cv::Mat input_frame;
        cv::resize(frame, input_frame, cv::Size(new_width, new_height), 0, 0, cv::INTER_AREA);
        cv::copyMakeBorder(input_frame, input_frame, 0, padding_y, 0, padding_x, cv::BORDER_CONSTANT, padding_color);

        // Menghitung rasio x dan y
        float ratio_x = (float)frame.cols / (float)(input_frame.cols - padding_x);
        float ratio_y = (float)frame.rows / (float)(input_frame.rows - padding_y);

        // Inferensi dengan OpenVINO
        ov::Tensor input_tensor(ov::element::u8, {1, INPUT_HEIGHT, INPUT_WIDTH, 3}, input_frame.data);
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();

        // Menangani output model
        const ov::Tensor& output_tensor = infer_request.get_output_tensor();
        ov::Shape output_shape = output_tensor.get_shape();
        float *detections = output_tensor.data<float>();

        // Post-processing
        std::vector<cv::Rect> boxes;
        std::vector<int> class_ids;
        std::vector<float> confidences;

        for (int i = 0; i < output_shape[1]; i++)
        {
            float *detection = detections + (i * output_shape[2]);
            float confidence = detection[4];
            if (confidence < CONF_THRESH)
                continue;

            float *class_scores = detection + 5;
            cv::Mat scores(1, N_CLASSES, CV_32FC1, class_scores);
            cv::Point class_id;
            double max_class_score;
            cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

            if (max_class_score < SCORE_THRESH)
                continue;

            float x = detection[0];
            float y = detection[1];
            float w = detection[2];
            float h = detection[3];
            float x_min = x - (w / 2);
            float y_min = y - (h / 2);

            confidences.push_back(confidence);
            class_ids.push_back(class_id.x);
            boxes.push_back(cv::Rect(x_min, y_min, w, h));
        }

        // NMS untuk mengurangi duplikasi deteksi
        std::vector<int> nms_result;
        cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESH, NMS_THRESH, nms_result);

        std::vector<Detection> output;
        for (int &i : nms_result)
        {
            Detection d;
            d.class_ = class_ids[i];
            d.conf = confidences[i];
            d.box = boxes[i];
            output.push_back(d);
        }

        // Menampilkan hasil hanya jika label "Flare", "Baskom", atau "Celana" terdeteksi
        for (Detection &d : output)
        {
            if (d.class_ == 0 || d.class_ == 1 || d.class_ == 2)  // "Flare", "Baskom", "Celana"
            {
                cv::Rect &box = d.box;
                box.x /= ratio_x;
                box.y /= ratio_y;
                box.width /= ratio_x;
                box.height /= ratio_y;

                cv::Scalar color = cv::Scalar(255, 0, 0); // Warna kotak (red)
                float color_mean = cv::mean(color)[0];
                cv::Scalar text_color = color_mean > 0.5 ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255);

                cv::rectangle(frame, cv::Point(box.x, box.y), cv::Point(box.x + box.width, box.y + box.height), color * 255, 2);
                int baseline = 0;
                char text[256];
                sprintf(text, "%s %0.1f%%", coconame[d.class_], d.conf * 100);
                cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
                cv::Scalar text_background_color = color * 0.7 * 255;

                cv::rectangle(frame, cv::Rect(cv::Point(box.x, box.y), cv::Size(label_size.width, label_size.height + baseline)), text_background_color, -1);
                cv::putText(frame, text, cv::Point(box.x, box.y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1);
            }
        }

        // Mengonversi gambar OpenCV ke pesan sensor_msgs::msg::Image
        sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

        // Publikasi hasil deteksi
        image_publisher_->publish(*output_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}

