#include "nodes/camera.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cmath>
#include <string>

namespace nodes {

// COCO 80 tried (YOLO poradie)
static const std::vector<std::string> COCO_NAMES = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
    "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
    "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};

CameraNode::CameraNode() : Node("camera_node"), coco_names_(COCO_NAMES) {
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->declare_parameter<bool>("use_compressed", false);
    this->declare_parameter<bool>("publish_compressed", true);
    this->declare_parameter<std::string>("compressed_topic", "/camera/compressed");
    this->declare_parameter<std::string>("yolo_model", "");
    this->declare_parameter<double>("conf_threshold", 0.45);
    this->declare_parameter<double>("nms_threshold", 0.4);
    this->declare_parameter<int>("input_size", 640);

    image_topic_ = this->get_parameter("image_topic").as_string();
    use_compressed_ = this->get_parameter("use_compressed").as_bool();
    publish_compressed_ = this->get_parameter("publish_compressed").as_bool();
    std::string compressed_topic = this->get_parameter("compressed_topic").as_string();
    conf_threshold_ = static_cast<float>(this->get_parameter("conf_threshold").as_double());
    nms_threshold_ = static_cast<float>(this->get_parameter("nms_threshold").as_double());
    input_size_ = this->get_parameter("input_size").as_int();

    pub_objects_ = this->create_publisher<std_msgs::msg::String>("/detected_objects", 10);
    pub_people_count_ = this->create_publisher<std_msgs::msg::Int32>("/detected_people", 10);
    if (publish_compressed_)
        pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(compressed_topic, 10);

    std::string model_path = this->get_parameter("yolo_model").as_string();
    // Placeholder cesta -> pouzit model z balika (models/yolov5n.onnx)
    if (model_path.empty() || model_path.find("/cesta/") != std::string::npos)
        model_path = "models/yolov5n.onnx";
    if (!model_path.empty() && model_path[0] != '/') {
        try {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("waverower");
            if (!pkg_share.empty())
                model_path = pkg_share + "/" + model_path;
        } catch (...) {}
    }
    if (!model_path.empty()) {
        try {
            net_ = cv::dnn::readNet(model_path);
            net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            model_loaded_ = true;
            RCLCPP_INFO(this->get_logger(), "YOLO model nacitany: %s", model_path.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "YOLO model sa nenacital (%s). Spustam bez detekcie.", e.what());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Parameter yolo_model prazdny. Nastav cestu k .onnx.");
    }

    if (use_compressed_) {
        sub_compressed_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            image_topic_.empty() ? "/camera/compressed" : image_topic_, 10,
            std::bind(&CameraNode::compressed_callback, this, std::placeholders::_1));
    } else {
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_.empty() ? "/camera/image_raw" : image_topic_, 10,
            std::bind(&CameraNode::image_callback, this, std::placeholders::_1));
    }
    RCLCPP_INFO(this->get_logger(), "Camera: %s (YOLO=%d)", image_topic_.c_str(), model_loaded_ ? 1 : 0);
}

void CameraNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat frame = cv_ptr->image.clone();
        if (frame.empty()) return;
        process_frame(frame);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
    }
}

void CameraNode::compressed_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;
        process_frame(frame);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CompressedImage: %s", e.what());
    }
}

void CameraNode::detect_yolo(cv::Mat& frame, std::vector<cv::Rect>& boxes, std::vector<int>& class_ids, std::vector<float>& scores) {
    boxes.clear();
    class_ids.clear();
    scores.clear();
    if (!model_loaded_ || net_.empty()) return;

    cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0, cv::Size(input_size_, input_size_), cv::Scalar(), true, false);
    net_.setInput(blob);
    std::vector<cv::Mat> outs;
    net_.forward(outs, net_.getUnconnectedOutLayersNames());

    float scale_x = static_cast<float>(frame.cols) / input_size_;
    float scale_y = static_cast<float>(frame.rows) / input_size_;
    std::vector<cv::Rect> all_boxes;
    std::vector<int> all_class_ids;
    std::vector<float> all_scores;

    // YOLOv5 ONNX: vystup (1, 25200, 85) alebo (1, 85, 25200)
    for (const cv::Mat& out : outs) {
        if (out.dims != 3) continue;
        const int d0 = out.size[0], d1 = out.size[1], d2 = out.size[2];
        int num_det;
        int num_classes;
        bool transposed;  // (1, 85, N) -> citame po stlpcoch
        if (d1 == 85 && d2 != 85) {
            num_det = d2;
            num_classes = 80;
            transposed = true;
        } else if (d2 == 85 || (d2 - 5 > 0 && d2 - 5 <= 80)) {
            num_det = d1;
            num_classes = d2 - 5;
            transposed = false;
        } else
            continue;
        if (num_classes <= 0 || num_classes > 80) continue;

        for (int i = 0; i < num_det; ++i) {
            float cx, cy, w, h, conf;
            int best_class = 0;
            float best_score = 0.f;
            if (transposed) {
                cx = out.ptr<float>(0)[i];
                cy = out.ptr<float>(1)[i];
                w = out.ptr<float>(2)[i];
                h = out.ptr<float>(3)[i];
                conf = out.ptr<float>(4)[i];
                best_score = out.ptr<float>(5)[i];
                for (int c = 1; c < num_classes; ++c) {
                    float s = out.ptr<float>(5 + c)[i];
                    if (s > best_score) { best_score = s; best_class = c; }
                }
            } else {
                const float* row = out.ptr<float>(0, i);
                cx = row[0]; cy = row[1]; w = row[2]; h = row[3];
                conf = row[4];
                best_score = row[5];
                for (int c = 1; c < num_classes; ++c) {
                    float s = row[5 + c];
                    if (s > best_score) { best_score = s; best_class = c; }
                }
            }
            if (conf < conf_threshold_) continue;
            float score = conf * best_score;
            if (score < conf_threshold_) continue;
            float fx = cx * scale_x;
            float fy = cy * scale_y;
            float fw = w * scale_x;
            float fh = h * scale_y;
            int x = static_cast<int>(fx - fw / 2.f);
            int y = static_cast<int>(fy - fh / 2.f);
            all_boxes.push_back(cv::Rect(x, y, static_cast<int>(fw), static_cast<int>(fh)));
            all_class_ids.push_back(best_class);
            all_scores.push_back(score);
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(all_boxes, all_scores, conf_threshold_, nms_threshold_, indices);
    for (int i : indices) {
        boxes.push_back(all_boxes[i]);
        class_ids.push_back(all_class_ids[i]);
        scores.push_back(all_scores[i]);
    }
}

void CameraNode::process_frame(cv::Mat& frame) {
    std::string objects_str;
    int people_count = 0;

    if (model_loaded_) {
        std::vector<cv::Rect> boxes;
        std::vector<int> class_ids;
        std::vector<float> scores;
        detect_yolo(frame, boxes, class_ids, scores);
        for (size_t i = 0; i < boxes.size(); ++i) {
            int c = class_ids[i];
            if (c >= 0 && c < static_cast<int>(coco_names_.size())) {
                if (!objects_str.empty()) objects_str += ",";
                objects_str += coco_names_[c] + ":" + std::to_string(static_cast<int>(std::round(scores[i] * 100)));
                if (coco_names_[c] == "person") people_count++;
            }
            cv::rectangle(frame, boxes[i], cv::Scalar(0, 255, 0), 2);
            std::string label = (c >= 0 && c < static_cast<int>(coco_names_.size())) ? coco_names_[c] : "?";
            cv::putText(frame, label, cv::Point(boxes[i].x, boxes[i].y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }
        if (!objects_str.empty()) {
            std_msgs::msg::String obj_msg;
            obj_msg.data = objects_str;
            pub_objects_->publish(obj_msg);
        }
    }

    std_msgs::msg::Int32 count_msg;
    count_msg.data = people_count;
    pub_people_count_->publish(count_msg);

    if (publish_compressed_ && pub_compressed_) {
        std::vector<uchar> buf;
        if (cv::imencode(".jpg", frame, buf)) {
            sensor_msgs::msg::CompressedImage out;
            out.format = "jpeg";
            out.data = buf;
            pub_compressed_->publish(out);
        }
    }
}

}  // namespace nodes
