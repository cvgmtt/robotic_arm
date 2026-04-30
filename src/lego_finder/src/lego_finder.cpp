#include "lego_finder.hpp"

LegoFinder::LegoFinder(): Node("lego_finder"){
    image_receiver.subscribe(this, "/rgbd_camera/image", rmw_qos_profile_sensor_data);
    depth_receiver.subscribe(this, "/rgbd_camera/depth_image", rmw_qos_profile_sensor_data);
    sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), image_receiver, depth_receiver);
    sync->registerCallback(std::bind(&LegoFinder::sub_callback, this, _1, _2));

    service = this->create_service<interfaces::srv::Poses>("get_legos", std::bind(&LegoFinder::service_callback, this, _1, _2));


    //change this string
    std::string model_path = "/home/matteo/Desktop/Projects/robotic_arm/src/lego_finder/include/best.onnx";
    
    env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "lego_finder");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session = std::make_unique<Ort::Session>(*env, model_path.c_str(), session_options);
}

void LegoFinder::sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr image, const sensor_msgs::msg::Image::ConstSharedPtr depth){
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image = image;
    latest_depth = depth;    
    RCLCPP_INFO(this->get_logger(), "Immagine ricevuta! Altezza: %d", latest_image->height);
}

void LegoFinder::service_callback(const std::shared_ptr<interfaces::srv::Poses::Request> request, std::shared_ptr<interfaces::srv::Poses::Response> response) {
    // --- lock zone (invariata) ---
    sensor_msgs::msg::Image::ConstSharedPtr image_to_process;
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        if (latest_image == nullptr) {
            RCLCPP_WARN(this->get_logger(), "No image received yet");
            response->success = false;
            return;
        }
        image_to_process = latest_image;
    }

    cv::Mat cv_image = cv_bridge::toCvCopy(image_to_process, "bgr8")->image;
    
    // 1. DA BGR A RGB (YOLO vuole RGB)
    cv::Mat rgb_image;
    cv::cvtColor(cv_image, rgb_image, cv::COLOR_BGR2RGB);

    // 2. CREIAMO IL BLOB MAGICO
    // blobFromImage si occupa di:
    // - Ridimensionare a 640x640
    // - Dividere per 255.0 (normalizzazione)
    // - Convertire da HWC a CHW
    // - Gestire la memoria in modo continuo
    cv::Mat blob = cv::dnn::blobFromImage(rgb_image, 1.0/255.0, cv::Size(640, 640), cv::Scalar(), true, false);

    // 3. PREPARIAMO IL TENSORE PER ONNX
    std::vector<int64_t> input_shape = {1, 3, 640, 640};
    
    // Il blob è già nel formato perfetto CHW, prendiamo i dati grezzi
    size_t input_tensor_size = 1 * 3 * 640 * 640; 
    
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        reinterpret_cast<float*>(blob.data), // Passiamo i dati puliti del blob!
        input_tensor_size,
        input_shape.data(),
        input_shape.size()
    );

    // --- inferenza ---
    const char* input_names[] = {"images"};
    const char* output_names[] = {"output0"};
        auto outputs = session->Run(
        Ort::RunOptions{nullptr},
        input_names, &input_tensor, 1,
        output_names, 1
    );

    // --- parsing output YOLO: shape [1, 5+num_classes, 8400] ---
    float* raw = outputs[0].GetTensorMutableData<float>();
    auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    int num_predictions = shape[2];   // 8400
    int row_size = shape[1];          // 4 + num_classes

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    float conf_threshold = 0.25f;

    for (int i = 0; i < num_predictions; i++) {
        // output è [1, row_size, 8400]: accesso colonna i
        float cx = raw[0 * num_predictions + i];
        float cy = raw[1 * num_predictions + i];
        float w  = raw[2 * num_predictions + i];
        float h  = raw[3 * num_predictions + i];

        // trova la classe con score massimo
        float max_score = 0.0f;
        int best_class = 0;
        for (int c = 0; c < row_size - 4; c++) {
            float score = raw[(4 + c) * num_predictions + i];
            if (score > max_score) {
                max_score = score;
                best_class = c;
            }
        }

        if (max_score > conf_threshold) {
            // scala da 640x640 alle dimensioni reali
            float scale_x = (float)cv_image.cols / 640.0f;
            float scale_y = (float)cv_image.rows / 640.0f;
            int x = (int)((cx - w / 2.0f) * scale_x);
            int y = (int)((cy - h / 2.0f) * scale_y);
            int bw = (int)(w * scale_x);
            int bh = (int)(h * scale_y);

            boxes.push_back(cv::Rect(x, y, bw, bh));
            confidences.push_back(max_score);
            class_ids.push_back(best_class);
        }
    }

    // --- NMS ---
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, 0.4f, indices);

    for (int idx : indices) {
        RCLCPP_INFO(this->get_logger(),
            "Lego classe %d | pixel X:%d Y:%d W:%d H:%d | conf %.2f",
            class_ids[idx], boxes[idx].x, boxes[idx].y,
            boxes[idx].width, boxes[idx].height, confidences[idx]);
    }
    for (int idx : indices) {
    cv::rectangle(cv_image, boxes[idx], cv::Scalar(0, 255, 0), 2);
    
    std::string label = "Classe " + std::to_string(class_ids[idx]) 
                      + " " + std::to_string((int)(confidences[idx] * 100)) + "%";
    
    cv::putText(cv_image, label,
        cv::Point(boxes[idx].x, boxes[idx].y - 10),
        cv::FONT_HERSHEY_SIMPLEX, 0.5,
        cv::Scalar(0, 255, 0), 1);
    }

    cv::imwrite("/tmp/lego_debug.png", cv_image);

    response->success = true;
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LegoFinder>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}