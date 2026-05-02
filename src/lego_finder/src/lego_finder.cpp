#include "lego_finder.hpp"

LegoFinder::LegoFinder(): Node("lego_finder"){
    image_receiver.subscribe(this, "/rgbd_camera/image", rmw_qos_profile_sensor_data);
    depth_receiver.subscribe(this, "/rgbd_camera/depth_image", rmw_qos_profile_sensor_data);
    intrinsics_receiver = this->create_subscription<sensor_msgs::msg::CameraInfo>("/rgbd_camera/camera_info", rclcpp::SensorDataQoS(), 
        std::bind(&LegoFinder::intrinsics_callback, this, _1));

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


int LegoFinder::detectColor(int b, int g, int r) {
    // 1. Identifica il Bianco: tutti i canali sono alti e vicini tra loro

    if (b > 180 && g > 180 && r > 180) {
        return 4;
    }

    // 2. Logica per colori primari basata sulla predominanza
    // Rosso: R è nettamente superiore a G e B
    if (r > g * 1.5 && r > b * 1.5) {
        return 2;
    }

    // Blu: B è nettamente superiore a R e G
    if (b > r * 1.5 && b > g * 1.5) {
        return 0;
    }

    // Verde: G è nettamente superiore a R e B
    if (g > r * 1.2 && g > b * 1.2) {
        return 1;
    }

    // Giallo: R e G sono entrambi alti, B è basso
    if (r > 150 && g > 150 && b < 100) {
        return 3;
    }
    return 5;
}

    geometry_msgs::msg::Pose LegoFinder::findPose(int &px, int &py, const int &crop_value){
        cv::Mat depth_image;
        geometry_msgs::msg::Pose p;
        try {
            depth_image = cv_bridge::toCvCopy(latest_depth, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Errore cv_bridge (Depth): %s", e.what());
            return p;
        }
        RCLCPP_INFO(this->get_logger(), "nella funzione");

        int px_original = px + crop_value;
        int py_original = py;

        double Z = depth_image.at<float>(py_original, px_original);

        if (std::isnan(Z) || std::isinf(Z) || Z <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Profondità non valida (Z=%f) al pixel (%d, %d)", Z, px_original, py_original);
            return p;  //scarta la posa vuota nel servizio)
        }
        double X = ((double) px_original - cx_cam) * Z/fx;
        double Y = ((double) py_original - cy_cam) * Z/fy;

        p.position.x = X;
        p.position.y = Y;
        p.position.z = Z;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);

        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();


        RCLCPP_INFO(this->get_logger(), "posizione di x: %f", p.position.x);
        return p;

    }


void LegoFinder::sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr image, const sensor_msgs::msg::Image::ConstSharedPtr depth){
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image = image;
    latest_depth = depth;   
}

void LegoFinder::intrinsics_callback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg){
    if(intrinsics_received){
        return;
    }
    fx = msg->k[0] * 2.0;       
    cx_cam = msg->k[2] * 2.0;   
    fy = msg->k[4] * 2.0;       
    cy_cam = msg->k[5] * 2.0;   

    intrinsics_received = true;
    RCLCPP_INFO(this->get_logger(), "Intrinsics Ricevuti! fx:%.1f, fy:%.1f, cx:%.1f, cy:%.1f", fx, fy, cx_cam, cy_cam);
    return;
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

    using std::chrono::high_resolution_clock;
    using std::chrono::duration;
    auto start_preprocess = high_resolution_clock::now();

    cv::Mat cv_image = cv_bridge::toCvCopy(image_to_process, "bgr8")->image;

    // crop della fascia grigia
    const int crop_value = 230;
    cv::Mat cropped = cv_image(cv::Rect(230, 0, cv_image.cols - crop_value, cv_image.rows));

    // blob - swapRB=true per convertire BGR→RGB che vuole YOLO
    cv::Mat blob = cv::dnn::blobFromImage(cropped, 1.0/255.0, cv::Size(640, 640), cv::Scalar(), true, false);

    //creiamo il vettore per salvare le classi di colori
    std::vector<int>color_id;


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

    auto end_preprocess = high_resolution_clock::now();

    // --- inferenza ---
    const char* input_names[] = {"images"};
    const char* output_names[] = {"output0"};
        auto outputs = session->Run(
        Ort::RunOptions{nullptr},
        input_names, &input_tensor, 1,
        output_names, 1
    );

    auto end_inference = high_resolution_clock::now();

    // --- parsing output YOLO: shape [1, 5+num_classes, 8400] ---
    float* raw = outputs[0].GetTensorMutableData<float>();
    auto shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    int num_predictions = shape[2];   // 8400
    int row_size = shape[1];          // 4 + num_classes

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    std::vector<std::vector<int>> pixel_coordinates;


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
                float scale_x = (float)cropped.cols / 640.0f;
                float scale_y = (float)cropped.rows / 640.0f;
                int x = (int)((cx - w / 2.0f) * scale_x);
                int y = (int)((cy - h / 2.0f) * scale_y);
                int bw = (int)(w * scale_x);
                int bh = (int)(h * scale_y);
                boxes.push_back(cv::Rect(x, y, bw, bh));
                std::vector<int> temp_vect;
                temp_vect.push_back(static_cast<int>(cx * scale_x));
                temp_vect.push_back(static_cast<int>(cy * scale_y));
                pixel_coordinates.push_back(temp_vect);
                confidences.push_back(max_score);
                class_ids.push_back(best_class);
            }
        }

    // --- NMS ---
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, 0.4f, indices);
    
    
    //color matching:
    for(int idx : indices){

        int px = pixel_coordinates[idx][0];
        int py = pixel_coordinates[idx][1];
        cv::Vec3b pixel = cropped.at<cv::Vec3b>(py, px);
        int blue = pixel[0];
        int green = pixel[1];
        int red = pixel[2];
        color_id.push_back(detectColor(blue, green, red));

        lego_poses.header.stamp = this->get_clock()->now(); // timestamp of creation of the msg
        lego_poses.header.frame_id = "map"; // frame id in which the array is published
        lego_poses.poses.push_back(findPose(px, py, crop_value));
    }
    int counter = 0;
    for (int idx : indices) {
        RCLCPP_INFO(this->get_logger(),
            "Lego classe %d | pixsrc/lego_finder/third_party/el X:%d Y:%d W:%d H:%d Color: %d | conf %.2f",
            class_ids[idx], boxes[idx].x, boxes[idx].y,
            boxes[idx].width, boxes[idx].height, color_id[counter], confidences[counter]);
            counter++;
    }
    for (int idx : indices) {
    cv::rectangle(cropped, boxes[idx], cv::Scalar(0, 255, 0), 2);
    
    std::string label = "Classe " + std::to_string(class_ids[idx]) 
                      + " " + std::to_string((int)(confidences[idx] * 100));

    
    // cv::putText(cv_image, label,
    //     cv::Point(boxes[idx].x, boxes[idx].y - 10),
    //     cv::FONT_HERSHEY_SIMPLEX, 0.5,
    //     cv::Scalar(0, 255, 0), 1);
    // 
    }
    auto end_postprocess = high_resolution_clock::now();

    cv::imwrite("/tmp/lego_debug.png", cropped);

    duration<double, std::milli> time_preprocess = end_preprocess - start_preprocess;
    duration<double, std::milli> time_inference = end_inference - end_preprocess;
    duration<double, std::milli> time_postprocess = end_postprocess - end_inference;
    duration<double, std::milli> time_total = end_postprocess - start_preprocess;

    // Stampiamo a schermo nel formato stile Python!
    RCLCPP_INFO(this->get_logger(), 
        "Speed: %.2fms preprocess, %.2fms inference, %.2fms postprocess per image (Totale: %.2fms)",
        time_preprocess.count(), time_inference.count(), time_postprocess.count(), time_total.count());


    response->success = true;
    response->lego_poses = lego_poses;
    response->color_ids = color_id;
}



int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LegoFinder>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}