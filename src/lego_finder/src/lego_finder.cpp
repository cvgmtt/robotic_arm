#include "lego_finder.hpp"

LegoFinder::LegoFinder(): Node("lego_finder"){
            image_receiver.subscribe(this, "/rgbd_camera/image");
            depth_receiver.subscribe(this, "/rgbd_camera/depth_image");
            uint32_t queue_size = 10;
            sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(image_receiver, depth_receiver, queue_size);
            sync->registerCallback(std::bind(&LegoFinder::sub_callback, this, _1, _2));

            service = this->create_service<interfaces::srv::Poses>("get_legos", std::bind(&LegoFinder::service_callback, this, _1, _2));
        }

void LegoFinder::sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr image, const sensor_msgs::msg::Image::ConstSharedPtr depth){
    std::lock_guard<std::mutex> lock(image_mutex_);
    latest_image = image;
    latest_depth = depth;    
    RCLCPP_INFO(this->get_logger(), "Immagine ricevuta! Altezza: %d", latest_image->height);
}

void LegoFinder::service_callback(const std::shared_ptr<interfaces::srv::Poses::Request> request, std::shared_ptr<interfaces::srv::Poses::Response> response){
    //lock zone    
    {
        sensor_msgs::msg::Image::ConstSharedPtr image_to_process;
        sensor_msgs::msg::Image::ConstSharedPtr depth_to_process;

        std::lock_guard<std::mutex> lock(image_mutex_);
        if(latest_image != nullptr && latest_depth != nullptr){
            image_to_process = latest_image;
            depth_to_process = latest_depth;
        } else{
            RCLCPP_WARN(this->get_logger(), "No image received yet");
            response->success = false;
            return;
        }
    }   

        //synch depth and image
        //opencv to get colors
        //get u and v from yolo, get z from depth camera
        //get intrinsics subscribing to camera_info and calculate x,y,z
        //understand quaternions
        //send the pose 

}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LegoFinder>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}