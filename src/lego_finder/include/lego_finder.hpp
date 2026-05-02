#ifndef __LEGO_FINDER_HPP__
#define __LEGON_FINDER_HPP__
#include <stdlib.h>
#include <memory.h>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp> 
#include <interfaces/srv/poses.hpp>
#include "message_filters/subscriber.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <onnxruntime/core/session/onnxruntime_cxx_api.h> //modulo per l'inference
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>

using std::placeholders::_1; 
using std::placeholders::_2;

class LegoFinder: public rclcpp::Node{
    private:
        rclcpp::Service<interfaces::srv::Poses>::SharedPtr service;
        message_filters::Subscriber<sensor_msgs::msg::Image> image_receiver;
        message_filters::Subscriber<sensor_msgs::msg::Image> depth_receiver;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr intrinsics_receiver;

        //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr intrinsics_receiver;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
        sensor_msgs::msg::Image::ConstSharedPtr latest_image;
        sensor_msgs::msg::Image::ConstSharedPtr latest_depth;
        double fx, fy, cx_cam, cy_cam;
        bool intrinsics_received = false;
        geometry_msgs::msg::PoseArray lego_poses;
    
        //variables for onnx runtime
        std::unique_ptr<Ort::Env> env;
        std::unique_ptr<Ort::Session> session;

        enum colorClasses{
            RED, GREEN, BLUE, YELLOW, WHITE, SCONOSCIUTO
        };

        std::mutex image_mutex_;
    public:
        LegoFinder();

        void sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr image, const sensor_msgs::msg::Image::ConstSharedPtr depth);
        void service_callback(const std::shared_ptr<interfaces::srv::Poses::Request> request,
                                std::shared_ptr<interfaces::srv::Poses::Response> response);
        void intrinsics_callback(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
        int detectColor(int b, int g, int r); 
        geometry_msgs::msg::Pose findPose(int &px, int &py, const int &crop_value);
};


#endif