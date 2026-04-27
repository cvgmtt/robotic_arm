#ifndef __LEGO_FINDER_HPP__
#define __LEGON_FINDER_HPP__
#include <stdlib.h>
#include <memory.h>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <interfaces/srv/poses.hpp>

using std::placeholders::_1; 
using std::placeholders::_2;

class LegoFinder: public rclcpp::Node{
    private:
        rclcpp::Service<interfaces::srv::Poses>::SharedPtr service;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_receiver;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_receiver;
        //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr intrinsics_receiver;


        sensor_msgs::msg::Image::ConstSharedPtr latest_image;
        std::mutex image_mutex_;
    public:
        LegoFinder(): Node("lego_finder"){
            image_receiver = this->create_subscription<sensor_msgs::msg::Image>("/rgbd_camera/image", 10, std::bind(&LegoFinder::sub_callback, this, _1));
            depth_receiver = this->create_subscription<sensor_msgs::msg::Image>("/rgbd_camera/depth_image", 10, std::bind(&LegoFinder::sub_callback, this, _1));


            service = this->create_service<interfaces::srv::Poses>("get_legos", std::bind(&LegoFinder::service_callback, this, _1, _2));
        }

        void sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void service_callback(const std::shared_ptr<interfaces::srv::Poses::Request> request,
                                std::shared_ptr<interfaces::srv::Poses::Response> response);
};


#endif