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
        message_filters::Subscription<sensor_msgs::msg::Image>::SharedPtr image_receiver;
        message_filters::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_receiver;
        //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr intrinsics_receiver;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
        sensor_msgs::msg::Image::ConstSharedPtr latest_image;
        sensor_msgs::msg::Image::ConstSharedPtr latest_depth;

        std::mutex image_mutex_;
    public:
        LegoFinder();

        void sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void service_callback(const std::shared_ptr<interfaces::srv::Poses::Request> request,
                                std::shared_ptr<interfaces::srv::Poses::Response> response);
};


#endif