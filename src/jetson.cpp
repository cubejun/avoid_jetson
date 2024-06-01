#include "avoid_jetson/jetson.hpp"
Pub::Pub() : Node("campub")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile );
    timer_ = this->create_wall_timer(25ms, std::bind(&Pub::publish_msg, this));
    cap.open(src, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video!");
        rclcpp::shutdown();
        return;
    }

    if(!dxl.open())
    {
        RCLCPP_ERROR(this->get_logger(), "dynamixel open error");
        rclcpp::shutdown();
        //return -1;
    } 
    std::function<void(const std_msgs::msg::Int32::SharedPtr msg)> fn;
    fn = std::bind(&Pub::mysub_callback, this, dxl, _1);
    sub_ = this->create_subscription<std_msgs::msg::Int32>("err", qos_profile,fn);
}
void Pub::publish_msg()
{
    cap >> frame;
    if (frame.empty()) { RCLCPP_ERROR(this->get_logger(), "frame empty"); return;}
    msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
    pub_->publish(*msg);
}

void Pub::mysub_callback(Dxl& mdxl, const std_msgs::msg::Int32::SharedPtr intmsg)
{
    int chk = 0;
    err = intmsg->data;
    if(err == 100){
        lvel = 100;//왼쪽 바퀴 속도
	    rvel = 100;//오른쪽 바퀴 속도
    }
    else  if(err == 200){
        lvel = 50;//왼쪽 바퀴 속도
	    rvel = 50;//오른쪽 바퀴 속도
    }
    else  if(err == 300){
        lvel = -50;//왼쪽 바퀴 속도
	    rvel = -50;//오른쪽 바퀴 속도
    }
    else  if(err == 400){
        lvel = 100;//왼쪽 바퀴 속도
	    rvel = 100;//오른쪽 바퀴 속도
    }
    else  if(err == 500){
        lvel = -100;//왼쪽 바퀴 속도
	    rvel = -100;//오른쪽 바퀴 속도
    }
    else {
        lvel = 50 - gain * -err;//왼쪽 바퀴 속도
	    rvel = -(52 + gain * -err);//오른쪽 바퀴 속도
        chk = 0;
    }

    RCLCPP_INFO(this->get_logger(), "Received message: %d %d", lvel, rvel);
    mdxl.setVelocity(lvel, rvel);
}
