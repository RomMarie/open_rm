#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

void callBack(const sensor_msgs::ImageConstPtr& msg){
    try
    {
      cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR_STREAM("Could not convert from '"<<msg->encoding.c_str()<<"' to 'bgr8'.");
    }
}

int main(int argc, char** argv){
    ros::init(argc,argv,"rm_mask_to_skel");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    ros::Rate loop_rate(100.);

    image_transport::Subscriber sub=it.subscribe("input_mask",1,&callBack);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
