#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <open_rm/AxeMedian/fdeltama.h>

cv::Mat mask,skel;
double delta;

image_transport::Publisher pub;

void callBack(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        mask=cv_bridge::toCvShare(msg, "mono8")->image;
        skel=rm::AxeMedian::FDeltaMA::ma(mask,delta);
        cv::imshow("skel",skel);cv::waitKey(1);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", skel).toImageMsg();
        pub.publish(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR_STREAM("Could not convert from '"<<msg->encoding.c_str()<<"' to 'mono8'.");
    }
}

int main(int argc, char** argv){
    ros::init(argc,argv,"rm_mask_to_skel");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    nh.param("delta",delta,1.);

    ros::Rate loop_rate(100.);

    image_transport::Subscriber sub=it.subscribe("/input_mask",1,&callBack);
    pub=it.advertise("/skel",1);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
