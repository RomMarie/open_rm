#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

int main(int argc,char ** argv){

    ros::init(argc, argv, "rm_cam_to_img");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/input_image", 1);

    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH,1024);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,768);
    // Check if video device can be opened with the given index
    if(!cap.isOpened())
    {
        ROS_ERROR_STREAM("Erreur lors de la connexion a la camera");
        ros::shutdown();
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(20.);

    while (ros::ok()) {
      cap >> frame;
      ROS_INFO_STREAM(frame.rows<<" "<<frame.cols);
      // Check if grabbed frame is actually full with some content
      if(!frame.empty()) {
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
}
