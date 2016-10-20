#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>

int main(int argc,char ** argv){
    ros::init(argc,argv,"virtualcam_to_img");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_img=it.advertise("/virtual_image",1);

    ros::Publisher pub_info=nh.advertise<sensor_msgs::CameraInfo>("/camera_info",1);


    std::string frame;
    nh.param<std::string>("camera_frame",frame,"/camera_frame");

    double fps;
    nh.param("fps",fps,50.0);

    int width;
    nh.param("width",width,640);

    int height;
    nh.param("height",height,480);

    double u0,v0,alpha_u,alpha_v;
    nh.param("u0",u0,320.);
    nh.param("v0",v0,240.);
    nh.param("alpha_u",alpha_u,200.);
    nh.param("alpha_v",alpha_v,200.);

    sensor_msgs::CameraInfo camInfo;
    for(boost::array<double,12>::iterator it(camInfo.P.begin());it!=camInfo.P.end();it++)
        *it=0.0;
    camInfo.P[0]=alpha_u;
    camInfo.P[2]=u0;
    camInfo.P[5]=alpha_v;
    camInfo.P[6]=v0;
    camInfo.binning_x=1.0;
    camInfo.binning_y=1.0;
    camInfo.height=height;
    camInfo.width=width;
    camInfo.header.frame_id=frame;


    cv::Mat img=cv::Mat::zeros(height,width,CV_8UC3);
    ros::Rate loop_rate(fps);
    std_msgs::Header header;
    header.frame_id=frame;
    sensor_msgs::ImagePtr msg;
    while(ros::ok()){
        ros::Time now=ros::Time::now();
        camInfo.header.stamp=now;
        header.stamp=now;

        pub_info.publish(camInfo);
        msg = cv_bridge::CvImage(header, "rgb8", img).toImageMsg();
        pub_img.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
