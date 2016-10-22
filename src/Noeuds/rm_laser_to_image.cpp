#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>

class worker{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber sub_;
    image_transport::Publisher pub_;
    ros::Subscriber sub_laser_;
    cv::Mat img_;
    tf::TransformListener listener_;
    laser_geometry::LaserProjection projector_;
    std::string camera_frame_,laser_frame_;
    image_geometry::PinholeCameraModel cam_model_;
public:
    worker():nh_("~"),it_(nh_),img_(0,0,CV_8U)
    {
        sub_ = it_.subscribeCamera("/image", 1, &worker::cbImg, this);
        sub_laser_ = nh_.subscribe("/laser", 1, &worker::cbLaser, this);
        pub_ = it_.advertise("/image_out",1);
        camera_frame_=nh_.resolveName("/camera_frame");
        laser_frame_=nh_.resolveName("/laser_frame");
    }

    void cbImg(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        cam_model_.fromCameraInfo(info_msg); // On définit les paramètres de notre caméra
        cv_bridge::CvImagePtr input_bridge;
        try {
            input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            img_ = input_bridge->image;
        }
        catch (cv_bridge::Exception& ex){
            ROS_ERROR_STREAM(ex.what());
            return;
        }
    }

    void cbLaser(const sensor_msgs::LaserScanPtr& msg)
    {
        sensor_msgs::PointCloud cloud;
        try
        {
            listener_.waitForTransform(camera_frame_,laser_frame_,ros::Time::now(),ros::Duration(0.3));
            msg->header.frame_id=laser_frame_; // Pas beau, mais seul moyen de permettre plusieurs lasers depuis le simulateur
            projector_.transformLaserScanToPointCloud(camera_frame_,*msg, cloud,listener_);
        }
        catch (tf::TransformException& e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }

        try
        {
            for (std::vector<geometry_msgs::Point32>::iterator it(cloud.points.begin());it!=cloud.points.end();it++)
            {
                cv::Point3d pt3D(it->x,it->y,it->z);
                cv::Point2d uv;
                if(it->z>0){
                    uv=cam_model_.project3dToPixel(pt3D);
                    cv::circle(img_,uv,2,cv::Scalar(255,255,255),CV_FILLED);
                }
            }
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_).toImageMsg();
            pub_.publish(msg);
        }
        catch (cv::Exception& e)
        {
            ROS_WARN_STREAM(e.what());
            return;
        }
    }
};

int main(int argc,char ** argv){
    ros::init(argc,argv,"laser_to_image");
    ros::NodeHandle nh("~");

    worker Worker;

    ros::spin();

}
