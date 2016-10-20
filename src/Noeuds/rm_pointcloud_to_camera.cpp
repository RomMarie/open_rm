#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>


cv::Mat imgIn,imgOut;
image_geometry::PinholeCameraModel cam_model;
image_transport::Publisher pub_res;
tf::TransformListener* listener;

bool imgOk;
bool calibOk;


/*!
 * \brief Callback gérant la réception d'un nuage de points
 * \param msg Nuage de points considéré
 */
void cbLaser(const sensor_msgs::PointCloudPtr& msg){

    try{
        if(!imgOk)return;
        if(!calibOk)return;

        imgOut=imgIn.clone();


        sensor_msgs::PointCloud out;
        ros::Time acquisition_time = msg->header.stamp;
        ros::Duration timeout(1.0 / 30);
        listener->waitForTransform(cam_model.tfFrame(), "/base_laser_link",
                                  acquisition_time, timeout);
        listener->transformPointCloud(cam_model.tfFrame(),*msg,out);

        for (std::vector<geometry_msgs::Point32>::iterator it(out.points.begin());it!=out.points.end();it++)
        {
            cv::Point3d pt3D(it->x,it->y,it->z);
            cv::Point2d uv;
            uv=cam_model.project3dToPixel(pt3D);
              cv::circle(imgOut,uv,2,cv::Scalar(255,255,255),CV_FILLED);
        }

        if(!imgIn.empty()) {
            sensor_msgs::ImagePtr res;
            res = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgOut).toImageMsg();
            pub_res.publish(res);
        }
    }
    catch (tf::TransformException& e)
    {
        ROS_ERROR_STREAM(e.what());
    }
}

/*!
 * \brief Gère la réception des paramètres de la caméra
 * \param msg Paramètres de calibrage de la caméra
 */
void cbCalib(const sensor_msgs::CameraInfo& msg){
    cam_model.fromCameraInfo(msg);
    calibOk=true;
}

/*!
 * \brief Callback gérant la réception d'une image depuis la caméra
 * \param msg Image recue
 */
void cbImg(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        imgIn=cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        imgOk=true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("Image camera non recue : "<<e.what());
    }
}

int main(int argc, char** argv){

    ros::init(argc,argv,"pointcloud_to_camera");


    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    listener=new(tf::TransformListener);
    
    imgOk=false;
    calibOk=false;

    ros::Subscriber sub_laser=nh.subscribe("/point_cloud",1,&cbLaser);
    ros::Subscriber sub_calib=nh.subscribe("/camera_info",1,&cbCalib);

    image_transport::Subscriber sub_cam=it.subscribe("/image_in",1,&cbImg);

    pub_res=it.advertise("/image_out",1);
    ros::spin();

    delete listener;
}
