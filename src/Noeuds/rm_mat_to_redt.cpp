#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>


cv::Mat mat,redt;
double delta;

image_transport::Publisher pub;
image_transport::Publisher pub2;

/*!
 * \brief cb_img CallBack gérant la réception d'une image sur input_mask
 * \param msg Image lue sur le topic d'entrée
 */
void cb_img(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        mat=cv_bridge::toCvShare(msg,"")->image;
        redt=cv::Mat::zeros(mat.size(),CV_8U);
        for(unsigned int i=0;i<mat.rows;i++){
            for(unsigned int j=0;j<mat.cols;j++){
                if(mat.at<double>(i,j)>0)
                    cv::circle(redt,cv::Point(j,i),mat.at<double>(i,j),255,CV_FILLED);
            }
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", redt).toImageMsg();
        pub.publish(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("Erreur mask -> skel : "<<e.what());
    }
}

int main(int argc, char** argv){
    ros::init(argc,argv,"rm_mat_to_redt");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    // Souscription au topic d'entrée
    image_transport::Subscriber sub=it.subscribe("/medial_axis_transform",1,&cb_img);
    // Définition des topics de sortie
    pub=it.advertise("/redt",1);

    ros::spin();
}

