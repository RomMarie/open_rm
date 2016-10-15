#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <open_rm/Blobs/blobs.h>

cv::Mat skel;
/*!
 * \brief Action à réaliser à la réception d'une image squelette
 * \param msg Masque définissant le squelette à traiter
 */
void callback(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        skel=cv_bridge::toCvShare(msg,"mono8")->image;

        if(rm::Blobs::nBlobs8Connexe(skel)>1){
            ROS_ERROR_STREAM("Erreur skel -> graph : Le squelette n'est pas connexe");
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("Erreur skel -> graph : "<<e.what());
    }
}

int main(int argc,char ** argv){
    ros::init(argc,argv,"skel_to_graph");

    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub=it.subscribe("/skeleton",1,&callback);

    ros::spin();
}
