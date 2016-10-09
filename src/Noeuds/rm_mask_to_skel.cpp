#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

#include <open_rm/AxeMedian/fdeltama.h>
#include <open_rm/Blobs/blobs.h>
#include <open_rm/mask_to_skelConfig.h>

cv::Mat mask,skel,mat;
double delta;
bool onlyConnected;

image_transport::Publisher pub;
image_transport::Publisher pub2;

/*!
 * \brief cb_dyn CallBack de la configuration dynamique
 * \param config Structure régissant les paramètres dynamiques
 * \param level ???
 */
void cb_dyn(open_rm::mask_to_skelConfig &config, uint32_t level)
{
    delta=config.delta;
    onlyConnected=config.onlyConnected;
}

/*!
 * \brief cb_img CallBack gérant la réception d'une image sur input_mask
 * \param msg Image lue sur le topic d'entrée
 */
void cb_img(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        mask=cv_bridge::toCvShare(msg, "mono8")->image;
        mat=rm::AxeMedian::FDeltaMA::mat(mask,delta);

        skel=cv::Mat::zeros(mat.size(),CV_8U);
        for(unsigned int i=0;i<skel.rows;i++){
            for(unsigned int j=0;j<skel.cols;j++){
                if(mat.at<double>(i,j)>0)
                    skel.at<uchar>(i,j)=255;
            }
        }

        if(onlyConnected){
            std::vector<cv::Point> blob=rm::Blobs::largestBlob8Connexe(skel);
            skel=cv::Mat::zeros(skel.size(),CV_8U);
            for(std::vector<cv::Point>::iterator it(blob.begin());it!=blob.end();it++){
                cv::Point pt=*it;
                skel.at<uchar>(pt.x,pt.y)=255;
            }
            for(unsigned int i=0;i<skel.rows;i++){
                for(unsigned int j=0;j<skel.cols;j++){
                    if(mat.at<double>(i,j)>0&&skel.at<uchar>(i,j)==0)
                        mat.at<double>(i,j)=0;
                }
            }
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", skel).toImageMsg();
        pub.publish(msg);

        sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_64FC1, mat).toImageMsg();
        pub2.publish(msg2);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR_STREAM("Erreur mask -> skel : "<<e.what());
    }
}

int main(int argc, char** argv){
    ros::init(argc,argv,"rm_mask_to_skel");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    nh.param("delta",delta,1.);
    nh.param("onlyConnected",onlyConnected,false);


    // Initialisation du serveur de reconfiguration dynamique
    dynamic_reconfigure::Server<open_rm::mask_to_skelConfig> srv;
    dynamic_reconfigure::Server<open_rm::mask_to_skelConfig>::CallbackType f;
    f = boost::bind(&cb_dyn, _1, _2);
    srv.setCallback(f);

    // Souscription au topic d'entrée
    image_transport::Subscriber sub=it.subscribe("/input_mask",1,&cb_img);
    // Définition des topics de sortie
    pub=it.advertise("/raw_skeleton",1);
    pub2=it.advertise("/raw_mat",1);

    ros::spin();
}
