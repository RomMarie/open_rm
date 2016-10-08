#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <QImageReader>
#include <boost/filesystem.hpp>
class ImageDirectory{
public:
    ImageDirectory(std::string dir, bool withCycle=false);
    std::string next(){
        img_++;
        if(img_==imgs_.end()){
            if(cycle_)
                img_=imgs_.begin();
            else{
                ROS_INFO_STREAM("Fin de la sequence d'images");
                ros::shutdown();
            }
        }

        return dir_+"/"+*img_;
    }

    std::string prev(){
        if(img_!=imgs_.begin())
            img_--;

        return dir_+"/"+*img_;
    }

    std::string current(){
        return dir_+"/"+*img_;
    }

    std::string dir(){
        return dir_;
    }

    std::string img(){
        return *img_;
    }

private:
    std::string dir_;
    std::string file_;

    std::vector<std::string> imgs_;
    std::vector<std::string>::iterator img_;

    bool cycle_;
};
ImageDirectory::ImageDirectory(std::string dir, bool withCycle){

    cycle_=withCycle;

    boost::filesystem::path p(dir);
    try
    {
        // On vérifie que le répertoire existe
        if (!boost::filesystem::exists(p))
        {
            ROS_ERROR_STREAM("Repertoire d'images incorrect");
            ros::shutdown();
        }
        dir_=dir;

        // On copie les fichiers du répertoire dans un vector
        std::vector<boost::filesystem::path> fics;
        std::copy(boost::filesystem::directory_iterator(p),
                  boost::filesystem::directory_iterator(),
                  std::back_inserter(fics));
        // On retient uniquement les éléments qui sont des images
        for (std::vector<boost::filesystem::path>::const_iterator it (fics.begin()); it != fics.end();it++)
        {
            std::string filepath= dir+"/"+it->filename().string();
            QImageReader reader(filepath.c_str());

            if(reader.imageFormat()!=0)
            {
                imgs_.push_back(it->filename().c_str());
            }
        }

        // On garantit que les images soient triées par ordre alphabétique
        std::sort(imgs_.begin(), imgs_.end());

        if(imgs_.empty())
        {
            ROS_ERROR_STREAM("Aucune image dans le repertoire");
            ros::shutdown();
        }
        // On positionne l'itérateur sur la première image
        img_=imgs_.begin();
    }
    catch (const boost::filesystem::filesystem_error& ex)
    {
        ROS_ERROR_STREAM(ex.what());
        ros::shutdown();
    }

}

int main(int argc,char ** argv){
    ros::init(argc,argv,"rm_dir_to_image");

    ros::NodeHandle nh("~");

    // Lecture des paramètres
    bool withCycle;
    nh.param("cyclic",withCycle,false);

    double framerate;
    nh.param("frame_rate",framerate,1.);

    bool isColor;
    nh.param("is_color",isColor,false);

    std::string directory;
    nh.param<std::string>("input_dir",directory,"");

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub=it.advertise("/input_image",0);

    ros::Rate loop_rate(framerate);

    cv::Mat img;
    sensor_msgs::ImagePtr msg;

    ImageDirectory repertoire(directory,withCycle);

    img=cv::imread(repertoire.current(),isColor);
    while(ros::ok()){
        if(!img.empty()){
            if(isColor)
                msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
            else
                msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
            pub.publish(msg);
        }

        img=cv::imread(repertoire.next(),isColor);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
