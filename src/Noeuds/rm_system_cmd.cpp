#include <ros/ros.h>

int main(int argc,char ** argv){
    ros::init(argc,argv,"system_call");
    ros::NodeHandle nh("~");

    std::string commande;
    nh.param<std::string>("command",commande,"");

    return system(commande.c_str());
}
