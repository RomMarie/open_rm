#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <open_rm/Algebre/equations.h>
#include <open_rm/Geometrie/bezier.h>

int main(int argc,char ** argv){


    std::vector<double> res=rm::Algebre::thirdOrderEq(6,-6,12,7);
    for(unsigned int i=0;i<res.size();i++)
     std::cout<<res[i]<<std::endl;


    std::cout<<1/3.+1/3.*(pow(5/2.,1/3.)-pow(5.,2/3.)*pow(2.,1/3.))<<std::endl;

    std::vector<cv::Point2d> pc;
    pc.push_back(cv::Point2d(100,400));
    pc.push_back(cv::Point2d(250,150));
    pc.push_back(cv::Point2d(450,450));
    pc.push_back(cv::Point2d(50,350));

    rm::Geometrie::bezierCurve bc(pc);

//    bc.draw(img);


    for(int i=0;i<100;i++){
//    bc.movePtCtrl(1,cv::Point2d(0,1),10);
        cv::Mat img(500,500,cv::DataType<cv::Vec3b>::type);
    bc.movePtCurve(1/3.,cv::Point2d(0.5,0.5),23);

    bc.draw(img,cv::Scalar(255,255,0));
    cv::imshow("img",img);cv::waitKey();
    }
}
