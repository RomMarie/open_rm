#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <open_rm/Algebre/equations.h>
#include <open_rm/Geometrie/bezier.h>
#include <open_rm/Algebre/polynomes.h>

int main(int argc,char ** argv){


    std::vector<cv::Point2d> pc;
    pc.push_back(cv::Point2d(100,400));
    pc.push_back(cv::Point2d(250,150));
    pc.push_back(cv::Point2d(350,550));
    pc.push_back(cv::Point2d(250,350));

    rm::Geometrie::bezierCurve bc(pc);

    double t;

//    for(int i=0;i<100000;i++)
        bc.distToCurve(cv::Point2d(100,450),t);


        cv::Mat img(600,500,cv::DataType<cv::Vec3b>::type);

        cv::Point2d pt(100,400);


    bc.draw(img,cv::Scalar(0,0,255));

    cv::imshow("img",img);cv::waitKey();
}
