#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <open_rm/Geometrie/bezier.h>

int main(int argc,char ** argv){
    std::vector<cv::Point2d> pc;
    pc.push_back(cv::Point2d(100,400));
    pc.push_back(cv::Point2d(450,150));
    pc.push_back(cv::Point2d(450,450));
    pc.push_back(cv::Point2d(350,350));

    rm::Geometrie::bezierCurve bc(pc);

    cv::Mat img(500,500,cv::DataType<cv::Vec3b>::type);
    bc.draw(img);

    rm::Geometrie::bezierCurve bc2=bc.split();
    bc.draw(img,cv::Scalar(255,0,0));
    bc2.draw(img,cv::Scalar(255,0,255));

    cv::imshow("img",img);cv::waitKey();
}
