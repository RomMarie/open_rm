#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <open_rm/Algebre/equations.h>
#include <open_rm/Geometrie/bezier.h>

int main(int argc,char ** argv){


    std::vector<cv::Point2d> pc;
    pc.push_back(cv::Point2d(100,400));
    pc.push_back(cv::Point2d(250,150));
    pc.push_back(cv::Point2d(450,450));
//    pc.push_back(cv::Point2d(50,350));

    rm::Geometrie::bezierCurve bc(pc);





        cv::Mat img(500,500,cv::DataType<cv::Vec3b>::type);

        cv::Point2d pt(100,400);


    bc.draw(img,cv::Scalar(0,0,255));

    cv::circle(img,bc.computePt(bc.closestPt(pt)),3,cv::Scalar(255,0,0));
    cv::imshow("img",img);cv::waitKey();
}
