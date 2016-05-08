#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <open_rm/Algebre/equations.h>
#include <open_rm/Geometrie/bezier.h>
#include <open_rm/Algebre/polynomes.h>

#include <ctime>

int main(int argc,char ** argv){


    // 11 points pour test échantillonnage squelette
    std::vector<cv::Point2d> skel;
    skel.push_back(cv::Point2d(100,100));
    skel.push_back(cv::Point2d(110,120));
    skel.push_back(cv::Point2d(130,140));
    skel.push_back(cv::Point2d(360,160));
    skel.push_back(cv::Point2d(160,180));
    skel.push_back(cv::Point2d(150,200));
    skel.push_back(cv::Point2d(130,220));
    skel.push_back(cv::Point2d(330,240));
    skel.push_back(cv::Point2d(140,260));
    skel.push_back(cv::Point2d(160,280));
//    skel.push_back(cv::Point2d(190,280));


    cv::Mat img(1000,1000,CV_8UC3);
    rm::Geometrie::Bezier::Courbe bc(skel);
    bc.draw(img,cv::Scalar(255,0,255));

    cv::imshow("img",img);cv::waitKey();



}
