#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <open_rm/Algebre/equations.h>
#include <open_rm/Geometrie/bezier.h>
#include <open_rm/Algebre/polynomes.h>

int main(int argc,char ** argv){


    std::vector<cv::Point2d> pc;
    pc.push_back(cv::Point2d(100,400));
    pc.push_back(cv::Point2d(250,150));
    pc.push_back(cv::Point2d(450,550));
//    pc.push_back(cv::Point2d(50,350));

    rm::Geometrie::bezierCurve bc(pc);


    std::vector<double> N;
    N.push_back(0);
    N.push_back(-2);
    N.push_back(3);
    N.push_back(-1);
    N.push_back(-1);
    N.push_back(1);

    std::vector<double> D;
    D.push_back(1);
    D.push_back(-1);
    D.push_back(1);


    rm::Algebre::Polynome n(N),d(D),q,r;
    n.divisionPolynomiale(D,q,r);

        cv::Mat img(600,500,cv::DataType<cv::Vec3b>::type);

        cv::Point2d pt(100,400);


    bc.draw(img,cv::Scalar(0,0,255));

    cv::circle(img,bc.computePt(bc.closestPt(pt)),3,cv::Scalar(255,0,0));
    cv::imshow("img",img);cv::waitKey();
}
