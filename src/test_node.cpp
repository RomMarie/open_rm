#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <open_rm/Algebre/equations.h>
#include <open_rm/Geometrie/bezier.h>
#include <open_rm/Algebre/polynomes.h>

#include <ctime>

int main(int argc,char ** argv){


    std::vector<cv::Point2d> pc;
    pc.push_back(cv::Point2d(200,400));
    pc.push_back(cv::Point2d(250,250));
    pc.push_back(cv::Point2d(350,350));
    pc.push_back(cv::Point2d(250,450));

    rm::Geometrie::bezierCurve bc(pc);

    double t;

    cv::Point2d p(120,200);


    double poly[5];
    poly[0]=1;
    poly[1]=-1;
    poly[2]=2;
    poly[3]=-11;
    poly[4]=1;

    rm::Algebre::Polynome po((double*)poly,4);
    double poly2[4];
    poly2[0]=1;
    poly2[1]=-1;
    poly2[2]=2;
    poly2[3]=-1;

    rm::Algebre::Polynome po2((double*)poly2,3);

    clock_t  a=clock();
    for(int i=0;i<100000;i++)
        po.sturmSequence(0,1,1);

        //bc.distToCurve(p,t);
      //  std::cout<<bc.distToCurve(p,t)<<std::endl;
       // std::cout<<t<<std::endl;

    std::cout<<"division Polynomiale : "<<(clock()-a)/100000000.<<std::endl;
        cv::Mat img(600,500,cv::DataType<cv::Vec3b>::type);

        cv::Point2d pt(100,400);


    bc.draw(img,cv::Scalar(0,0,255));
    cv::circle(img,p,3,cv::Scalar(0,255,0));
    cv::circle(img,bc.computePtPoly(t),3,cv::Scalar(0,255,0));
    cv::imshow("img",img);cv::waitKey();
}
