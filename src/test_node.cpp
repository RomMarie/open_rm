#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <open_rm/Algebre/equations.h>
#include <open_rm/Geometrie/bezier.h>
#include <open_rm/Algebre/polynomes.h>

#include <ctime>

int main(int argc,char ** argv){


    // 11 points pour test échantillonnage squelette
    std::vector<cv::Point2d> skel;
  /*  skel.push_back(cv::Point2d(300,100));
    skel.push_back(cv::Point2d(300,150));
    skel.push_back(cv::Point2d(280,190));
    skel.push_back(cv::Point2d(260,230));
    skel.push_back(cv::Point2d(230,260));
    skel.push_back(cv::Point2d(190,270));
    skel.push_back(cv::Point2d(160,300));
    skel.push_back(cv::Point2d(140,330));
    skel.push_back(cv::Point2d(120,360));
    skel.push_back(cv::Point2d(120,300));
    skel.push_back(cv::Point2d(140,250));
    skel.push_back(cv::Point2d(160,200));
    skel.push_back(cv::Point2d(180,150));*/
    skel.push_back(cv::Point2d(200,100));
    skel.push_back(cv::Point2d(220,150));
    skel.push_back(cv::Point2d(250,190));
    skel.push_back(cv::Point2d(300,220));
    skel.push_back(cv::Point2d(320,160));
    skel.push_back(cv::Point2d(350,140));
    skel.push_back(cv::Point2d(400,250));
    skel.push_back(cv::Point2d(450,270));
    skel.push_back(cv::Point2d(480,270));
    skel.push_back(cv::Point2d(500,280));
    skel.push_back(cv::Point2d(520,290));
    skel.push_back(cv::Point2d(530,300));

    /*            skel.push_back(cv::Point2d(350,270));
            skel.push_back(cv::Point2d(300,250));
            skel.push_back(cv::Point2d(250,240));
            skel.push_back(cv::Point2d(200,220));
            skel.push_back(cv::Point2d(150,190));
            skel.push_back(cv::Point2d(120,150));
            skel.push_back(cv::Point2d(100,100));*/

    std::vector<double> poly1,poly2;
    poly1.push_back(1);
    poly1.push_back(-1);
    poly2.push_back(0);
    poly2.push_back(1);
    rm::Algebre::Polynome Poly1(poly1),Poly2(poly2);


    cv::Mat img(1000,1000,CV_8UC3);

    std::vector<rm::Geometrie::Bezier::Courbe> bcs;

    for(int i=0;i<1;i++)bcs=rm::Geometrie::Bezier::fitCubicCurves(skel,10);

    for(unsigned int i=0;i<bcs.size();i++){
        bcs[i].draw(img,cv::Scalar(0,128*i-1,255),false);

    }
    for(unsigned int i=0;i<skel.size();i++){
        cv::circle(img,skel[i],3,cv::Scalar(255,0,0));
    }

    std::cout<<bcs[0].polyX()<<std::endl;
    std::cout<<bcs[0].polyY()<<std::endl;

    std::cout<<bcs[0].polyX().derivate()<<std::endl;
    cv::imshow("img",img);cv::waitKey();

}
