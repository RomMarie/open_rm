#ifndef BEZIER_H
#define BEZIER_H

#include <opencv2/opencv.hpp>

namespace rm{
namespace Geometrie{

/*!
 * \brief Classe représentant une courbe de Bézier d'ordre quelconque
 */
struct bezierCurve{

    bezierCurve(std::vector<cv::Point2d> ptsControle);
    unsigned int degre();
    bezierCurve split();
    cv::Point2d computePt(double t);
    void draw(cv::Mat& img, cv::Scalar color=cv::Scalar(255,255,255));

    std::vector<cv::Point2d> pc; ///< Vecteur contenant les points de controle de la courbe

private:
    cv::Point2d deCasteljau(std::vector<cv::Point2d> Pc, float t);
};



}
}

#endif // BEZIER_H
