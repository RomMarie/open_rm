#ifndef BEZIER_H
#define BEZIER_H

#include <opencv2/opencv.hpp>

#include <open_rm/Algebre/equations.h>

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
    void draw(cv::Mat& img, cv::Scalar color=cv::Scalar(255,255,255), bool withPC=true);
    void movePtCtrl(int ind, cv::Point2d dir, double scale);
    void movePtCurve(double t, cv::Point2d dir, double scale);
    double closestPt(cv::Point2d pt);

    std::vector<cv::Point2d> pc; ///< Vecteur contenant les points de controle de la courbe

private:
    cv::Point2d deCasteljau(std::vector<cv::Point2d> Pc, float t);
};

std::vector<bezierCurve> fitCurves(std::vector<cv::Point2d> pts, double seuil);

}
}

#endif // BEZIER_H
