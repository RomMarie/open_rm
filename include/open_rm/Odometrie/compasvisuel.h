#ifndef COMPASVISUEL_H
#define COMPASVISUEL_H

#include <opencv2/opencv.hpp>
#include <vector>

namespace rm{
/*! \brief Namespace gérant les outils mesurant le déplacement
 */
namespace Odometrie{
/*! \brief Namespace regroupant l'ensemble des algorithmes de compas visuel
 */
namespace CompasVisuel{

double complet(const cv::Mat& imgSph, const cv::Mat& imgRef);
double ring(const cv::Mat& imgSph, const cv::Mat& imgRef, cv::Point HOI);
double roi(const cv::Mat& imgSph, const cv::Mat& imgRef, cv::Rect ROI);

}
}
}

#endif // COMPASVISUEL_H
