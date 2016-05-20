#ifndef TRANSFDISTANCE
#define TRANSFDISTANCE

#include <opencv2/opencv.hpp>

namespace rm{
/*! \brief Namespace gérant les opérations de transformée en distance, et calcul de projection
 */
namespace TransfDistance{

void DPHesselink(const cv::Mat& mask, cv::Mat &D, cv::Mat &P);
void DPHesselink3d(const cv::Mat &mask, cv::Mat &D, cv::Mat &P, int d1, int d2, int d3);

}

}

#endif // TRANSFDISTANCE

