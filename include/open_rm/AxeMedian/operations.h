#ifndef OPERATIONS_H
#define OPERATIONS_H

#include <opencv2/opencv.hpp>
#include <vector>
namespace rm{
namespace AxeMedian{
/*! \brief regroupe l'ensemble des opérations appliquées à un squelette extrait
 */
namespace Operations{

int topoDegree(const cv::Mat& mask, cv::Point pos,int r);

}
}
}

#endif // OPERATIONS_H
