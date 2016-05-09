#ifndef DISTANCES_H
#define DISTANCES_H

#include <opencv2/opencv.hpp>

namespace rm{
namespace Geometrie{

/*!
 * Définition de divers mesures de distances
 */
namespace Distances{


extern cv::Point2d defaultProjection; ///< Projection par défaut en cas de besoin
double pointToSegment2D(cv::Point2d p, cv::Point2d p1, cv::Point2d p2, cv::Point2d& proj=defaultProjection);

}
}
}

#endif // DISTANCES_H
