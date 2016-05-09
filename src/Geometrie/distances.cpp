#include <open_rm/Geometrie/distances.h>

namespace rm{
namespace Geometrie{
namespace Distances{
cv::Point2d defaultProjection;
/*!
 * \brief Détermine la distance entre un point et un segment
 * \param p Point quelconque du plan
 * \param p1 Première extrémité du segment
 * \param p2 Seconde extrémité du segment
 * \param proj (optionnel) Projection de p sur le segment
 * \return distance etre p et [p1,p2]
 */
double pointToSegment2D(cv::Point2d p, cv::Point2d p1, cv::Point2d p2, cv::Point2d &proj)
{
    double L2=(p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y);

    double r=((p.x-p1.x)*(p2.x-p1.x)+(p.y-p1.y)*(p2.y-p1.y))/L2;

    proj=p1+r*(p2-p1);

    return cv::norm(proj-p);
}

}
}
}
