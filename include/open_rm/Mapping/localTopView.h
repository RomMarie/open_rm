#ifndef MAPPING_H
#define MAPPING_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace rm{
namespace Mapping{

/*!
 * \brief Classe permettant de construire et maintenir une vue du dessus centrée sur le robot.
 *
 * L'objectif de cette classe est de projeter dans un même référentiel les données en provenance de plusieurs capteurs
 * (lasers, bird eye view, sonars, ...)
 */
class LocalTopView
{
public:
    LocalTopView(const double pixPerMeter, const cv::Size size);
    void reset();
    void addPts(const std::vector<cv::Point2f> pts,const cv::Vec3b couleur);
    void move(const float R, const cv::Point2f t);
    cv::Mat drawMask();
    void showPts(const std::string nom);
    void showMask(const std::string nom);
    ~LocalTopView();
private:
    cv::Mat _occGrid;
    double _ppm;
    cv::Size _size;
};

}
}

#endif // MAPPING_H
