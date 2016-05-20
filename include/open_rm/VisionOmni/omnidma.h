#ifndef OMNIDMA_H
#define OMNIDMA_

#include <open_rm/VisionOmni/paramscalib.h>
#include <opencv2/opencv.hpp>

namespace rm{
namespace VisionOmni{

/*! \brief Classe permettant de calculer le Omni Delta Medial Axis
 * d'un espace libre omnidirectionnel.
 *
 * La classe précalcule la métrique adaptée pour permettre à l'algorithme
 * de tourner en temps réel. L'entrée est un masque binaire défini directement
 * dans l'image omnidirectionnelle et identifiant l'espace navigable visible.
 */
class OmniDMA
{
public:
    OmniDMA();
    OmniDMA(const rm::VisionOmni::ParamsCalib& pc);
    ~OmniDMA();

    void init(const rm::VisionOmni::ParamsCalib& pc);
    cv::Mat computeWithDT(const cv::Mat& mask, int delta);
    cv::Mat compute(const cv::Mat& mask, int delta);
    cv::Mat computeOmniDT(const cv::Mat& mask);
private:
    double dSol(cv::Point p1,cv::Point p2);
    cv::Mat _lut;
    cv::Point _c;
};

}
}

#endif // OMNIDMA_H
