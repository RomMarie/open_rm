#ifndef SPHEREEQ_H
#define SPHEREEQ_H

#include <open_rm/VisionOmni/paramscalib.h>
#include <opencv2/opencv.hpp>

namespace rm{
namespace VisionOmni{

/*! \brief Classe définissant une sphère d'équivalence pour une caméra omni avec une résolution désirée.
 *
 * L'intérêt est de précalculer la projection d'une image acquise sur la sphère, donc de gagner
 * énormément de temps de calcul
 *
 * \todo Image d'illustration
 */
class SphereEq{
public:
    SphereEq();
    SphereEq(const rm::VisionOmni::ParamsCalib& pc,int nTheta,int nPhi);
    ~SphereEq();
    cv::Mat project(const cv::Mat& img,int thetaMin=-1,int thetaMax=-1);
    void init(const rm::VisionOmni::ParamsCalib& pc,int nTheta,int nPhi);
private:
    cv::Mat _lut;
};

}
}


#endif // SPHEREEQ_H
