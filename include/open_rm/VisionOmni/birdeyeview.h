#ifndef BIRDEYEVIEW_H
#define BIRDEYEVIEW_H

#include <open_rm/VisionOmni/paramscalib.h>
#include <opencv2/opencv.hpp>

namespace rm{

namespace VisionOmni{

/*! \brief Classe gérant la projection d'une image omni sur une bird eye view
 *
 * La bird eye view est une projection de l'image omni sur un plan parallèle au plan image.
 * L'idée est de supposer que tous les points de l'image appartiennent à un même
 * plan d'équation Z=h dans la scène, ce qui permet de lever les ambiguités sur le
 * modèle de projection inverse. Si (X,Y,Z) correspond aux coordonnées 3D d'un point,
 * et (x,y) sa projection dans le plan image normalisé, on a alors :
 *
 * \f$\left\{\begin{array}{l}
 * \textbf{x}=\textbf{X}/(\textbf{h}+\rho \xi)\\
 * \textbf{y}=\textbf{Y}/(\textbf{h}+\rho \xi)\\
 * \rho=\sqrt{X^2+Y^2+h^2}\end{array}\right.
 * \f$
 *
 * \todo Image d'illustration
 */
class BirdEyeView
{
public:
    BirdEyeView();
    BirdEyeView(const rm::VisionOmni::ParamsCalib &pc,double pixPerMeter=50,cv::Size dim=cv::Size(500,500));

    cv::Mat project(const cv::Mat& img);
    cv::Mat projectSansLUT(const rm::VisionOmni::ParamsCalib &pc,const cv::Mat& img, double pixPerMeter=50, cv::Size dim=cv::Size(500,500));

    void init(const rm::VisionOmni::ParamsCalib &pc,double pixPerMeter=50,cv::Size dim=cv::Size(500,500));
    ~BirdEyeView();
private:
    cv::Mat _lut;
};

}}
#endif // BIRDEYEVIEW_H
