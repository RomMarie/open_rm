#ifndef FDELTAMA_H
#define FDELTAMA_H

#include <opencv2/opencv.hpp>

namespace rm{
/*! \brief Namespace gérant les algorithmes de squelettisation
 * et opérations sur axes médians
 */
namespace AxeMedian{

/*! \brief Classe permettant le calcul du FDMA
 */
class FDeltaMA
{
public:
    FDeltaMA();
    ~FDeltaMA();

    static cv::Mat ma(const cv::Mat& mask, double delta);
    static cv::Mat mat(const cv::Mat& mask,double delta);
};

}
}
#endif // FDELTAMA_H
