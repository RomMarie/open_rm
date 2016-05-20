#ifndef DELTAMA_H
#define DELTAMA_H

#include <open_rm/TransfDistance/transfdistance.h>

namespace rm{
namespace AxeMedian{

/*! \brief Classe permettant le calcul du DMA
 */
class DeltaMA
{
public:
    DeltaMA();
    ~DeltaMA();

    static cv::Mat ma(const cv::Mat& mask, double delta);
    static cv::Mat mat(const cv::Mat& mask,double delta);
};

}
}
#endif // DELTAMA_H
