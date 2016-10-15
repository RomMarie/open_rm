#ifndef BLOBS_H
#define BLOBS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace rm{
/*! \brief Namespace gérant la détection, la manipulation et l'affichage de blobs
 */
namespace Blobs{

// Tous les blobs générique
std::vector<std::vector<cv::Point> > allKBlobs(const cv::Mat& mask, float k);
std::vector<std::vector<cv::Point> > nLargestKBlobs(const cv::Mat& mask, int n, float k);

// Un k-blob générique
std::vector<cv::Point> seedKBlob(const cv::Mat& mask, cv::Point s, float k);
std::vector<cv::Point> largestKBlob(const cv::Mat& mask,float k);

// Tous les blobs 4/8 connexes
std::vector<std::vector<cv::Point> > allBlob4Connexe(const cv::Mat& mask);
std::vector<std::vector<cv::Point> > allBlob8Connexe(const cv::Mat& mask);
std::vector<std::vector<cv::Point> > nLargestBlob4Connexe(const cv::Mat& mask, int n);
std::vector<std::vector<cv::Point> > nLargestBlob8Connexe(const cv::Mat& mask, int n);

// Un blob 4/8 connexe
std::vector<cv::Point> seedBlob4Connexe(const cv::Mat& mask,cv::Point s);
std::vector<cv::Point> seedBlob8Connexe(const cv::Mat& mask,cv::Point s);
std::vector<cv::Point> largestBlob4Connexe(const cv::Mat& mask);
std::vector<cv::Point> largestBlob8Connexe(const cv::Mat& mask);

// Colorie les blobs pour affichage
cv::Mat colorizeBlobs(const cv::Mat& img);

// Nombre de blobs
unsigned int nBlobs8Connexe(const cv::Mat& mask);

}
}
#endif // BLOBS_H
