#ifndef SIGNATURESVISUELLES_H
#define SIGNATURESVISUELLES_H

#include <opencv2/opencv.hpp>


namespace rm{
/*! \brief Namespace regroupant les traitements relatifs au calcul et à l'utilisation de signatures visuelles.
 */
namespace SignaturesVisuelles{

// Calcul de signatures visuelles
std::vector<cv::Vec3f> histoCouleur(const cv::Mat& img, int nBin=256, bool flou=0);
std::vector<cv::Vec3f> histoCouleurNormalise(const cv::Mat& img, int nBin=256, bool flou=0);
std::vector<cv::Vec3f> monomesRGB(const cv::Mat& img, const std::vector<cv::Vec2i> monomes,int nBin=256, bool flou=0);
std::vector<cv::Vec3f> haarFmFpFt(const cv::Mat& img,const std::vector<cv::Vec2i> monomes, const cv::Mat& mask, int nBin=256);
std::vector<cv::Vec3f> haarFmFpFt(const cv::Mat& img,const std::vector<cv::Vec2i> monomes, cv::Vec3b colPert=cv::Vec3b(255,0,0), int nBin=256);
std::vector<cv::Vec3f> haarFmFpFtFs(const cv::Mat& img,const std::vector<cv::Vec2i> monomes, std::vector<cv::Point2f> M, cv::Vec3b colPert=cv::Vec3b(255,0,0),int nBin=256);
std::vector<cv::Vec3f> haarFmFpFtFs(const cv::Mat& img,const std::vector<cv::Vec2i> monomes, std::vector<cv::Point2f> M, const cv::Mat& mask,int nBin=256);

// Distances entre signatures
float intersectionHistoCouleur(std::vector<cv::Vec3f> histo1,std::vector<cv::Vec3f> histo2,bool isNormalized=true);
float intersectionHisto(std::vector<float> histo1,std::vector<float> histo2,bool isNormalized=true);

// Opérations
std::vector<float> normalizeHisto(const std::vector<float>& histo);
std::vector<cv::Vec3f> normalizeHisto(const std::vector<cv::Vec3f>& histo);

// Affichages
cv::Mat printHistoCouleur(const std::vector<cv::Vec3f>& histo,float vMax=1);

}
}

#endif // SIGNATURESVISUELLES_H
