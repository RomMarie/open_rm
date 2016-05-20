#include <open_rm/Odometrie/compasvisuel.h>
#include <iostream>
namespace rm{
namespace Odometrie{
namespace CompasVisuel{

/*! \brief Calcul l'orientation relative entre deux vues panoramique
 *
 * Utilise un algorithme de compas visuel par minimisation de l'erreur
 * quadratique sur l'ensemble de l'image.
 * \param[in] imgSph Image courante dont l'orientation est à estimer
 * \param[in] imgRef Image de référence définissant l'orientation 0
 * \return angle relatif entre les deux images (en radian, entre -PI et PI)
 * \note Accepte les images ndg et RGB
 */
double complet(const cv::Mat &imgSph, const cv::Mat &imgRef)
{
    // On commence par dupliquer l'image de référence (pour éviter les effets de bord)
    std::vector<cv::Mat> vec;
    vec.push_back(imgRef);
    vec.push_back(imgRef);
    cv::Mat fresque;
    hconcat( vec, fresque );

    int best,bestSom=255*3*imgSph.rows*imgSph.cols;
    for(int o=0;o<imgSph.cols;o++){
        int som=0;
        for(int i=0;i<imgSph.rows;i++){
            for(int j=0;j<imgSph.cols;j++){
                switch(imgSph.type()){
                case CV_8U:
                    som+=abs((int)imgSph.at<uchar>(i,j)-(int)fresque.at<uchar>(i,j+o));
                    break;
                case CV_8UC3:
                    som+=abs((int)imgSph.at<cv::Vec3b>(i,j)[0]-(int)fresque.at<cv::Vec3b>(i,j+o)[0]);
                    som+=abs((int)imgSph.at<cv::Vec3b>(i,j)[1]-(int)fresque.at<cv::Vec3b>(i,j+o)[1]);
                    som+=abs((int)imgSph.at<cv::Vec3b>(i,j)[2]-(int)fresque.at<cv::Vec3b>(i,j+o)[2]);
                    break;
                }
            }
        }
        if(som<bestSom){
            bestSom=som;
            best=o;
        }
    }

    if(best<imgSph.cols/2){
        return best*2*CV_PI/imgSph.cols;
    }
    else{
        return (best-imgSph.cols)*2*CV_PI/imgSph.cols;
    }
}

/*! \brief Calcul l'orientation relative entre deux vues panoramique
 *
 * Utilise un algorithme de compas visuel par minimisation de l'erreur
 * quadratique sur une région d'intérêt de l'image.
 * \param[in] imgSph Image courante dont l'orientation est à estimer
 * \param[in] imgRef Image de référence définissant l'orientation 0
 * \param[in] ROI Région d'intérêt de l'image (définie par un cv::Rect)
 * \return angle relatif entre les deux images (en radian, entre -PI et PI)
 * \note Accepte les images ndg et RGB
 */
double roi(const cv::Mat &imgSph, const cv::Mat &imgRef, cv::Rect ROI)
{

    // On commence par dupliquer l'image de référence (pour éviter les effets de bord)
    std::vector<cv::Mat> vec;
    vec.push_back(imgRef);
    vec.push_back(imgRef);
    cv::Mat fresque;
    hconcat( vec, fresque );

    int best,bestSom=255*3*imgSph.rows*imgSph.cols;
    for(int o=0;o<imgSph.cols;o++){
        int som=0;
        for(int i=ROI.y;i<ROI.y+ROI.width;i++){
            for(int j=ROI.x;j<ROI.x+ROI.height;j++){
                switch(imgSph.type()){
                case CV_8U:
                    som+=abs((int)imgSph.at<uchar>(i,j)-(int)fresque.at<uchar>(i,j+o));
                    break;
                case CV_8UC3:
                    som+=abs((int)imgSph.at<cv::Vec3b>(i,j)[0]-(int)fresque.at<cv::Vec3b>(i,j+o)[0]);
                    som+=abs((int)imgSph.at<cv::Vec3b>(i,j)[1]-(int)fresque.at<cv::Vec3b>(i,j+o)[1]);
                    som+=abs((int)imgSph.at<cv::Vec3b>(i,j)[2]-(int)fresque.at<cv::Vec3b>(i,j+o)[2]);
                    break;
                }
            }
        }
        if(som<bestSom){
            bestSom=som;
            best=o;
        }
    }

    if(best<imgSph.cols/2){
        return best*2*CV_PI/imgSph.cols;
    }
    else{
        return (best-imgSph.cols)*2*CV_PI/imgSph.cols;
    }
}

/*! \brief Calcul l'orientation relative entre deux vues panoramique
 *
 * Utilise un algorithme de compas visuel par minimisation de l'erreur
 * quadratique sur une région d'intérêt de l'image.
 * \param[in] imgSph Image courante dont l'orientation est à estimer
 * \param[in] imgRef Image de référence définissant l'orientation 0
 * \param[in] HOI Anneau d'intéret dans la vue panoramique (définie par ses coordonnées verticales
 * hautes et basses : cv::Point)
 * \return angle relatif entre les deux images (en radian, entre -PI et PI)
 * \note Accepte les images ndg et RGB
 */
double ring(const cv::Mat &imgSph, const cv::Mat &imgRef, cv::Point HOI)
{
    return rm::Odometrie::CompasVisuel::roi(imgSph,imgRef,cv::Rect(HOI.x,0,HOI.y-HOI.x,imgSph.cols));
}


}
}
}
