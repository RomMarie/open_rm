#ifndef MAPPING_H
#define MAPPING_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <open_rm/Graphe/graphe.h>

namespace rm{
/*! \brief Namespace regroupant les classes relatives à la cartographie
 * en robotique mobile.
 */
namespace Mapping{

/*! \brief Classe gérant une grille d'occupation locale
 * centrée sur le robot
 */
class localOccGrid
{
public:

    // Constructeurs / destructeurs
    localOccGrid();
    localOccGrid(float pixPerMeter, cv::Size size, float inc=0.1, float dec=0.1);
    ~localOccGrid();

    // Opérations sur la grille
    void addWPts(const std::vector<cv::Point2f> pts);
    void move(const float R,const cv::Point2f t);
    cv::Mat build3States(const float tMin,const float tMax);

    // Affichage
    void show(const std::string nom);

private:
    // Méthodes internes
    void increment(int i,int j);
    void decrement(int i,int j,float coef);


    cv::Size _size; // dimensions de la grille
    float _ppm; // pixels par mètre
    cv::Mat _grid; // structure de la carte
    float _inc; // incrément lorsqu'une cellule est détectée libre
    float _dec; // décrément non pondéré lorsqu'une cellule est détectée occupée
};

}
}

#endif // MAPPING_H
