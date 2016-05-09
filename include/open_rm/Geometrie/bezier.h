#ifndef BEZIER_H
#define BEZIER_H

#include <opencv2/opencv.hpp>
#include <open_rm/Algebre/polynomes.h>
#include <open_rm/Intervalles/intervalles.h>


namespace rm{
namespace Geometrie{

/*!
 * Namespace gérant la manipulation de courbes de Bézier
 */
namespace Bezier{

/*!
 * \brief Classe représentant une courbe de Bézier d'ordre quelconque
 */
class Courbe{
public:
    Courbe();
    Courbe(std::vector<cv::Point2d> ptsControle,bool withPoly=true);
    void set(std::vector<cv::Point2d> ptsControle, bool withPoly=true);
    unsigned int degre();
    Courbe split();
    cv::Point2d computePt(double t);
    cv::Point2d computePtPoly(double t);

    std::vector<cv::Point2d> pc() const;

    void draw(cv::Mat& img, cv::Scalar color=cv::Scalar(255,255,255), bool withPC=true);
    void movePtCtrl(int ind, cv::Point2d dir, double scale);
    double distToCurve(cv::Point2d pt, double &t);

    rm::Algebre::Polynome polyX();
    rm::Algebre::Polynome polyY();


private:
    std::vector<cv::Point2d> _pc; ///< Vecteur contenant les points de controle de la courbe

    bool _withPoly; ///< Flag indiquant si le polynome et sa dérivée doivent etre calculés
    bool _polyOK; ///< Flag indiquant si le polynome et sa dérivée ont été calculés
    rm::Algebre::Polynome _polyX; ///< Polynome décrivant la coordonnée x de tout point de la courbe
    rm::Algebre::Polynome _polyY; ///< Polynome décrivant la coordonnée y de tout point de la courbe
    rm::Algebre::Polynome _dpolyX; ///< Polynome décrivant la variation de la coordonnée x en tout point de la courbe
    rm::Algebre::Polynome _dpolyY; ///< Polynome décrivant la variation de la coordonnée y en tout point de la courbe


    // Exprime la courbe sous la forme d'un polynome
    void buildPoly();
    rm::Algebre::Polynome deCasteljauPoly(std::vector<double> Pc);

    // Fonction interne computePt
    cv::Point2d deCasteljau(std::vector<cv::Point2d> Pc, float t);

};


std::vector<Courbe> fitCubicCurves(std::vector<cv::Point2d> pts,double thres);
}
}
}

#endif // BEZIER_H
