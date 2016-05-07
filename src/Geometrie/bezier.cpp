#include <open_rm/Geometrie/bezier.h>
#include <open_rm/Algebre/equations.h>

namespace rm{
namespace Geometrie{

/*!
 * \brief Constructeur par défaut
 */
bezierCurve::bezierCurve()
{
    _polyOK=false;
    _withPoly=false;
}

/*!
 * \brief Constructeur principal
 * \param ptsControle points de controle de la courbe de Bézier
 * \param withPoly Indique s'il faut précalculer le polynome définissant la courbe et sa dérivée (oui par défaut)
 */
bezierCurve::bezierCurve(std::vector<cv::Point2d> ptsControle, bool withPoly)
{
    _pc=ptsControle;
    _polyOK=false;
    _withPoly=withPoly;
    if(withPoly)
        buildPoly();
}

/*!
 * \brief Définit l'ensemble des points de controle de la courbe
 * \param ptsControle Vecteur contenant les nouveaux points de controle
 * \param withPoly Indique s'il faut précalculer le polynome définissant la courbe et sa dérivée (oui par défaut)
 */
void bezierCurve::set(std::vector<cv::Point2d> ptsControle, bool withPoly)
{
    _pc=ptsControle;
    _polyOK=false;
    _withPoly=withPoly;
    if(withPoly)
        buildPoly();
}

/*!
 * \brief Indique le degré de la courbe portée par la structure
 * \return Degré de la courbe de Bézier
 */
unsigned int bezierCurve::degre()
{
    return _pc.size()-1;
}

/*!
 * \brief Sépare la courbe en deux courbes vérifiant une connectivité C^1
 *
 * La première des deux courbes écrase la courbe courante stockée par la structure, tandis
 * que la seconde courbe (nouvellement créée) est retournée par la fonction
 *
 * \return la courbe de Bézier correspondant à la seconde portion de la courbe initiale
 */
bezierCurve bezierCurve::split()
{
    // La génération de nouveaux points de controle passe par un échantillonnage de la courbe de Bezier
    // (7 points sont nécessaires)
    // On choisit les points p_0,...,p_6 tels que p_i=i/6
    std::vector<cv::Point2d> p;
    for(unsigned int i=0;i<=6;i++){
        p.push_back(computePt((double)i/6.));
    }

    // A partir des coordonées de n points d'une courbe de Bézier de degré n-1 dont on connait la position (valeur de t)
    // il est possible de retrouver les n points de contrôle de la courbe. Pour couper la courbe initiale en deux,
    // on va considérer les points p_0,...,p_3 tels que t=0,1/3,2/3 et 1 pour la première courbe,
    // et p3,...,p6 tels que t=0,1/3,2/3 et 1 pour la seconde courbe.
    // On peut ainsi en déduire les points de contrôle pour chacune des deux courbes
    std::vector<cv::Point2d> pc1,pc2;
    cv::Point2d pt;

    pc1.push_back(p[0]);
    pc1.push_back(1/18.*(-15*p[0]+54*p[1]-27*p[2]+6*p[3]));
    pc1.push_back(1/18.*(6*p[0]-27*p[1]+54*p[2]-15*p[3]));
    pc1.push_back(p[3]);
    pc2.push_back(p[3]);
    pc2.push_back(1/18.*(-15*p[3]+54*p[4]-27*p[5]+6*p[6]));
    pc2.push_back(1/18.*(6*p[3]-27*p[4]+54*p[5]-15*p[6]));
    pc2.push_back(p[6]);

    _pc=pc1;

    return bezierCurve(pc2);

}

/*!
 * \brief Calcule les coordonnées d'un point appartenant à la courbe
 * \param t position du point considéré (entre 0 et 1)
 * \return coordonnées du point appartenant à la courbe
 */
cv::Point2d bezierCurve::computePt(double t)
{
    return deCasteljau(_pc,t);
}

/*!
 * \brief Détermine un point de la courbe en utilisant son expression polynomiale (beaucoup plus rapide)
 * \param t Position du point considéré (entre 0 et 1)
 * \return Coordonnées du point appartenant à la courbe
 */
cv::Point2d bezierCurve::computePtPoly(double t)
{
    return cv::Point2d(_polyX.compute(t),_polyY.compute(t));
}

/*!
 * \brief Retourne les points de controle de la courbe
 * \return Vecteur contenant les points de controle de la courbe
 */
std::vector<cv::Point2d> bezierCurve::pc() const
{
    return _pc;
}

/*!
 * \brief Fonction de debuggage qui affiche la courbe de Bézier
 * \param img Image sur laquelle écrire
 * \param color (optionnel) Couleur du dessin
 * \param withPC (optionnel) Indique s'il faut afficher les points de controle (oui par défaut)
 */
void bezierCurve::draw(cv::Mat &img, cv::Scalar color, bool withPC)
{
    // On commence par vérifier que l'image peut afficher le patch
    std::vector<cv::Point> pts; // La fonction boundingRect de OpenCV n'accepte que des cv::Point
    for(unsigned int i=0;i<_pc.size();i++)
        pts.push_back(_pc[i]);
    cv::Rect bb=cv::boundingRect(pts);
    if(bb.x<0||bb.y<0||bb.height+bb.y>=img.rows||bb.width+bb.x>=img.cols)
        return;

    // Si les points de controle doivent etre affichés
    if(withPC){
        for(unsigned int i=0;i<_pc.size()-1;i++){
            cv::circle(img,_pc[i],3,color,CV_FILLED);
            cv::line(img,_pc[i],_pc[i+1],color);
        }
        cv::circle(img,_pc[_pc.size()-1],3,color,CV_FILLED);
    }

    // On échantillonne la courbe en 1000 points (c'est une fonction de deboggage, ca devrait suffire
    for(double t=0;t<=1;t+=0.001)
        img.at<cv::Vec3b>(computePt(t))=cv::Vec3b(color[0],color[1],color[2]);
}

/*!
 * \brief Déplace le point de contrôle \æ ind dans la direction \a dir avec une amplitude \a scale
 * \param ind Indice du point de contrôle à déplacer
 * \param dir Vecteur directeur du mouvement
 * \param scale Amplitude du mouvement
 */
void bezierCurve::movePtCtrl(int ind, cv::Point2d dir, double scale)
{
    if(_pc.size()>ind){ // Si le point de controle existe
        _pc[ind]+=dir*scale; // On le déplace
        if(_withPoly)
            buildPoly();
    }
}

/*!
 * \brief Mesure la distance entre un point \a pt donné et la courbe de bézier.
 *
 * \note Cette fonction est basée sur https://hal.archives-ouvertes.fr/file/index/docid/518379/filename/Xiao-DiaoChen2007c.pdf
 * \param pt Point dont on cherche la distance à la courbe
 * \param t Indice du point de la courbe qui minimise cette distance
 * \return distance euclidienne entre le point et la courbe
 */
double bezierCurve::distToCurve(cv::Point2d pt,double& t)
{
    // Calcul des polynomes définissant la courbe si pas encore fait
    if(!_polyOK)
        buildPoly();

    // Calcul du polynome g(t)=Poly'(t).(pt-poly(t))
    std::vector<double> px,py;
    px.push_back(pt.x);
    py.push_back(pt.y);
    rm::Algebre::Polynome g=_dpolyX*(rm::Algebre::Polynome(px)-_polyX)+_dpolyY*(rm::Algebre::Polynome(py)-_polyY);

    // Sequence de Sturm pour identifier la position des racines
    std::vector<int> res=g.sturmSequence(0,1,0.1);

    // On vérifie que chaque intervalle contient au plus une racine
    // Sinon, on applique de nouveau la séquence de Sturm pour subdiviser l'intervalle en question
    // jusqu'à obtenir une racine par intervalle
    for(unsigned int i=0;i<res.size();i++){
        if(res[i]>1){
            double borneMin=i*0.1;
            double borneMax=(i+1)*0.1;
            double pas=0.1;
            bool ok=false;
            while(!ok){
                std::vector<int> resTmp=g.sturmSequence(borneMin,borneMax,pas);
            }
        }
    }


}

/*!
 * \brief Exprime la courbe de Bézier sous forme d'un polynome
 */
void bezierCurve::buildPoly()
{
    std::vector<double> pcX,pcY;
    for(unsigned int i=0;i<_pc.size();i++){
        pcX.push_back(_pc[i].x);
        pcY.push_back(_pc[i].y);
    }

    _polyX=deCasteljauPoly(pcX);
    _polyY=deCasteljauPoly(pcY);
    _dpolyX=_polyX.derivate();
    _dpolyY=_polyY.derivate();

    _withPoly=true;
    _polyOK=true;
}

/*!
 * \brief Algorithme récursif de DeCasteljau pour l'expression de la courbe sous forme de polygone
 *
 * Il s'agit d'une méthode privée de la structure, appelée par la méthode privée buildPoly
 *
 * \param pc points de controle considérés à une itération donnée
 * \return Polynome à une itération donnée
 */
Algebre::Polynome bezierCurve::deCasteljauPoly(std::vector<double> Pc)
{
    std::vector<double> res;
    if(Pc.size()==1){
        res.push_back(Pc[0]);
        return(rm::Algebre::Polynome(res));
    }

    std::vector<double> poly1,poly2;
    poly1.push_back(1);
    poly1.push_back(-1);
    poly2.push_back(0);
    poly2.push_back(1);

    std::vector<double> v1,v2;
    v1.insert(v1.begin(),Pc.begin(),Pc.begin()+Pc.size()-1);
    v2.insert(v2.begin(),Pc.begin()+1,Pc.begin()+Pc.size());
    return rm::Algebre::Polynome(poly1)*deCasteljauPoly(v1)+rm::Algebre::Polynome(poly2)*deCasteljauPoly(v2);
}

/*!
 * \brief Algorithme récursif de DeCasteljau pour le calcul des coordonnées d'un point de la courbe
 *
 * Il s'agit d'une méthode privée de la structure, appelée par la méthode publique computePt
 *
 * \param pc points de controle considérés, à une itération donnée
 * \param t position du point recherché sur la courbe (entre 0 et 1)
 * \return coordonnées du point recherché
 */
cv::Point2d bezierCurve::deCasteljau(std::vector<cv::Point2d> Pc, float t)
{
    // Si un seul point de controle, la récursivité s'achève
    if(Pc.size()==1)
        return Pc[0];

    // v1 = pc - son dernier élément
    // v2 = pc - son premier élément
    std::vector<cv::Point2d> v1,v2;
    v1.insert(v1.begin(),Pc.begin(),Pc.begin()+Pc.size()-1);
    v2.insert(v2.begin(),Pc.begin()+1,Pc.begin()+Pc.size());
    return (1-t)*deCasteljau(v1,t)+t*deCasteljau(v2,t);
}


}
}
