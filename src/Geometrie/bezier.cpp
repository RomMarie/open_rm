#include <open_rm/Geometrie/bezier.h>
#include <open_rm/Algebre/equations.h>
#include <open_rm/Algebre/polynomes.h>

namespace rm{
namespace Geometrie{

/*!
 * \brief Constructeur principal
 * \param ptsControle points de controle de la courbe de Bézier
 */
bezierCurve::bezierCurve(std::vector<cv::Point2d> ptsControle)
{
    pc=ptsControle;
}

/*!
 * \brief Indique le degré de la courbe portée par la structure
 * \return Degré de la courbe de Bézier
 */
unsigned int bezierCurve::degre()
{
    return pc.size()-1;
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

    pc=pc1;

    return bezierCurve(pc2);

}

/*!
 * \brief Calcule les coordonnées d'un point appartenant à la courbe
 * \param t position du point considéré (entre 0 et 1)
 * \return coordonnées du point appartenant à la courbe
 */
cv::Point2d bezierCurve::computePt(double t)
{
    return deCasteljau(pc,t);
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
    for(unsigned int i=0;i<pc.size();i++)
        pts.push_back(pc[i]);
    cv::Rect bb=cv::boundingRect(pts);
    if(bb.x<0||bb.y<0||bb.height+bb.y>=img.rows||bb.width+bb.x>=img.cols)
        return;

    // Si les points de controle doivent etre affichés
    if(withPC){
        for(unsigned int i=0;i<pc.size()-1;i++){
            cv::circle(img,pc[i],3,color,CV_FILLED);
            cv::line(img,pc[i],pc[i+1],color);
        }
        cv::circle(img,pc[pc.size()-1],3,color,CV_FILLED);
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
    if(pc.size()>ind) // Si le point de controle existe
        pc[ind]+=dir*scale; // On le déplace
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

}

/*!
 * \brief Détermine le point d'une courbe de Bezier quadratique le plus proche d'un point donné
 * \param pt Point dont on cherche la projection
 * \return indice t (entre 0 et 1) du point de la courbe le plus proche
 * \note basé sur http://blog.gludion.com/2009/08/distance-to-quadratic-bezier-curve.html
 */
double bezierCurve::closestPt(cv::Point2d pt)
{
    cv::Point2d A=pc[1]-pc[0];
    cv::Point2d B=pc[0]+pc[2]-2*pc[1];
    cv::Point2d pos=pc[0]-pt;

    // On cherche les points P de la courbe de Bezier tels que PM.(dP / dt) = 0
    // où M est le point pt passé en argument.
    // Ils correspondent aux solutions d'une équation du troisieme ordre
    double a=B.x*B.x+B.y*B.y;
    double b=3.*(A.x*B.x+A.y*B.y);
    double c=2.*(A.x*A.x+A.y*A.y)+pos.x*B.x+pos.y*B.y;
    double d=pos.x*A.x+pos.y*A.y;

    std::vector<double> sol= rm::Algebre::thirdOrderEq(a, b, c, d);
    double t;
    double dist;
    double tMin=-1;
    double distMin=1000000;

    if(sol.size()>0)
    {
        // On cherche le point le plus proche parmi les solutions de l'équation du troisième ordre
        for(unsigned int i = 0; i<sol.size(); i++)
        {
            t = sol[i];
            if(t>=-0.001&&t<0)t=0; // On lutte contre les erreurs d'arrondis
            if(t<=1.001&&t>1)t=1; // idem
            if(t>=0&&t<=1)
            {
                pos = computePt(t);
                dist = cv::norm(pt-pos);
                if(dist<distMin)
                {
                    tMin = t;
                    distMin = dist;
                }
            }
        }
    }

    return tMin;
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

/*!
 * \brief Fonction g définie dans https://hal.archives-ouvertes.fr/file/index/docid/518379/filename/Xiao-DiaoChen2007c.pdf
 * \param pt Point dont on cherche la distance à la courbe
 * \param t Point d'évaluation de g sur la courbe (entre 0 et 1)
 * \return Evaluation de g(t)
 */
double bezierCurve::g(cv::Point2d pt, double t)
{
    std::vector<double> coefsX,coefsY;
    coefsX.push_back(pc[0].x);
    coefsX.push_back(3*(pc[1].x-pc[0].x));
    coefsX.push_back(3*(pc[0].x-2*pc[1].x+pc[2].x));
    coefsX.push_back(pc[3].x+3*pc[1].x-3*pc[2].x-pc[0].x);
    coefsX.push_back(pc[0].y);
    coefsX.push_back(3*(pc[1].y-pc[0].y));
    coefsX.push_back(3*(pc[0].y-2*pc[1].y+pc[2].y));
    coefsX.push_back(pc[3].y+3*pc[1].y-3*pc[2].y-pc[0].y);

    rm::Algebre::Polynome Vx(coefsX),Vy(coefsY);
    rm::Algebre::Polynome Vprimex(Vx.derivate()),Vprimey(Vy.derivate());


}

}
}
