#include <open_rm/Mapping/localOccGrid.h>

#include <iostream>
namespace rm{
namespace Mapping{

/*! \brief Constructeur par défaut qui génère une grille d'occupation vide
 * de taille prédéfinie, i.e. 600*600, avec une résolution de 100pixels / mètre.
 *
 * Ce constructeur représentel'environnement à une distance
 * de 3m autour du robot.
 */
localOccGrid::localOccGrid()
{
    _ppm=100;
    _size=cv::Size(600,600);
    _grid=cv::Mat::ones(_size,CV_32F)*0.5;
    _inc=(float)0.1;
    _dec=(float)0.1;
}

/*! \brief Constructeur principal de la classe qui spécifie les dimensions et la résolution
 * de la grille d'occupation.
 * \param [in] pixPerMeter Résolution de la grille (en pixels par mètre)
 * \param [in] size Dimensions de la représentation (en pixels)
 * \param [in] inc vitesse de maj de l'espace libre en % (0.1 par défaut)
 * \param [in] dec vitesse de maj des obstacles en % (0.1 par défaut)
 */
localOccGrid::localOccGrid(float pixPerMeter, cv::Size size, float inc, float dec)
{
    _ppm=pixPerMeter;
    _size=size;
    _grid=cv::Mat::ones(_size,CV_32F)*0.5;
    _inc=inc;
    _dec=dec;
}

/*! \brief Destructeur par défaut
 */
localOccGrid::~localOccGrid()
{

}

/*! \brief Ajout de points à la carte locale
 *
 * \param [in] pts points à ajouter (exprimés en (x,y) dans le repère robot)
 *
 * Convention utilisée :
 *     v
 *   ----->----------------------------------
 * u |                                      |
 *   |                                      |
 *   v                  ^                   |
 *   |                  | x                 |
 *   |                  |                   |
 *   |             <-----                   |
 *   |                y                     |
 *   |                                      |
 *   |                                      |
 *   |                                      |
 *   |                                      |
 *   ----------------------------------------
 */
void localOccGrid::addWPts(const std::vector<cv::Point2f> pts)
{

    // Le robot est toujours au centre de la grille d'occupation
    cv::Point centre=cv::Point((int)floor(_size.height/2.+0.5),(int)floor(_size.width/2.+0.5));

    // On commence par convertir les points en coordonnées polaires
    std::vector<cv::Point2f> pts_pol;
    for(unsigned int i=0;i<pts.size();i++)
    {
        float rho=sqrt(pts[i].x*pts[i].x+pts[i].y*pts[i].y);
        float theta=atan2(pts[i].y,pts[i].x);
        pts_pol.push_back(cv::Point2f(rho,theta));
    }

    // Pour chaque point de la grille, on cherche ceux
    // dans la même orientation qu'un des points en entrée
    // mais à une distance inférieure au robot.
    // On admet une tolérance de 0.5 degré sur l'orientation
    // /!\ cette tolérance est potentiellement à améliorer
    // /!\ Les points à une distance < 5cm du point en entrée
    // sont considérés de manière particulière
    for(int i=0;i<_grid.rows;i++)
    {
        for(int j=0;j<_grid.cols;j++)
        {
            // Coordonnées rho,theta du points i,j
            float rho=sqrt((centre.x-i)/_ppm*(centre.x-i)/_ppm+(centre.y-j)/_ppm*(centre.y-j)/_ppm);
            float theta=atan2((centre.y-j)/_ppm,(centre.x-i)/_ppm);

            for(unsigned int k=0;k<pts_pol.size();k++)
            {
                // Calcul de l'orientation relative entre (i,j) et pts_pol[i]
                double difTheta;
                if(theta>pts_pol[k].y)
                    difTheta=theta-pts_pol[k].y;
                else
                    difTheta=pts_pol[k].y-theta;
                if(difTheta>=CV_PI)
                    difTheta=2*CV_PI-difTheta;


                // Vérification si le point est dans l'espace libre défini par pts_pol[i]
                // c'est à dire s'il est à plus de 5cm du point et dans une orientation de +/- 0.5°
                if(difTheta<=CV_PI/360.&&pts_pol[k].x-rho>0.05)
                {

                    cv::Point2f pt=cv::Point2f(centre.x-i/_ppm,centre.y-j/_ppm);
                    bool go=1;
                    for(unsigned int l=0;l<pts_pol.size();l++)
                    {
                        if((pt.x-pts[l].x)*(pt.x-pts[l].x)+(pt.y-pts[l].y)*(pt.y-pts[l].y)<=0.05*0.05)
                        {
                            go=0;
                            break;
                        }
                    }

                    // Dans ce cas, le point (i,j) est dans l'espace libre détecté
                    if(go)
                    {
                        increment(i,j);
                    }
                    // On n'incrémente qu'une fois un point de la grille par itération
                    break;
                }
            }
        }
    }

    // On décrémente toutes les cellules à distance <5cm des points
    // en pondérant leur distance au point (fonction triangle)
    for(unsigned int i=0;i<pts.size();i++)
    {
        // On calcule ses coordonnées dans la grille d'occupation
        int u=(int)floor(-_ppm*pts[i].x+centre.x+0.5);
        int v=(int)floor(-_ppm*pts[i].y+centre.y+0.5);

        for(int j=-5;j<=5;j++)
        {
            for(int k=-5;k<=5;k++)
            {
                if(j*j+k*k<=5*5&&
                        u+j>0&&u+j<_grid.rows&&
                        v+k>0&&v+k<_grid.cols)
                {
                    decrement(u+j,v+k,5-(float)sqrt((float)j*j+k*k));
                }
            }
        }
    }

}

/*! \brief Fonction qui applique une transformation euclidienne à la grille
 * d'occupation.
 *
 * La valeur attribuée à chaque point de la nouvelle grille
 * est obtenue par transformation inverse et interpolation. Les points
 * hors de la première grille obtiennent un statut "non défini" (0.5).
 *
 * \param [in] R rotation à appliquer (en radian)
 * \param [in] t translation à appliquer suivant x et y (en mètres)
 * /!\ Si R=0, t correspond aux coordonnées du nouveau repère dans l'ancien
 */
void localOccGrid::move(const float R, const cv::Point2f t)
{
    cv::Mat newOccGrid=_grid.clone();

    // Le robot est toujours au centre de la grille d'occupation
    cv::Point centre=cv::Point((int)floor(_size.height/2.+0.5),(int)floor(_size.width/2.+0.5));

    for(int i=0;i<_grid.rows;i++)
    {
        for(int j=0;j<_grid.cols;j++)
        {
            cv::Point2f pCur; // Coordonnées métriques du point (i,j) de la nouvelle grille
            pCur.x=(centre.x-i)/_ppm;
            pCur.y=(centre.y-j)/_ppm;

            // Calcul des coordonnées de pCur dans l'ancien repère
            cv::Point2f pPrev;
            pPrev.x=pCur.x*cos(R)-pCur.y*sin(R)+t.x;
            pPrev.y=pCur.x*sin(R)+pCur.y*cos(R)+t.y;

            // Déduction des coordonnées dans l'ancienne grille d'occupation
            cv::Point2f pt;
            pt.x=centre.x-pPrev.x*_ppm;
            pt.y=centre.y-pPrev.y*_ppm;

            // Interpolation d'ordre 1 aux coordonnées pPrev pour attribuer la valeur correspondante
            double coef;
            double som=0;

            if(pt.x>0&&pt.x<_grid.rows-1&&
                    pt.y>0&&pt.y<_grid.cols-1)
            {
                coef=0.25*(pt.x-floor(pt.x)+pt.y-floor(pt.y));
                som+=coef*_grid.at<float>((int)floor(pt.x),(int)floor(pt.y));

                coef=0.25*(pt.x-floor(pt.x)-pt.y+floor(pt.y)+1);
                som+=coef*_grid.at<float>((int)floor(pt.x),(int)(1+floor(pt.y)));

                coef=0.25*(-pt.x+1+floor(pt.x)+pt.y-floor(pt.y));
                som+=coef*_grid.at<float>((int)(1+floor(pt.x)),(int)floor(pt.y));

                coef=0.25*(-pt.x+1+floor(pt.x)-pt.y+1+floor(pt.y));
                som+=coef*_grid.at<float>((int)(1+floor(pt.x)),(int)(1+floor(pt.y)));
            }
            else
                som=0.5;

            // On attribue la nouvelle valeur à la case (avant recopie complète à la fin)
            newOccGrid.at<float>(i,j)=(float)som;
        }
    }

    _grid=newOccGrid.clone();
}

/*! \brief Applique un double seuillage sur la grille d'occupation
 * pour générer un masque à 3 états.
 *
 * Les états sont définis par les niveaux de gris suivants :
 * - 0 = obstacle
 * - 128 = indéterminé
 * - 255 = libre
 *
 * \param [in] tMin seuil en dessous duquel un point est défini comme obstacle
 * \param [in] tMax seuil au delà duquel un point est défini comme libre
 * \returns masque à trois états
 */
cv::Mat localOccGrid::build3States(const float tMin, const float tMax)
{
    cv::Mat res(_grid.size(),CV_8U);

    for(int i=0;i<res.rows;i++)
    {
        for(int j=0;j<res.cols;j++)
        {
            if(_grid.at<float>(i,j)<=tMin)
                res.at<uchar>(i,j)=0;
            else if(_grid.at<float>(i,j)>=tMax)
                res.at<uchar>(i,j)=255;
            else
                res.at<uchar>(i,j)=128;
        }
    }
    return res.clone();
}

/*! \brief Fonction qui incrémente la cellule (i,j) de la grille d'occupation
 * \param [in] i,j coordonnées de la cellule
 */
void localOccGrid::increment(int i, int j)
{
    _grid.at<float>(i,j)=std::min<float>(1,_grid.at<float>(i,j)+_inc);
}

/*! \brief Fonction qui décrémente la cellule (i,j) de la grille d'occupation
 * en appliquant un coefficient réducteur coef compris entre 0 et 1
 * \param [in] i,j coordonnées de la cellule considérée
 * \param [in] coef coefficient à appliquer au décrément
 */
void localOccGrid::decrement(int i, int j, float coef)
{
    _grid.at<float>(i,j)=std::max<float>(0,_grid.at<float>(i,j)-(_dec*coef));
}

/*! \brief Fonction qui affiche la grille d'occupation
 * dans une fenêtre opencv. Le nom de la fenêtre
 * est donné en argument
 */
void localOccGrid::show(const std::string nom)
{
    cv::imshow(nom,_grid);cv::waitKey(1);
}

}
}
