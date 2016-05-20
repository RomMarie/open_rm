#include <open_rm/Mapping/localTopView.h>

#include <iostream>
namespace rm{
namespace Mapping{

/*! \brief Constructeur principal de la classe LocalTopView
 *  \param [in] pixPerMeter Echelle de la carte locale (en pixels par mètre)
 *  \param [in] size Taille de la carte locale (en pixels)
 */
LocalTopView::LocalTopView(const double pixPerMeter, const cv::Size size)
{
    _ppm=pixPerMeter;
    _size=size;
    reset(); // La carte est mise à zéro
}

/*! \brief Remise à zéro de la grille d'occupation
 */
void LocalTopView::reset()
{
    _occGrid=cv::Mat::zeros(_size,cv::DataType<cv::Vec3b>::type);
}

/*! \brief Ajout de points à la carte locale
 * \param [in] pts les points à ajouter (exprimés dans le repère robot)
 * \param [in] couleur couleur utilisée pour l'affichage (utile en cas de multi-capteurs)
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
void LocalTopView::addPts(const std::vector<cv::Point2f> pts, const cv::Vec3b couleur)
{
    // Le robot est toujours au centre de la grille d'occupation
    cv::Point centre=cv::Point((int)floor(_size.height/2.+0.5),(int)floor(_size.width/2.+0.5));

    // Pour chaque point
    for(unsigned int i=0;i<pts.size();i++)
    {
        // On calcule ses coordonnées dans la grille d'occupation
        int u=(int)floor(-_ppm*pts[i].x+centre.x+0.5);
        int v=(int)floor(-_ppm*pts[i].y+centre.y+0.5);

        // Si elles sont dans les limites de l'image, on l'affiche
        if(u>=0&&u<_occGrid.rows&&v>=0&&v<_occGrid.cols)
            _occGrid.at<cv::Vec3b>(u,v)=couleur;
    }
}

/*! \brief Fonction qui applique une transformation euclidienne à la carte locale
 *
 * La valeur attribuée à chaque point de la nouvelle grille
 * est obtenue par transformation inverse et interpolation d'ordre 0. Les points
 * hors de la première grille obtiennent un statut "inoccupé" (0,0,0)
 *
 * \param [in] R rotation à appliquer (en radian)
 * \param [in] t translation à appliquer suivant x et y (en mètres)
 * /!\ Si R=0, t correspond aux coordonnées du nouveau repère dans l'ancien
 */
void LocalTopView::move(const float R, const cv::Point2f t)
{
    cv::Mat newOccGrid=_occGrid.clone();

    // Le robot est toujours au centre de la grille d'occupation
    cv::Point centre=cv::Point((int)floor(_size.height/2.+0.5),(int)floor(_size.width/2.+0.5));

    for(int i=0;i<_occGrid.rows;i++)
    {
        for(int j=0;j<_occGrid.cols;j++)
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

            // Interpolation d'ordre 0 aux coordonnées pPrev pour attribuer la valeur correspondante
            double coef;
            cv::Vec3b som;

            if(pt.x>0&&pt.x<_occGrid.rows-1&&
                    pt.y>0&&pt.y<_occGrid.cols-1)
            {
                newOccGrid.at<cv::Vec3b>(i,j)=_occGrid.at<cv::Vec3b>((int)floor(pt.x+0.5),(int)floor(pt.y+0.5));
            }
            else
                newOccGrid.at<cv::Vec3b>(i,j)=cv::Vec3b(0,0,0);
        }
    }

    _occGrid=newOccGrid.clone();
}

/*! \brief Transforme la grille d'occupation en un masque binaire (pour squelettisation ou autre).
 *
 * Chaque point est transformé en droite radiale allant jusqu'aux frontières de l'image
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
cv::Mat LocalTopView::drawMask()
{
    cv::Mat mask=cv::Mat::zeros(_occGrid.size(),CV_8U); // Le masque résultat, mêmes dimensions que la grille
    cv::Point centre=cv::Point((int)floor(_size.height/2.+0.5),(int)floor(_size.width/2.+0.5));
    cv::Point pt;
    float theta;

    // Pour chaque point "non noir" de la grille d'occupation
    for(int i=0;i<_occGrid.rows;i++)
    {
        for(int j=0;j<_occGrid.cols;j++)
        {
            if(_occGrid.at<cv::Vec3b>(i,j)!=cv::Vec3b(0,0,0))
            {
                theta=atan2((float)(centre.y-j),(float)(centre.x-i)); // calcul de l'angle de la radiale
                pt.x=centre.x-(int)floor(_size.height*cos(theta)+0.5); // puis des coordonnées du point au bord de l'image
                pt.y=centre.y-(int)floor(_size.width*sin(theta)+0.5); // y appartenant
                cv::line(mask,cv::Point(j,i),cv::Point(pt.y,pt.x),255); // Enfin, traçage du segment correspondant
            }
        }
    }

    return mask.clone();
}

/*! \brief Fonction d'affichage de la grille d'occupation brute, i.e. sans les radiales
 *
 * \param [in] nom le nom de la fenêtre où a lieu l'affichage
 */
void LocalTopView::showPts(const std::string nom)
{
    cv::imshow(nom,_occGrid);
    cv::waitKey(1);
}

/*! \brief Fonction d'affichage du masque (avec les radiales) construit à partir de la carte locale.
 *
 * Les couleurs liées à chaque source sont conservées
 * \param [in] nom nom de la fenêtre où a lieu l'affichage
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
void LocalTopView::showMask(const std::string nom)
{
    cv::Mat mask=_occGrid.clone(); // A la différence de drawMask, ici mask est de type cv::Mat<cv::Vec3b>

    cv::Point centre=cv::Point((int)floor(_size.height/2.+0.5),(int)floor(_size.width/2.+0.5));
    cv::Point pt;
    float theta;

    for(int i=0;i<_occGrid.rows;i++)
    {
        for(int j=0;j<_occGrid.cols;j++)
        {
            if(_occGrid.at<cv::Vec3b>(i,j)!=cv::Vec3b(0,0,0))
            {
                theta=atan2((float)(centre.y-j),(float)(centre.x-i));
                pt.x=centre.x-(int)floor(_size.height*cos(theta)+0.5);
                pt.y=centre.y-(int)floor(_size.width*sin(theta)+0.5);

                // A la différence de drawMask, ici la radiale est tracée dans la couleur du point, et pas en blanc
                cv::line(mask,cv::Point(j,i),cv::Point(pt.y,pt.x),cv::Scalar(_occGrid.at<cv::Vec3b>(i,j)[0],
                         _occGrid.at<cv::Vec3b>(i,j)[1],
                        _occGrid.at<cv::Vec3b>(i,j)[2]));
            }
        }
    }


    cv::imshow(nom,mask);cv::waitKey(1);
}

/*! \brief Destructeur de la classe LocalTopView
 */
LocalTopView::~LocalTopView()
{

}


}
}
