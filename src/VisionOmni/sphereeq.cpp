#include <open_rm/VisionOmni/sphereeq.h>


namespace rm{
namespace VisionOmni{

/*! \brief Constructeur par défaut de la classe
 *
 * \warning Ne doit être utilisé qu'avec un appel explicite de la méthode init
 */
SphereEq::SphereEq()
{

}

/*! \brief Constructeur principal de la classe
 * \param pc Paramètres de calibration de la caméra omni
 * \param nTheta Nombre de points d'échantillonnage suivant théta
 * \param nPhi Nombre de points d'échantillonnage suivant phi
 *
 * Fait appel en interne à la méthode init en lui fournissant les
 * paramètres de la caméra et les propriétés désirées de la sphère.
 *
 * Par exemple, si nTheta = 180 et nPhi = 360, la sphère aura une résolution
 * de 1 point par degré.
 */
SphereEq::SphereEq(const rm::VisionOmni::ParamsCalib &pc, int nTheta, int nPhi)
{
    init(pc,nTheta,nPhi);
}

/*! \brief Destructeur par défaut de la classe
 */
SphereEq::~SphereEq()
{

}

/*! \brief Projette une image sur la sphère définie par la méthode init
 *
 * \param img Image omni à projeter
 * \param thetaMin Borne inférieure de la vue
 * \param thetaMax Borne supérieure de la vue
 * \return Image paramétrée en (théta,phi) représentant la sphère
 * \note l'image projetée peut être de type uchar, float, double ou cv::Vec3b
 * \todo ajouter les types supportés cv::Vec3f, cv::Vec3d et int
 *
 * \warning Il faut absolument avoir appelé au préalable la fonction init
 * (soit par le constructeur principal, soit explicitement).
 */
cv::Mat SphereEq::project(const cv::Mat &img, int thetaMin, int thetaMax)
{
    cv::Mat res(_lut.size(),img.type());

    // Pour chaque point de la sphère
    for(int i=0;i<_lut.rows;i++)
    {
        for(int j=0;j<_lut.cols;j++)
        {
            switch(img.type()) // La sortie dépend du type de l'entrée
            {
            case CV_8U: // cas <uchar>
                // Si le point n'est pas dans les frontières de l'image
                if(_lut.at<cv::Point>(i,j).x<0||
                        _lut.at<cv::Point>(i,j).y<0||
                        _lut.at<cv::Point>(i,j).x>=img.rows||
                        _lut.at<cv::Point>(i,j).y>=img.cols)
                {
                    res.at<uchar>(i,j)=255; // On le met en blanc
                }
                else // Sinon
                {
                    res.at<uchar>(i,j)=img.at<uchar>(_lut.at<cv::Point>(i,j).x,_lut.at<cv::Point>(i,j).y); // On recopie la valeur lue dans la lut
                }
                break;
            case CV_64F: // Cas <double> (cf cas <uchar> pour détails)
                if(_lut.at<cv::Point>(i,j).x<0||
                        _lut.at<cv::Point>(i,j).y<0||
                        _lut.at<cv::Point>(i,j).x>=img.rows||
                        _lut.at<cv::Point>(i,j).y>=img.cols)
                {
                    res.at<double>(i,j)=255;
                }
                else
                {
                    res.at<double>(i,j)=img.at<double>(_lut.at<cv::Point>(i,j).x,_lut.at<cv::Point>(i,j).y);
                }
                break;
            case CV_32F: // Cas <float> (cf cas <uchar> pour détails)
                if(_lut.at<cv::Point>(i,j).x<0||
                        _lut.at<cv::Point>(i,j).y<0||
                        _lut.at<cv::Point>(i,j).x>=img.rows||
                        _lut.at<cv::Point>(i,j).y>=img.cols)
                {
                    res.at<float>(i,j)=255;
                }
                else
                {
                    res.at<float>(i,j)=img.at<float>(_lut.at<cv::Point>(i,j).x,_lut.at<cv::Point>(i,j).y);
                }
                break;
            case CV_8UC3: // Cas <cv::Vec3b> (cf cas <cv::Vec3b> pour détails)
                if(_lut.at<cv::Point>(i,j).x<0||
                        _lut.at<cv::Point>(i,j).y<0||
                        _lut.at<cv::Point>(i,j).x>=img.rows||
                        _lut.at<cv::Point>(i,j).y>=img.cols)
                {
                    res.at<cv::Vec3b>(i,j)=cv::Vec3b(255,255,255);
                }
                else
                {
                    res.at<cv::Vec3b>(i,j)=img.at<cv::Vec3b>(_lut.at<cv::Point>(i,j).x,_lut.at<cv::Point>(i,j).y);
                }
                break;
            }
        }
    }
    if(thetaMin==-1&&thetaMax==-1)
        return res.clone();
    else
    {
        cv::Mat res2(res,cv::Rect(0,thetaMin,res.cols,thetaMax-thetaMin));
        return res2.clone();
    }
}

/*! \brief Initialise une sphère d'équivalence
 * \param pc Paramètres de calibration de la caméra omni
 * \param nTheta Nombre de points d'échantillonnage suivant théta
 * \param nPhi Nombre de points d'échantillonnage suivant phi
 *
 * Chaque coordonée \f$ (\theta,\phi) \f$ est associé à une coordonée image \f$ (u,v) \f$
 * par une interpolation d'ordre 0,à partir des équations suivantes :
 *
 * \f$ \left(\begin{array}{c}
 * u\\ v\\ 1\end{array}\right) =
 * \frac{sin(\theta)}{cos(\theta)-\xi}*K*
 * \left(\begin{array}{c}
 * cos(\phi)\\
 * sin(\phi)\\
 * (cos(\theta)-\xi)/sin(\theta)\end{array}\right)
 * \f$
 * où \f$ K \f$ est la matrice des paramètres intrinsèques de la caméra
 */
void SphereEq::init(const rm::VisionOmni::ParamsCalib &pc, int nTheta, int nPhi)
{
    _lut.create(nTheta,nPhi,cv::DataType<cv::Point>::type);
    double theta,phi;
    double frac; // pour stocker sin(t)/(cos(t)+xi)

    // Pour chaque couple (theta,phi), on calcule et mémorise les coordonnées du point image le plus proche
    //		(u)     sin(t)		   (       cos(p)       )
    //		(v)= ----------- . K . (       sin(p)       )
    //		(1)   cos(t)-xi        ( (cos(t)-xi)/sin(t) )
    for(int i=0;i<nTheta;i++)
    {
        theta=((float)i*CV_PI)/nTheta;
        if(cos(theta)!=pc.xi()) // sinon division par 0
            frac=sin(theta)/(cos(theta)-pc.xi());
        for(int j=0;j<nPhi;j++)
        {
            phi=(float)j*(2*CV_PI)/nPhi;
            if(cos(theta)>=pc.xi())_lut.at<cv::Point>(i,j)=cv::Point(-1,-1); // Ces points ne peuvent pas se projeter dans l'image
            else _lut.at<cv::Point>(i,j)=cv::Point((int)floor(frac*cos(phi)*pc.au()+pc.u0()+0.5),
                                                  (int)floor(frac*sin(phi)*pc.av()+pc.v0()+0.5));
        }
    }
}

}
}

