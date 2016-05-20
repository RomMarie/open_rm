#include <open_rm/VisionOmni/birdeyeview.h>

namespace rm{

namespace VisionOmni{

/*! \brief Constructeur par défaut
 *
 * N'utiliser ce constructeur qu'en association avec la méthode
 * init qui permet de définir la look up table obligatoire
 */
BirdEyeView::BirdEyeView()
{

}

/*! \brief Constructeur principal de la classe.
 *
 * \param pc Paramètres de calibration du capteur omni
 * \param pixPerMeter résolution de la bird eye view (en pixels par mètre)
 * \param dim dimensions de la représentation (en pixels)
 */
BirdEyeView::BirdEyeView(const ParamsCalib &pc, double pixPerMeter, cv::Size dim)
{
    init(pc,pixPerMeter,dim);
}

/*! \brief Projette une image sur la bird eye view initialisée
 *
 * \param img Image omnidirectionnelle à projeter
 * \return bird eye view construite à partir de l'image
 *
 * \warning Il faut absolument avoir appelé la méthode init auparavant
 * (soit avec le constructeur principal, soit par un appel explicite).
 */
cv::Mat BirdEyeView::project(const cv::Mat &img)
{
    cv::Mat res=cv::Mat::zeros(_lut.size(),img.type());
    for(int i=0;i<_lut.rows;i++)
    {
        for(int j=0;j<_lut.cols;j++)
        {
            cv::Point p=_lut.at<cv::Point>(i,j);
            if(p.x>=0&&p.x<img.rows&&p.y>=0&&p.y<img.cols)
            {
                switch(img.type())
                {
                case CV_8U:res.at<uchar>(i,j)=img.at<uchar>(p.x,p.y);break;
                case CV_32F:res.at<float>(i,j)=img.at<float>(p.x,p.y);break;
                case CV_64F:res.at<double>(i,j)=img.at<double>(p.x,p.y);break;
                case CV_8UC3:res.at<cv::Vec3b>(i,j)=img.at<cv::Vec3b>(p.x,p.y);break;
                default:
                    std::cout<<"type d'image non supporté (bird eye view). Fin prematuree"<<std::endl;
                    exit(-1);
                    break;
                }
            }
        }
    }

    return res.clone();
}

/*! \brief Projette une image sur la bird eye view sans LUT
 *
 * \param pc Paramètres de calibrage
 * \param img Image omnidirectionnelle à projeter
 * \param pixPerMeter résolution de la bird eye view (en pixels par mètre)
 * \param dim Dimensions de l'image représentant la bird eye view
 * \return bird eye view construite à partir de l'image
 *
 * \warning Ne doit pas être utilisé en pratique. Existe juste pour illustrer
 * l'intérêt de la LUT
 */
cv::Mat BirdEyeView::projectSansLUT(const ParamsCalib &pc,const cv::Mat &img, double pixPerMeter, cv::Size dim)
{
    cv::Mat res=cv::Mat::zeros(_lut.size(),img.type());
    double u0=pc.u0();
    double v0=pc.v0();
    double au=pc.au();
    double av=pc.av();
    double xi=pc.xi();
    double h=pc.h();

    double X0=dim.height/2.;
    double Y0=dim.width/2.;

    // On déclare les variables utilisées
    double X,Y,Z,x,y,u,v;

    // Et pour chaque pixel de la lut, après avoir défini les coordonnées 3d correspondantes,
    // on en déduit les coordonnées de sa projection dans l'image omni.
    Z=h;
    for(int i=0;i<_lut.rows;i++)
    {
        // On calcule les coordonnées (X,Y,H) du point 3D
        X=(X0-i)/pixPerMeter;
        for(int j=0;j<_lut.cols;j++)
        {
            Y=(Y0-j)/pixPerMeter;

            // On projette le point dans le plan image normalisé
            x=X/(h+sqrt(X*X+Y*Y+h*h)*xi);
            y=Y/(h+sqrt(X*X+Y*Y+h*h)*xi);

            // On en déduit les coordonnées pixelliques dans l'image omni
            u=floor(u0-x*au+0.5);
            v=floor(v0-y*av+0.5);

            cv::Point p=cv::Point(u,v);
            if(p.x>=0&&p.x<img.rows&&p.y>=0&&p.y<img.cols)
            {
                switch(img.type())
                {
                case CV_8U:res.at<uchar>(i,j)=img.at<uchar>(p.x,p.y);break;
                case CV_32F:res.at<float>(i,j)=img.at<float>(p.x,p.y);break;
                case CV_64F:res.at<double>(i,j)=img.at<double>(p.x,p.y);break;
                case CV_8UC3:res.at<cv::Vec3b>(i,j)=img.at<cv::Vec3b>(p.x,p.y);break;
                default:
                    std::cout<<"type d'image non supporté (bird eye view). Fin prematuree"<<std::endl;
                    exit(-1);
                    break;
                }
            }

        }
    }
    return res.clone();
}

/*! \brief Initialise la bird eye view.
 *
 * \param pc Paramètres de calibration du capteur omni
 * \param pixPerMeter résolution de la bird eye view (en pixels par mètre)
 * \param dim dimensions de la représentation (en pixels)
 */
void BirdEyeView::init(const ParamsCalib &pc, double pixPerMeter, cv::Size dim)
{
    // On alloue l'espace requis pour la look up table
    _lut=cv::Mat::zeros(dim,cv::DataType<cv::Point>::type);


    // On récupère les paramètres de calibration pour une utilisation simplifiée
    double u0=pc.u0();
    double v0=pc.v0();
    double au=pc.au();
    double av=pc.av();
    double xi=pc.xi();
    double h=pc.h();

    double X0=dim.height/2.;
    double Y0=dim.width/2.;

    // On déclare les variables utilisées
    double X,Y,Z,x,y,u,v;

    // Et pour chaque pixel de la lut, après avoir défini les coordonnées 3d correspondantes,
    // on en déduit les coordonnées de sa projection dans l'image omni.
    Z=h;
    for(int i=0;i<_lut.rows;i++)
    {
        // On calcule les coordonnées (X,Y,H) du point 3D
        X=(X0-i)/pixPerMeter;
        for(int j=0;j<_lut.cols;j++)
        {
            Y=(Y0-j)/pixPerMeter;

            // On projette le point dans le plan image normalisé
            x=X/(h+sqrt(X*X+Y*Y+h*h)*xi);
            y=Y/(h+sqrt(X*X+Y*Y+h*h)*xi);

            // On en déduit les coordonnées pixelliques dans l'image omni
            u=floor(u0-x*au+0.5);
            v=floor(v0-y*av+0.5);
            // Et on écrit dans la lut les coordonnées trouvées
            _lut.at<cv::Point>(i,j)=cv::Point((int)u,(int)v);

        }
    }
}

/*! \brief Destructeur de la classe
 *
 * N'a aucun intérêt particulier.
 */
BirdEyeView::~BirdEyeView()
{

}

}}
