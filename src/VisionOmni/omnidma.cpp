#include <open_rm/VisionOmni/omnidma.h>

namespace rm{
namespace VisionOmni{

/*! \brief Constructeur par défaut.
 *
 * N'a aucune action particulière.
 *
 * \warning ne peut être utilisé qu'en association avec la méthode
 * init, afin d'initialiser la look up table.
 */
OmniDMA::OmniDMA()
{

}

/*! \brief Constructeur principal de la classe.
 *
 * \param pc paramètres du capteur omni
 *
 * Fait appel à la méthode init pour initialiser
 * la look up table.
 */
OmniDMA::OmniDMA(const ParamsCalib &pc)
{
    init(pc);
}

/*! \brief Destructeur de la classe
 *
 * N'a aucune action particulière.
 */
OmniDMA::~OmniDMA()
{

}

/*! \brief Effectue les précalculs nécessaires à l'algorithme.
 *
 * \param pc Paramètres du capteur omni
 *
 * Pour chaque pixel dans le domaine de l'image (porté par resU
 * et resV dans pc), on calcule et mémorise ses coordonnées
 * dans le plan 3D d'équation \f$Z=1\f$. Pour tout point \f$m=(x,y)\f$
 * du plan image normalisé, les coordonnées \f$M=(X,Y,1)\f$ du point 3D correspondant
 * sont données par :
 *
 * \f$ \left\{\begin{array}{l}X=A(m)x/(A(m)-\xi)\\
 * Y=A(m)y/(A(m)-\xi)\end{array}\right.,\ A(m)=\frac{\xi+\sqrt{1+(1-\xi^2)(x^2+y^2)}}{x^2+y^2+1}\f$
 */
void OmniDMA::init(const ParamsCalib &pc)
{
    // On stocke les coordonnées du centre de projection pour utilisation ultérieure
    _c.x=pc.resU();
    _c.y=pc.resV();

    _lut=cv::Mat::zeros(pc.resU(),pc.resV(),cv::DataType<cv::Point2d>::type);

    double x,y;
    double A;

    for(int i=0;i<_lut.rows;i++)
    {
        x=(pc.u0()-i)/pc.au();
        for(int j=0;j<_lut.cols;j++)
        {
            y=(pc.v0()-j)/pc.av();
            A=(pc.xi()+(double)sqrt(1+(1-pc.xi()*pc.xi())*(x*x+y*y)))/(x*x+y*y+1);
            _lut.at<cv::Point2d>(i,j)=cv::Point2d(A*x/(A-pc.xi()),A*y/(A-pc.xi()));
        }
    }
}

/*! \brief Calcule le ODMA (sous forme de medial axis transform) de l'espace navigable
 *
 * \param mask masque binaire définissant l'espace libre omnidirectionnel
 * \param delta paramètre du DMA exprimé en pixels dans l'image omni
 * \return le squelette calculé par le ODMA avec pour chaque point sa distance au bord
 *
 * \note le delta utilisé par le ODMA est la distance adaptée générée en utilisant le
 * delta passé par l'utilisateur. Il faut donc bien définir delta en pixels dans l'image
 * omni, et pas en m dans le plan du sol.
 * \warning Il faut absolument que la méthode init ait été appelée au moins une fois
 * (via le constructeur principal ou explicitement) avant d'utiliser cette méthode.
 */
cv::Mat OmniDMA::computeWithDT(const cv::Mat &mask,int delta)
{
    cv::Mat skel(mask.size(),CV_8U); // Imagee résultat (format CV_8U, pour masque binaire)
    cv::Mat D(mask.size(),cv::DataType<double>::type); // Distance Transform
    cv::Mat P(mask.size(),cv::DataType<cv::Point>::type); // Projection Transform

    // Initialisation de D et P pour chaque point du masque
    for(int i=0;i<mask.rows;i++)
    {
        for(int j=0;j<mask.cols;j++)
        {
            skel.at<uchar>(i,j)=0;
            if(mask.at<uchar>(i,j)==255)
            {
                D.at<double>(i,j)=10000*10000;
            }
            else
            {
                D.at<double>(i,j)=0;
                P.at<cv::Point>(i,j)=cv::Point(i,j);
            }
        }
    }

    // Algorithme Danielsson scan 1 : propagation des projections (donc distances)
    // de haut en bas et de gauche à droite
    // Masques considérés
    // o o o puis - - -
    // o - -      - - o
    // - - -      - - -
    for(int i=1;i<mask.rows;i++)
    {
        for(int j=1;j<mask.cols-1;j++)
        {
            if(mask.at<uchar>(i,j)==0)
            {
                D.at<double>(i,j)=0;
                P.at<cv::Point>(i,j)=cv::Point(i,j);
            }
            else
            {
                for(int k=-1;k<=0;k++)
                {
                    for(int l=-1;l<=1;l++)
                    {
                        // o o o
                        // o - -
                        // - - -
                        if(k==-1||l==-1)
                        {
                            double d=dSol(cv::Point(i,j),P.at<cv::Point>(i+k,j+l));
                            if(d<D.at<double>(i,j))
                            {
                                D.at<double>(i,j)=d;
                                P.at<cv::Point>(i,j)=P.at<cv::Point>(i+k,j+l);
                            }
                        }
                    }
                }
            }
        }

        for(int j=mask.cols-2;j>=0;j--)
        {
            // - - -
            // - - o
            // - - -
            if(mask.at<uchar>(i,j)==255)
            {
                double d=dSol(cv::Point(i,j),P.at<cv::Point>(i,j+1));
                if(d<D.at<double>(i,j))
                {
                    D.at<double>(i,j)=d;
                    P.at<cv::Point>(i,j)=P.at<cv::Point>(i,j+1);
                }
            }
        }
    }

    // Algorithme Danielsson scan 2 : propagation des projections (donc distances)
    // de bas en haut et de droite à gauche
    // Masques considérés
    // - - - puis - - -
    // - - o      o - -
    // o o o      - - -
    for(int i=mask.rows-2;i>=0;i--)
    {
        for(int j=mask.cols-2;j>=1;j--)
        {
            if(mask.at<uchar>(i,j)==255)
            {
                for(int k=1;k>=0;k--)
                {
                    for(int l=1;l>=-1;l--)
                    {
                        // - - -
                        // - - o
                        // o o o
                        if(k==1||l==1)
                        {
                            double d=dSol(cv::Point(i,j),P.at<cv::Point>(i+k,j+l));
                            if(d<D.at<double>(i,j))
                            {
                                D.at<double>(i,j)=d;
                                P.at<cv::Point>(i,j)=P.at<cv::Point>(i+k,j+l);
                            }
                        }
                    }
                }
            }
        }
        for(int j=1;j<mask.cols;j++)
        {
            if(mask.at<uchar>(i,j)==255)
            {
                // - - -
                // o - -
                // - - -
                double d=dSol(cv::Point(i,j),P.at<cv::Point>(i,j-1));
                if(d<D.at<double>(i,j))
                {
                    D.at<double>(i,j)=d;
                    P.at<cv::Point>(i,j)=P.at<cv::Point>(i,j-1);
                }
            }
        }
    }

    // Après les deux scans (ici), P et D sont connus pour chaque point de l'image
    // Avec D = 0 si le point appartient au background

    // La valeur de delta est définie à partir de la taille de la zone aveugle dans l'image
    double deltaA=dSol(cv::Point(_c.x,_c.y),cv::Point(_c.x+delta,_c.y));

    // A partir d'ici, algorithme du delta medial axis standard (cf namespace AxeMedian)
    for(int i=1;i<P.rows-1;i++)
    {
        for(int j=1;j<P.cols-1;j++)
        {
            if(D.at<double>(i,j)>0) // Si (i,j) est un point de la forme
            {
                for(int n=0;n<=1;n++) // Pour (i-1,j) et (i,j-1)
                {
                    int k=-n,l=n-1;
                    int nstep=cv::max(
                                abs(P.at<cv::Point>(i,j).x-P.at<cv::Point>(i+k,j+l).x),
                                abs(P.at<cv::Point>(i,j).y-P.at<cv::Point>(i+k,j+l).y)
                                );
                    double d=(P.at<cv::Point>(i,j).x-P.at<cv::Point>(i+k,j+l).x)*(P.at<cv::Point>(i,j).x-P.at<cv::Point>(i+k,j+l).x)+
                            (P.at<cv::Point>(i,j).y-P.at<cv::Point>(i+k,j+l).y)*(P.at<cv::Point>(i,j).y-P.at<cv::Point>(i+k,j+l).y);
                    if(d>2)
                    {
                        double step=1./nstep;
                        for(double a=step;a<=1-step;a+=step) // Pour chaque point de [P(x)P(y)]
                        {
                            double d2=D.at<double>(
                                        (int)floor(a*P.at<cv::Point>(i,j).x+(1-a)*P.at<cv::Point>(i+k,j+l).x+0.5),
                                        (int)floor(a*P.at<cv::Point>(i,j).y+(1-a)*P.at<cv::Point>(i+k,j+l).y+0.5)
                                        );
                            if(d2>deltaA)
                            {
                                if(D.at<double>(i,j)>=D.at<double>(i+k,j+l))skel.at<double>(i,j)=D.at<double>(i,j);
                                if(D.at<double>(i,j)<=D.at<double>(i+k,j+l))skel.at<double>(i+k,j+l)=D.at<double>(i+k,j+l);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    return skel.clone();
}

/*! \brief Calcule le ODMA de l'espace navigable
 *
 * \param mask masque binaire définissant l'espace libre omnidirectionnel
 * \param delta paramètre du DMA exprimé en pixels dans l'image omni
 * \return le squelette calculé par le ODMA sous forme de masque
 *
 * \note le delta utilisé par le ODMA est la distance adaptée générée en utilisant le
 * delta passé par l'utilisateur. Il faut donc bien définir delta en pixels dans l'image
 * omni, et pas en m dans le plan du sol.
 * \warning Il faut absolument que la méthode init ait été appelée au moins une fois
 * (via le constructeur principal ou explicitement) avant d'utiliser cette méthode.
 */
cv::Mat OmniDMA::compute(const cv::Mat &mask,int delta)
{
    cv::Mat skel(mask.size(),CV_8U); // Imagee résultat (format CV_8U, pour masque binaire)
    cv::Mat D(mask.size(),cv::DataType<double>::type); // Distance Transform
    cv::Mat P(mask.size(),cv::DataType<cv::Point>::type); // Projection Transform

    // Initialisation de D et P pour chaque point du masque
    for(int i=0;i<mask.rows;i++)
    {
        for(int j=0;j<mask.cols;j++)
        {
            skel.at<uchar>(i,j)=0;
            if(mask.at<uchar>(i,j)==255)
            {
                D.at<double>(i,j)=10000*10000;
            }
            else
            {
                D.at<double>(i,j)=0;
                P.at<cv::Point>(i,j)=cv::Point(i,j);
            }
        }
    }

    // Algorithme Danielsson scan 1 : propagation des projections (donc distances)
    // de haut en bas et de gauche à droite
    // Masques considérés
    // o o o puis - - -
    // o - -      - - o
    // - - -      - - -
    for(int i=1;i<mask.rows;i++)
    {
        for(int j=1;j<mask.cols-1;j++)
        {
            if(mask.at<uchar>(i,j)==0)
            {
                D.at<double>(i,j)=0;
                P.at<cv::Point>(i,j)=cv::Point(i,j);
            }
            else
            {
                for(int k=-1;k<=0;k++)
                {
                    for(int l=-1;l<=1;l++)
                    {
                        // o o o
                        // o - -
                        // - - -
                        if(k==-1||l==-1)
                        {
                            double d=dSol(cv::Point(i,j),P.at<cv::Point>(i+k,j+l));
                            if(d<D.at<double>(i,j))
                            {
                                D.at<double>(i,j)=d;
                                P.at<cv::Point>(i,j)=P.at<cv::Point>(i+k,j+l);
                            }
                        }
                    }
                }
            }
        }

        for(int j=mask.cols-2;j>=0;j--)
        {
            // - - -
            // - - o
            // - - -
            if(mask.at<uchar>(i,j)==255)
            {
                double d=dSol(cv::Point(i,j),P.at<cv::Point>(i,j+1));
                if(d<D.at<double>(i,j))
                {
                    D.at<double>(i,j)=d;
                    P.at<cv::Point>(i,j)=P.at<cv::Point>(i,j+1);
                }
            }
        }
    }

    // Algorithme Danielsson scan 2 : propagation des projections (donc distances)
    // de bas en haut et de droite à gauche
    // Masques considérés
    // - - - puis - - -
    // - - o      o - -
    // o o o      - - -
    for(int i=mask.rows-2;i>=0;i--)
    {
        for(int j=mask.cols-2;j>=1;j--)
        {
            if(mask.at<uchar>(i,j)==255)
            {
                for(int k=1;k>=0;k--)
                {
                    for(int l=1;l>=-1;l--)
                    {
                        // - - -
                        // - - o
                        // o o o
                        if(k==1||l==1)
                        {
                            double d=dSol(cv::Point(i,j),P.at<cv::Point>(i+k,j+l));
                            if(d<D.at<double>(i,j))
                            {
                                D.at<double>(i,j)=d;
                                P.at<cv::Point>(i,j)=P.at<cv::Point>(i+k,j+l);
                            }
                        }
                    }
                }
            }
        }
        for(int j=1;j<mask.cols;j++)
        {
            if(mask.at<uchar>(i,j)==255)
            {
                // - - -
                // o - -
                // - - -
                double d=dSol(cv::Point(i,j),P.at<cv::Point>(i,j-1));
                if(d<D.at<double>(i,j))
                {
                    D.at<double>(i,j)=d;
                    P.at<cv::Point>(i,j)=P.at<cv::Point>(i,j-1);
                }
            }
        }
    }

    // Après les deux scans (ici), P et D sont connus pour chaque point de l'image
    // Avec D = 0 si le point appartient au background

    // La valeur de delta est définie à partir de la taille de la zone aveugle dans l'image
    double deltaA=dSol(cv::Point(_c.x,_c.y),cv::Point(_c.x+delta,_c.y));

    // A partir d'ici, algorithme du delta medial axis standard (cf namespace AxeMedian)
    for(int i=1;i<P.rows-1;i++)
    {
        for(int j=1;j<P.cols-1;j++)
        {
            if(D.at<double>(i,j)>0) // Si (i,j) est un point de la forme
            {
                for(int n=0;n<=1;n++) // Pour (i-1,j) et (i,j-1)
                {
                    int k=-n,l=n-1;
                    int nstep=cv::max(
                                abs(P.at<cv::Point>(i,j).x-P.at<cv::Point>(i+k,j+l).x),
                                abs(P.at<cv::Point>(i,j).y-P.at<cv::Point>(i+k,j+l).y)
                                );
                    double d=(P.at<cv::Point>(i,j).x-P.at<cv::Point>(i+k,j+l).x)*(P.at<cv::Point>(i,j).x-P.at<cv::Point>(i+k,j+l).x)+
                            (P.at<cv::Point>(i,j).y-P.at<cv::Point>(i+k,j+l).y)*(P.at<cv::Point>(i,j).y-P.at<cv::Point>(i+k,j+l).y);
                    if(d>2)
                    {
                        double step=1./nstep;
                        for(double a=step;a<=1-step;a+=step) // Pour chaque point de [P(x)P(y)]
                        {
                            double d2=D.at<double>(
                                        (int)floor(a*P.at<cv::Point>(i,j).x+(1-a)*P.at<cv::Point>(i+k,j+l).x+0.5),
                                        (int)floor(a*P.at<cv::Point>(i,j).y+(1-a)*P.at<cv::Point>(i+k,j+l).y+0.5)
                                        );
                            if(d2>deltaA)
                            {
                                if(D.at<double>(i,j)>=D.at<double>(i+k,j+l))skel.at<uchar>(i,j)=255;
                                if(D.at<double>(i,j)<=D.at<double>(i+k,j+l))skel.at<uchar>(i+k,j+l)=255;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    return skel.clone();
}

/*! \brief Calcule la carte des distance de l'espace navigable omni
 * en utilisant la métrique adaptée.
 *
 * \param mask masque définissant l'espace navigable omni.
 * \return image (de type cv::Mat<double>) définissant la distance au bord en chaque point
 *
 * \warning Il faut absolument que la méthode init ait été appelée au moins une fois
 * (via le constructeur principal ou explicitement) avant d'utiliser cette méthode.
 */
cv::Mat OmniDMA::computeOmniDT(const cv::Mat &mask)
{
    cv::Mat D(mask.size(),cv::DataType<double>::type); // Distance Transform
    cv::Mat P(mask.size(),cv::DataType<cv::Point>::type); // Projection Transform

    // Initialisation de D et P pour chaque point du masque
    for(int i=0;i<mask.rows;i++)
    {
        for(int j=0;j<mask.cols;j++)
        {
            if(mask.at<uchar>(i,j)==255)
            {
                D.at<double>(i,j)=10000*10000;
            }
            else
            {
                D.at<double>(i,j)=0;
                P.at<cv::Point>(i,j)=cv::Point(i,j);
            }
        }
    }

    // Algorithme Danielsson scan 1 : propagation des projections (donc distances)
    // de haut en bas et de gauche à droite
    // Masques considérés
    // o o o puis - - -
    // o - -      - - o
    // - - -      - - -
    for(int i=1;i<mask.rows;i++)
    {
        for(int j=1;j<mask.cols-1;j++)
        {
            if(mask.at<uchar>(i,j)==0)
            {
                D.at<double>(i,j)=0;
                P.at<cv::Point>(i,j)=cv::Point(i,j);
            }
            else
            {
                for(int k=-1;k<=0;k++)
                {
                    for(int l=-1;l<=1;l++)
                    {
                        // o o o
                        // o - -
                        // - - -
                        if(k==-1||l==-1)
                        {
                            double d=dSol(cv::Point(i,j),P.at<cv::Point>(i+k,j+l));
                            if(d<D.at<double>(i,j))
                            {
                                D.at<double>(i,j)=d;
                                P.at<cv::Point>(i,j)=P.at<cv::Point>(i+k,j+l);
                            }
                        }
                    }
                }
            }
        }

        for(int j=mask.cols-2;j>=0;j--)
        {
            // - - -
            // - - o
            // - - -
            if(mask.at<uchar>(i,j)==255)
            {
                double d=dSol(cv::Point(i,j),P.at<cv::Point>(i,j+1));
                if(d<D.at<double>(i,j))
                {
                    D.at<double>(i,j)=d;
                    P.at<cv::Point>(i,j)=P.at<cv::Point>(i,j+1);
                }
            }
        }
    }

    // Algorithme Danielsson scan 2 : propagation des projections (donc distances)
    // de bas en haut et de droite à gauche
    // Masques considérés
    // - - - puis - - -
    // - - o      o - -
    // o o o      - - -
    for(int i=mask.rows-2;i>=0;i--)
    {
        for(int j=mask.cols-2;j>=1;j--)
        {
            if(mask.at<uchar>(i,j)==255)
            {
                for(int k=1;k>=0;k--)
                {
                    for(int l=1;l>=-1;l--)
                    {
                        // - - -
                        // - - o
                        // o o o
                        if(k==1||l==1)
                        {
                            double d=dSol(cv::Point(i,j),P.at<cv::Point>(i+k,j+l));
                            if(d<D.at<double>(i,j))
                            {
                                D.at<double>(i,j)=d;
                                P.at<cv::Point>(i,j)=P.at<cv::Point>(i+k,j+l);
                            }
                        }
                    }
                }
            }
        }
        for(int j=1;j<mask.cols;j++)
        {
            if(mask.at<uchar>(i,j)==255)
            {
                // - - -
                // o - -
                // - - -
                double d=dSol(cv::Point(i,j),P.at<cv::Point>(i,j-1));
                if(d<D.at<double>(i,j))
                {
                    D.at<double>(i,j)=d;
                    P.at<cv::Point>(i,j)=P.at<cv::Point>(i,j-1);
                }
            }
        }
    }

    // Après les deux scans (ici), P et D sont connus pour chaque point de l'image
    // Avec D = 0 si le point n'appartient pas à l'espace navigable

    // P n'étant pas demandé ici, on renvoit D
    return D.clone();
}

/*! \brief mesure la distance adaptée entre deux points de l'espace navigable
 */
double OmniDMA::dSol(cv::Point p1, cv::Point p2)
{
    cv::Point2d a,b;
    a=_lut.at<cv::Point2d>(p1.x,p1.y);
    b=_lut.at<cv::Point2d>(p2.x,p2.y);

    return (double)sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}



}}
