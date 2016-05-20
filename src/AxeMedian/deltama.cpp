#include <open_rm/AxeMedian/deltama.h>

namespace rm{
namespace AxeMedian{

/*! \brief constructeur par défaut (inutilisé pour l'instant)
 */
DeltaMA::DeltaMA()
{

}

/*! \brief Destructeur par défaut (inutilisé pour l'instant)
 */
DeltaMA::~DeltaMA()
{

}

/*! \brief Calcule l'axe médian d'une forme binaire à partir
 * de l'algorithme du DMA.
 * \param[in] mask masque binaire (255=forme) définissant la forme considérée
 * \param[in] delta valeur du paramètre delta (voir article FDMA pour plus d'infos)
 * \return masque binaire définissant le squelette obtenu (255=squelette)
 */
cv::Mat DeltaMA::ma(const cv::Mat &mask, double delta)
{
    cv::Mat D,P;
    cv::Mat ma=cv::Mat::zeros(mask.size(),cv::DataType<uchar>::type);
    rm::TransfDistance::DPHesselink(mask,D,P);
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
                    if(d>4*delta*delta)
                    {
                        double step=1./(100*nstep);
                        for(double a=step;a<=1-step;a+=step) // Pour chaque point de [P(x)P(y)]
                        {
                            cv::Point2f m=cv::Point2f(a*P.at<cv::Point>(i,j).x+(1-a)*P.at<cv::Point>(i+k,j+l).x,
                                                      a*P.at<cv::Point>(i,j).y+(1-a)*P.at<cv::Point>(i+k,j+l).y);
                            cv::Point p1=P.at<cv::Point>((int)floor(a*P.at<cv::Point>(i,j).x+(1-a)*P.at<cv::Point>(i+k,j+l).x),
                                                         (int)floor(a*P.at<cv::Point>(i,j).y+(1-a)*P.at<cv::Point>(i+k,j+l).y));
                            cv::Point p2=P.at<cv::Point>((int)floor(a*P.at<cv::Point>(i,j).x+(1-a)*P.at<cv::Point>(i+k,j+l).x+1),
                                                         (int)floor(a*P.at<cv::Point>(i,j).y+(1-a)*P.at<cv::Point>(i+k,j+l).y));
                            cv::Point p3=P.at<cv::Point>((int)floor(a*P.at<cv::Point>(i,j).x+(1-a)*P.at<cv::Point>(i+k,j+l).x),
                                                         (int)floor(a*P.at<cv::Point>(i,j).y+(1-a)*P.at<cv::Point>(i+k,j+l).y+1));
                            cv::Point p4=P.at<cv::Point>((int)floor(a*P.at<cv::Point>(i,j).x+(1-a)*P.at<cv::Point>(i+k,j+l).x+1),
                                                         (int)floor(a*P.at<cv::Point>(i,j).y+(1-a)*P.at<cv::Point>(i+k,j+l).y+1));

                            double d1=(double)(m.x-p1.x)*(m.x-p1.x)+(double)(m.y-p1.y)*(m.y-p1.y);
                            double d2=(double)(m.x-p2.x)*(m.x-p2.x)+(double)(m.y-p2.y)*(m.y-p2.y);
                            double d3=(double)(m.x-p3.x)*(m.x-p3.x)+(double)(m.y-p3.y)*(m.y-p3.y);
                            double d4=(double)(m.x-p4.x)*(m.x-p4.x)+(double)(m.y-p4.y)*(m.y-p4.y);

                            double mind=std::min(d1,std::min(d2,std::min(d3,d4)));

                            if(mind>delta*delta)
                            {
                                if(D.at<double>(i,j)>=D.at<double>(i+k,j+l))ma.at<uchar>(i,j)=255;
                                if(D.at<double>(i,j)<=D.at<double>(i+k,j+l))ma.at<uchar>(i+k,j+l)=255;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    return ma.clone();
}

/*! \brief Calcule la transformée en axe médian d'une forme binaire à partir
 * de l'algorithme du DMA.
 * \param[in] mask masque binaire (255=forme) définissant la forme considérée
 * \param[in] delta valeur du paramètre delta (voir article DMA pour plus d'infos)
 * \return matrice (de doubles) définissant le squelette obtenu. A chaque point du squelette est
 * associée sa distance au bord.
 */
cv::Mat DeltaMA::mat(const cv::Mat &mask, double delta)
{
    cv::Mat D,P;
    cv::Mat mat=cv::Mat::zeros(mask.size(),cv::DataType<double>::type);
    rm::TransfDistance::DPHesselink(mask,D,P);

    double d;
    for(int i=1;i<P.rows-1;i++)
    {
        for(int j=1;j<P.cols-1;j++)
        {
            if(D.at<double>(i,j)>0)
            {
                for(int n=0;n<=1;n++)
                {
                    int k=-n,l=n-1;
                    int nstep=cv::max(
                                abs(P.at<cv::Point>(i,j).x-P.at<cv::Point>(i+k,j+l).x),
                                abs(P.at<cv::Point>(i,j).y-P.at<cv::Point>(i+k,j+l).y)
                                );
                    d=sqrt(
                                (double)(P.at<cv::Point>(i,j).x-P.at<cv::Point>(i+k,j+l).x)*(P.at<cv::Point>(i,j).x-P.at<cv::Point>(i+k,j+l).x)+
                                (double)(P.at<cv::Point>(i,j).y-P.at<cv::Point>(i+k,j+l).y)*(P.at<cv::Point>(i,j).y-P.at<cv::Point>(i+k,j+l).y)
                                );
                    if(d>=2*delta)
                    {

                        double step=1./nstep;
                        for(double a=step;a<=1-step;a+=step) // Pour chaque point de [P(x)P(y)]
                        {
                            cv::Point2f m=cv::Point2f(a*P.at<cv::Point>(i,j).x+(1-a)*P.at<cv::Point>(i+k,j+l).x,
                                                      a*P.at<cv::Point>(i,j).y+(1-a)*P.at<cv::Point>(i+k,j+l).y);
                            cv::Point p1=P.at<cv::Point>((int)floor(a*P.at<cv::Point>(i,j).x+(1-a)*P.at<cv::Point>(i+k,j+l).x),
                                                         (int)floor(a*P.at<cv::Point>(i,j).y+(1-a)*P.at<cv::Point>(i+k,j+l).y));
                            cv::Point p2=P.at<cv::Point>((int)floor(a*P.at<cv::Point>(i,j).x+(1-a)*P.at<cv::Point>(i+k,j+l).x+1),
                                                         (int)floor(a*P.at<cv::Point>(i,j).y+(1-a)*P.at<cv::Point>(i+k,j+l).y));
                            cv::Point p3=P.at<cv::Point>((int)floor(a*P.at<cv::Point>(i,j).x+(1-a)*P.at<cv::Point>(i+k,j+l).x),
                                                         (int)floor(a*P.at<cv::Point>(i,j).y+(1-a)*P.at<cv::Point>(i+k,j+l).y+1));
                            cv::Point p4=P.at<cv::Point>((int)floor(a*P.at<cv::Point>(i,j).x+(1-a)*P.at<cv::Point>(i+k,j+l).x+1),
                                                         (int)floor(a*P.at<cv::Point>(i,j).y+(1-a)*P.at<cv::Point>(i+k,j+l).y+1));

                            double d1=(double)(m.x-p1.x)*(m.x-p1.x)+(double)(m.y-p1.y)*(m.y-p1.y);
                            double d2=(double)(m.x-p2.x)*(m.x-p2.x)+(double)(m.y-p2.y)*(m.y-p2.y);
                            double d3=(double)(m.x-p3.x)*(m.x-p3.x)+(double)(m.y-p3.y)*(m.y-p3.y);
                            double d4=(double)(m.x-p4.x)*(m.x-p4.x)+(double)(m.y-p4.y)*(m.y-p4.y);

                            double mind=std::min(d1,std::min(d2,std::min(d3,d4)));

                            if(mind>delta*delta)
                            {
                                if(D.at<double>(i,j)>=D.at<double>(i+k,j+l))mat.at<double>(i,j)=D.at<double>(i,j);
                                if(D.at<double>(i,j)<=D.at<double>(i+k,j+l))mat.at<double>(i+k,j+l)=D.at<double>(i+k,j+l);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    return mat.clone();
}

}}

