#include <open_rm/TransfDistance/transfdistance.h>

namespace rm{
namespace TransfDistance{
/*! \brief Calcule la distance au bord et la projection
 * de chaque point du masque fourni en entrée
 *
 * Les points noirs désignent le background, les points blancs
 * sont la forme
 *
 * \param[in] mask Masque binaire fourni en entrée
 * \param[out] D distance au bord pour chaque point (double)
 * \param[out] P Projection pour chaque point (cv::Point)
 */
void DPHesselink(const cv::Mat& mask, cv::Mat &D, cv::Mat &P)
{
    // Initialisation des distances au bord et projection à 0
    P=cv::Mat::zeros(mask.size(),cv::DataType<cv::Point>::type);
    D=cv::Mat::zeros(mask.size(),cv::DataType<double>::type);
    cv::Mat P_1D(mask.size(),cv::DataType<cv::Point>::type);

    // La largeur de mask ne doit pas dépasser 10000
    int g[10000],s[10000],t[10000];

    // Premier scan, on considère chaque colonne comme un signal 1D
    // Et on calcule la distance au bord et la projection correspondante
    // cf. [Hesselink08]
    for(int j=0;j<mask.cols;j++)
    {
        if(mask.at<uchar>(mask.rows-1,j)==0) // Si le point est noir, il appartient au background
            g[mask.rows-1]=0; // sa distance au bord est donc nulle
        else
            g[mask.rows-1]=mask.rows*3; // sinon saa distance au bord est infinie

        for(int i=mask.rows-2;i>=0;i--) // On propage la distance au bord
        {
            if(mask.at<uchar>(i,j)==0)
                g[i]=0;
            else
                g[i]=1+g[i+1];
        }

        // Enfin, à partir de la distance au bord, on déduit la projection
        P_1D.at<cv::Point>(0,j).x=g[0];
        for(int i=1;i<mask.rows;i++)
        {
            if(i-P_1D.at<cv::Point>(i-1,j).x<=g[i])
                P_1D.at<cv::Point>(i,j).x=P_1D.at<cv::Point>(i-1,j).x;
            else
                P_1D.at<cv::Point>(i,j).x=i+g[i];

        }
    }

    // Deuxieme scan, on parcourt chaque ligne pour trouver la distance au bord
    // et la projection. cf [Hesselink08] pour les détails
    int q;
    int w;
    for(int i=0;i<mask.rows;i++)
    {
        q=0;
        s[0]=0;
        t[0]=0;
        for(int j=1;j<mask.cols;j++)
        {
            while(q>=0&&((t[q]-s[q])*(t[q]-s[q])+(i-P_1D.at<cv::Point>(i,s[q]).x)*(i-P_1D.at<cv::Point>(i,s[q]).x))>((t[q]-j)*(t[q]-j)+(i-P_1D.at<cv::Point>(i,j).x)*(i-P_1D.at<cv::Point>(i,j).x)))
                q--;
            if(q<0)
            {
                q=0;
                s[0]=j;
            }
            else
            {
                w=1+(j*j-s[q]*s[q]+(i-P_1D.at<cv::Point>(i,j).x)*(i-P_1D.at<cv::Point>(i,j).x)-(i-P_1D.at<cv::Point>(i,s[q]).x)*(i-P_1D.at<cv::Point>(i,s[q]).x))/(2*(j-s[q]));
                if(w<mask.cols)
                {
                    q++;
                    s[q]=j;
                    t[q]=w;
                }
            }
        }
        for(int j=mask.cols-1;j>=0;j--)
        {
            P.at<cv::Point>(i,j).x=P_1D.at<cv::Point>(i,s[q]).x;
            P.at<cv::Point>(i,j).y=s[q];
            if(j==t[q])q--;
        }
    }

    // Une fois la projection connue, la distance au bord est facilement déductible
    for(int i=0;i<P.rows;i++)
    {
        for(int j=0;j<P.cols;j++)
        {
            D.at<double>(i,j)=sqrt(
                        (double)(i-P.at<cv::Point>(i,j).x)*(i-P.at<cv::Point>(i,j).x)+
                        (double)(j-P.at<cv::Point>(i,j).y)*(j-P.at<cv::Point>(i,j).y)
                        );
        }
    }
}



/*! \brief Fonction qui calcule la distance au bord et la projection
 * dans le cas d'une forme définie par un masque 3d
 * \param [in] mask masque binaire ou 3états fourni en entrée
 * \param [out] D matrice de sortie pour la carte des distances
 * \param [out] P matrice de sortie pour la carte des projections
 * \param [in] d1,d2,d3 = dimensions du masque 3d
 */
void DPHesselink3d(const cv::Mat &mask, cv::Mat &D, cv::Mat &P, int d1, int d2, int d3)
{
    int sizes[] = { d1, d2, d3 };

    int g[10000];
    int s[10000];
    int t[10000];
    int q;
    int w;

    cv::Mat ft1(3,sizes,cv::DataType<cv::Point3i>::type);
    cv::Mat ft2(3,sizes,cv::DataType<cv::Point3i>::type);
    cv::Mat ft3(3,sizes,cv::DataType<cv::Point3i>::type);
    P.create(3,sizes,cv::DataType<cv::Point3i>::type);
    D.create(3,sizes,cv::DataType<double>::type);

    // Algo 1 [Hesselink08]
    for(int j=0;j<d2;j++)
    {
        for(int k=0;k<d3;k++)
        {

            // Scan 1.1
            if(mask.at<uchar>(d1-1,j,k)==255)
                g[d1-1]=0;
            else
                g[d1-1]=d1*d1;

            for(int i=d1-2;i>=0;i--)
            {
                if(mask.at<uchar>(i,j,k)==255)
                    g[i]=0;
                else
                    g[i]=1+g[i+1];
            }

            // Scan 1.2
            ft1.at<cv::Point3i>(0,j,k).x=g[0];
            for(int i=1;i<d1;i++)
            {
                if(i-ft1.at<cv::Point3i>(i-1,j,k).x<=g[i])
                    ft1.at<cv::Point3i>(i,j,k).x=ft1.at<cv::Point3i>(i-1,j,k).x;
                else
                    ft1.at<cv::Point3i>(i,j,k).x=i+g[i];
            }

        }
    }
    // Algo 2 [Hesselink08]
    for(int i=0;i<d1;i++)
    {
        for(int k=0;k<d3;k++)
        {
            q=0; s[0]=0; t[0]=0;

            // scan 2.1
            for(int j=1;j<d2;j++)
            {
                while(q>=0&&(t[q]-s[q])*(t[q]-s[q])+(i-ft1.at<cv::Point3i>(i,s[q],k).x)*(i-ft1.at<cv::Point3i>(i,s[q],k).x)>((t[q]-j)*(t[q]-j)+(i-ft1.at<cv::Point3i>(i,j,k).x)*(i-ft1.at<cv::Point3i>(i,j,k).x)))
                    q--;
                if(q<0)
                {
                    q=0;s[0]=j;
                }
                else
                {
                    w=1+(j*j-s[q]*s[q]+(i-ft1.at<cv::Point3i>(i,j,k).x)*(i-ft1.at<cv::Point3i>(i,j,k).x)-(i-ft1.at<cv::Point3i>(i,s[q],k).x)*(i-ft1.at<cv::Point3i>(i,s[q],k).x))/(2*(j-s[q]));
                    if(w<d2)
                    {
                        q++; s[q]=j; t[q]=w;
                    }
                }
            }

            // scan 2.2
            for(int j=d2-1;j>=0;j--)
            {
                ft2.at<cv::Point3i>(i,j,k).x=ft1.at<cv::Point3i>(i,s[q],k).x;
                ft2.at<cv::Point3i>(i,j,k).y=s[q];
                if(j==t[q])
                    q--;
            }
        }
    }

    // Algo 3 [Hesselink08]
    for(int i=0;i<d1;i++)
    {
        for(int j=0;j<d2;j++)
        {
            q=0; s[0]=0; t[0]=0;

            // scan 3.1
            for(int k=1;k<d3;k++)
            {
                while(q>=0&&
                      (t[q]-s[q])*(t[q]-s[q])+(i-ft2.at<cv::Point3i>(i,j,s[q]).x)*(i-ft2.at<cv::Point3i>(i,j,s[q]).x)+
                      (j-ft2.at<cv::Point3i>(i,j,s[q]).y)*(j-ft2.at<cv::Point3i>(i,j,s[q]).y)
                      >
                      (t[q]-k)*(t[q]-k)+(i-ft2.at<cv::Point3i>(i,j,k).x)*(i-ft2.at<cv::Point3i>(i,j,k).x)+
                      (j-ft2.at<cv::Point3i>(i,j,k).y)*(j-ft2.at<cv::Point3i>(i,j,k).y))
                {
                    q--;
                }

                if(q<0)
                {
                    q=0;s[0]=k;
                }
                else
                {
                    w=1+
                            (k*k-s[q]*s[q]+
                             (i-ft2.at<cv::Point3i>(i,j,k).x)*(i-ft2.at<cv::Point3i>(i,j,k).x)+(j-ft2.at<cv::Point3i>(i,j,k).y)*(j-ft2.at<cv::Point3i>(i,j,k).y)
                             -
                             (i-ft2.at<cv::Point3i>(i,j,s[q]).x)*(i-ft2.at<cv::Point3i>(i,j,s[q]).x)-(j-ft2.at<cv::Point3i>(i,j,s[q]).y)*(j-ft2.at<cv::Point3i>(i,j,s[q]).y))
                            /(2*(k-s[q]));
                    if(w<d3)
                    {
                        q++; s[q]=k; t[q]=w;
                    }
                }
            }

            // scan 3.2
            for(int k=d3;k>=0;k--)
            {
                ft3.at<cv::Point3i>(i,j,k).x=ft2.at<cv::Point3i>(i,j,s[q]).x;
                ft3.at<cv::Point3i>(i,j,k).y=ft2.at<cv::Point3i>(i,j,s[q]).y;
                ft3.at<cv::Point3i>(i,j,k).z=s[q];
                if(k==t[q])
                    q--;
            }
        }
    }

    // Déduction de P et D
    for(int i=0;i<d1;i++)
    {
        for(int j=0;j<d2;j++)
        {
            for(int k=0;k<d3;k++)
            {
                P.at<cv::Point3i>(i,j,k).x=ft3.at<cv::Point3i>(i,j,k).x;
                P.at<cv::Point3i>(i,j,k).y=ft3.at<cv::Point3i>(i,j,k).y;
                P.at<cv::Point3i>(i,j,k).z=ft3.at<cv::Point3i>(i,j,k).z;

                D.at<double>(i,j,k)=sqrt(
                            (double)(i-P.at<cv::Point3i>(i,j,k).x)*(i-P.at<cv::Point3i>(i,j,k).x)+
                            (double)(j-P.at<cv::Point3i>(i,j,k).y)*(j-P.at<cv::Point3i>(i,j,k).y)+
                            (double)(k-P.at<cv::Point3i>(i,j,k).z)*(k-P.at<cv::Point3i>(i,j,k).z)
                            );

            }
        }
    }

}


}

}
