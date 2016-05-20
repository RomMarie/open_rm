#include <open_rm/AxeMedian/operations.h>

namespace rm{
namespace AxeMedian{
namespace Operations{

/*! \brief Calcule le degré topologique d'un point
 *
 * Le dégré topologique correspond au nombre de direction
 * sur le squelette disponible depuis la position pos,
 * relativement à un rayon r donné
 * \param[in] mask Masque binaire définissant le squelette considéré
 * \param[in] pos Position du point considéré
 * \param[in] r rayon du voisinage de détection
 * \return degré topologique du point
 * \warning Il y a un bug dans le cas où la courbure du squelette
 * épouse le cercle de rayon r centré sur pos
 */
int topoDegree(const cv::Mat &mask, cv::Point pos, int r)
{
    std::vector<cv::Point> branchPoints;
    for(int k=-r;k<=r;k++)
    {
        for(int l=-r;l<=r;l++)
        {
            // Si la distance est correcte et qu'il s'agit d'un point du squelette
            if(k*k+l*l<(r+sqrt(2.))*(r+sqrt(2.))&&k*k+l*l>=r*r&&
                    pos.x+k>0&&pos.x+k<mask.rows&&pos.y+l>0&&pos.y+l<mask.cols&&
                    mask.at<uchar>(pos.x+k,pos.y+l)==255)
            {
                bool flag=1;
                // On vérifie que les points déja retenus ne sont pas adjacents
                // (pour éviter qu'une meme branche soit représentée par plusieurs points
                for(unsigned int m=0;m<branchPoints.size();m++)
                {
                    if((float)(pos.x+k-branchPoints[m].x)*(pos.x+k-branchPoints[m].x)+
                            (pos.y+l-branchPoints[m].y)*(pos.y+l-branchPoints[m].y)<=4)flag=0;
                }

                // Si tel n'est pas le cas, on ajoute le point courant au vecteur
                if(flag)
                {
                    branchPoints.push_back(cv::Point(pos.x+k,pos.y+l));
                }
            }
        }
    }

    return branchPoints.size();
}

}
}
}
