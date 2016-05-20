#include <open_rm/SignaturesVisuelles/signaturesvisuelles.h>

namespace rm{
namespace SignaturesVisuelles{

/*! \brief Fonction qui calcule l'histogramme couleur d'une image RGB
 * et le retourne sous forme de vector.
 * \param [in] img Image considérée (doit être de type Vec3b)
 * \param [in] nBin nombre de bins de l'histogramme
 * \param [in] flou Précise s'il s'agit d'un histogramme flou
 * \returns vecteur de taille nBin contenant un Vec3f par bin
 */
std::vector<cv::Vec3f> histoCouleur(const cv::Mat &img, int nBin, bool flou)
{
    // Variable diverses
    cv::Vec3i inf,sup;

    // On initialise l'histogramme en mettant chaque bin à 0
    std::vector<cv::Vec3f> res(nBin,0);

    // On gère le cas où :
    // - l'image fournie n'est pas au bon format
    // - le nombre de bins est négatif, nul ou supérieur à 256
    // - le nombre de bins n'est pas un diviseur de 256
    if(img.type()!=CV_8UC3||nBin<=0||nBin>256||256%nBin!=0)
    {
        res.clear();
        return res;
    }

    // Si la fonction n'a pas quittée, les conditions d'exécution sont bonnes
    // On parcourt chaque pixel de l'image
    for(int i=0;i<img.rows;i++)
    {
        for(int j=0;j<img.cols;j++)
        {
            // On récupère le point
            cv::Vec3i pt=(cv::Vec3i)img.at<cv::Vec3b>(i,j);

            // Si l'histogramme flou est actif
            if(flou)
            {
                // On calcule le bin inférieur pour chaque canal
                inf=cv::Vec3i((int)floor((float)pt[0]*nBin/256),(int)floor((float)pt[1]*nBin/256),(int)floor((float)pt[2]*nBin/256));

                // Puis on le modifie en fonction de sa distance à la valeur du pixel
                res[inf[0]][0]+=std::max<float>(0,(float)1-nBin/256*pt[0]+inf[0]);
                res[inf[1]][1]+=std::max<float>(0,(float)1-nBin/256*pt[1]+inf[1]);
                res[inf[2]][2]+=std::max<float>(0,(float)1-nBin/256*pt[2]+inf[2]);

                // On calcule le bin supérieur pour chaque canal
                sup[0]=inf[0]+1;
                sup[1]=inf[1]+1;
                sup[2]=inf[2]+1;

                // Et on le modifie en fonction de sa distance à la valeur du pixel
                // En vérifiant qu'on ne dépasse pas le nombre de bins
                if(sup[0]<nBin)
                    res[sup[0]][0]+=std::max<float>(0,(float)1-(sup[0]-nBin/256*pt[0]));
                if(sup[1]<nBin)
                    res[sup[1]][1]+=std::max<float>(0,(float)1-(sup[1]-nBin/256*pt[1]));
                if(sup[0]<nBin)
                    res[sup[2]][2]+=std::max<float>(0,(float)1-(sup[2]-nBin/256*pt[2]));
            }
            // Sinon, si c'est un histogramme classique
            else
            {
                // On identifie le bon bin pour chaque canal
                inf[0]=(int)floor((float)pt[0]*nBin/256.);
                inf[1]=(int)floor((float)pt[1]*nBin/256.);
                inf[2]=(int)floor((float)pt[2]*nBin/256.);

                // Et on l'incrémente
                res[inf[0]][0]++;
                res[inf[1]][1]++;
                res[inf[2]][2]++;
            }
        }
    }

    return res;
}

/*! \brief Fonction qui calcule l'histogramme couleur normalisé d'une image RGB
 * et le retourne sous forme de vector.
 * \param [in] img Image considérée (doit être de type Vec3b)
 * \param [in] nBin nombre de bins de l'histogramme
 * \param [in] flou Précise s'il s'agit d'un histogramme flou
 * \returns vecteur de taille nBin contenant un Vec3f par bin
 */
std::vector<cv::Vec3f> histoCouleurNormalise(const cv::Mat &img, int nBin, bool flou)
{
    return normalizeHisto(histoCouleur(img,nBin,flou));
}

/*! \brief Fonction qui calcule l'histogramme couleur obtenu en appliquant le monome d'ordre n
 * passé en argument, c'est à dire la racine n-ème du produit de chaque terme.
 *
 * \param [in] img image considérée (RGB)
 * \param [in] monomes vecteur définissant le monome
 * \param [in] nBin nombre de bins de l'histogramme
 * \param [in] flou histogramme flou ou non
 * \returns vecteur de taille nBin contenant un Vec3f par bin
 */
std::vector<cv::Vec3f> monomesRGB(const cv::Mat &img, const std::vector<cv::Vec2i> monomes, int nBin, bool flou)
{
    // Variable diverses
    cv::Vec3i inf,sup;
    cv::Vec3f prod;

    // On initialise l'histogramme en mettant chaque bin à 0
    std::vector<cv::Vec3f> res(nBin,0);

    // On gère le cas où :
    // - l'image fournie n'est pas au bon format
    // - le nombre de bins est négatif, nul ou supérieur à 256
    // - le nombre de bins n'est pas un diviseur de 256
    if(img.type()!=CV_8UC3||nBin<=0||nBin>256||256%nBin!=0)
    {
        res.clear();
        return res;
    }

    // Si la fonction n'a pas quittée, les conditions d'exécution sont bonnes
    // On parcourt chaque pixel de l'image
    for(int i=0;i<img.rows;i++)
    {
        for(int j=0;j<img.cols;j++)
        {
            // On initialise la valeur du monome
            prod=cv::Vec3f(1,1,1);

            // On calcule le produit des n termes du monome pour chaque canal
            for(unsigned int k=0;k<monomes.size();k++)
            {
                prod[0]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[0];
                prod[1]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[1];
                prod[2]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[2];
            }

            // On applique la racine n-ieme
            prod[0]=std::pow(prod[0],(float)1./monomes.size());
            prod[1]=std::pow(prod[1],(float)1./monomes.size());
            prod[2]=std::pow(prod[2],(float)1./monomes.size());

            // Si l'histogramme flou est actif
            if(flou)
            {
                // On calcule le bin inférieur pour chaque canal
                inf=cv::Vec3i((int)floor((float)prod[0]*nBin/256),(int)floor((float)prod[1]*nBin/256),(int)floor((float)prod[2]*nBin/256));

                // Puis on le modifie en fonction de sa distance à la valeur du pixel
                res[inf[0]][0]+=std::max<float>(0,(float)1-nBin/256*prod[0]+inf[0]);
                res[inf[1]][1]+=std::max<float>(0,(float)1-nBin/256*prod[1]+inf[1]);
                res[inf[2]][2]+=std::max<float>(0,(float)1-nBin/256*prod[2]+inf[2]);

                // On calcule le bin supérieur pour chaque canal
                sup[0]=inf[0]+1;
                sup[1]=inf[1]+1;
                sup[2]=inf[2]+1;

                // Et on le modifie en fonction de sa distance à la valeur du pixel
                // En vérifiant qu'on ne dépasse pas le nombre de bins
                if(sup[0]<nBin)
                    res[sup[0]][0]+=std::max<float>(0,(float)1-(sup[0]-nBin/256*prod[0]));
                if(sup[1]<nBin)
                    res[sup[1]][1]+=std::max<float>(0,(float)1-(sup[1]-nBin/256*prod[1]));
                if(sup[0]<nBin)
                    res[sup[2]][2]+=std::max<float>(0,(float)1-(sup[2]-nBin/256*prod[2]));
            }
            // Sinon, si c'est un histogramme classique
            else
            {
                // On identifie le bon bin pour chaque canal
                inf[0]=(int)floor((float)prod[0]*nBin/256.);
                inf[1]=(int)floor((float)prod[1]*nBin/256.);
                inf[2]=(int)floor((float)prod[2]*nBin/256.);

                // Et on l'incrémente
                res[inf[0]][0]++;
                res[inf[1]][1]++;
                res[inf[2]][2]++;
            }
        }
    }

    return res;
}

/*! \brief Fonction qui calcule la signature basée sur les invariants de Haar et intégrants les composantes suivantes :
 * - un noyau monomial passé en argument
 * - un noyau de pertinence défini par un masque
 * - un noyau de partition défini par les fonctions triangles
 *
 * Dans cette version, le noyau de pertinence est défini par le masque (une version sans masque existe)
 * \param [in] img image considérée (doit être sphérique à cause du sin)
 * \param [in] monomes vecteur définissant le noyau monomial considéré
 * \param [in] mask masque binaire définissant le noyau de pertinence
 * \param [in] nBin nombre de bins pour le noyau de partition
 * \returns vecteur de taille nBin contenant un Vec3f par bin
 */
std::vector<cv::Vec3f> haarFmFpFt(const cv::Mat &img, const std::vector<cv::Vec2i> monomes, const cv::Mat &mask, int nBin)
{
    // Variable diverses
    cv::Vec3i inf,sup;
    cv::Vec3f prod;

    // On initialise l'histogramme en mettant chaque bin à 0
    std::vector<cv::Vec3f> res(nBin,0);

    // On gère le cas où :
    // - l'image fournie n'est pas au bon format
    // - le nombre de bins est négatif, nul ou supérieur à 256
    // - le nombre de bins n'est pas un diviseur de 256
    if(img.type()!=CV_8UC3||nBin<=0||nBin>256||256%nBin!=0)
    {
        res.clear();
        return res;
    }

    // Si la fonction n'a pas quittée, les conditions d'exécution sont bonnes
    // On parcourt chaque pixel de l'image
    for(int i=0;i<img.rows;i++)
    {
        for(int j=0;j<img.cols;j++)
        {
            // On initialise la valeur du monome
            prod=cv::Vec3f(1,1,1);

            // On calcule le produit des n termes du monome pour chaque canal
            // en vérifiant que les pixels considérés respectent le noyau de pertinence
            for(unsigned int k=0;k<monomes.size();k++)
            {
                if(mask.at<uchar>((i+monomes[k][0])%256,(j+monomes[k][1])%256)==0)
                {
                    prod=cv::Vec3f(-1,-1,-1);
                    break;
                }
                else
                {
                    prod[0]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[0];
                    prod[1]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[1];
                    prod[2]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[2];
                }
            }

            // Si le pixel est considéré par le noyau de pertinence
            // on l'ajoute à l'histogramme
            if(prod[0]!=-1)
            {
                // On applique la racine n-ieme
                prod[0]=std::pow(prod[0],(float)1./monomes.size());
                prod[1]=std::pow(prod[1],(float)1./monomes.size());
                prod[2]=std::pow(prod[2],(float)1./monomes.size());

                // On calcule le bin inférieur pour chaque canal
                inf=cv::Vec3i((int)floor((float)prod[0]*nBin/256),(int)floor((float)prod[1]*nBin/256),(int)floor((float)prod[2]*nBin/256));

                // Puis on le modifie en fonction de sa distance à la valeur du pixel
                res[inf[0]][0]+=std::max<float>(0,(float)1-nBin/256*prod[0]+inf[0]);
                res[inf[1]][1]+=std::max<float>(0,(float)1-nBin/256*prod[1]+inf[1]);
                res[inf[2]][2]+=std::max<float>(0,(float)1-nBin/256*prod[2]+inf[2]);

                // On calcule le bin supérieur pour chaque canal
                sup[0]=inf[0]+1;
                sup[1]=inf[1]+1;
                sup[2]=inf[2]+1;

                // Et on le modifie en fonction de sa distance à la valeur du pixel
                // En vérifiant qu'on ne dépasse pas le nombre de bins
                if(sup[0]<nBin)
                    res[sup[0]][0]+=std::max<float>(0,(float)1-(sup[0]-nBin/256*prod[0]));
                if(sup[1]<nBin)
                    res[sup[1]][1]+=std::max<float>(0,(float)1-(sup[1]-nBin/256*prod[1]));
                if(sup[0]<nBin)
                    res[sup[2]][2]+=std::max<float>(0,(float)1-(sup[2]-nBin/256*prod[2]));

            }
        }
    }

    return res;
}

/*! \brief Fonction qui calcule la signature basée sur les invariants de Haar et intégrants les composantes suivantes :
 * - un noyau monomial passé en argument
 * - un noyau de pertinence défini par une couleur  ( 255,0,0 par défaut )
 * - un noyau de partition défini par les fonctions triangles
 *
 * Dans cette version, le noyau de pertinence est défini par une couleur spécifique (colPert)
 * \param [in] img image considérée (doit être sphérique à cause du sin)
 * \param [in] monomes vecteur définissant le noyau monomial considéré
 * \param [in] colPert couleur définissant la couleur des pixels non pertinents
 * \param [in] nBin nombre de bins pour le noyau de partition
 * \returns vecteur de taille nBin contenant un Vec3f par bin
 */
std::vector<cv::Vec3f> haarFmFpFt(const cv::Mat &img, const std::vector<cv::Vec2i> monomes, cv::Vec3b colPert, int nBin)
{
    // Variable diverses
    cv::Vec3i inf,sup;
    cv::Vec3f prod;

    // On initialise l'histogramme en mettant chaque bin à 0
    std::vector<cv::Vec3f> res(nBin,0);

    // On gère le cas où :
    // - l'image fournie n'est pas au bon format
    // - le nombre de bins est négatif, nul ou supérieur à 256
    // - le nombre de bins n'est pas un diviseur de 256
    if(img.type()!=CV_8UC3||nBin<=0||nBin>256||256%nBin!=0)
    {
        res.clear();
        return res;
    }

    // Si la fonction n'a pas quittée, les conditions d'exécution sont bonnes
    // On parcourt chaque pixel de l'image
    for(int i=0;i<img.rows;i++)
    {
        for(int j=0;j<img.cols;j++)
        {
            // On initialise la valeur du monome
            prod=cv::Vec3f(1,1,1);

            // On calcule le produit des n termes du monome pour chaque canal
            // en vérifiant que les pixels considérés respectent le noyau de pertinence
            for(unsigned int k=0;k<monomes.size();k++)
            {
                if(img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)==colPert)
                {
                    prod=cv::Vec3f(-1,-1,-1);
                    break;
                }
                else
                {
                    prod[0]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[0];
                    prod[1]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[1];
                    prod[2]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[2];
                }
            }

            // Si le pixel est considéré par le noyau de pertinence
            // on l'ajoute à l'histogramme
            if(prod[0]!=-1)
            {
                // On applique la racine n-ieme
                prod[0]=std::pow(prod[0],(float)1./monomes.size());
                prod[1]=std::pow(prod[1],(float)1./monomes.size());
                prod[2]=std::pow(prod[2],(float)1./monomes.size());

                // On calcule le bin inférieur pour chaque canal
                inf=cv::Vec3i((int)floor((float)prod[0]*nBin/256),(int)floor((float)prod[1]*nBin/256),(int)floor((float)prod[2]*nBin/256));

                // Puis on le modifie en fonction de sa distance à la valeur du pixel
                res[inf[0]][0]+=std::max<float>(0,(float)1-nBin/256*prod[0]+inf[0]);
                res[inf[1]][1]+=std::max<float>(0,(float)1-nBin/256*prod[1]+inf[1]);
                res[inf[2]][2]+=std::max<float>(0,(float)1-nBin/256*prod[2]+inf[2]);

                // On calcule le bin supérieur pour chaque canal
                sup[0]=inf[0]+1;
                sup[1]=inf[1]+1;
                sup[2]=inf[2]+1;

                // Et on le modifie en fonction de sa distance à la valeur du pixel
                // En vérifiant qu'on ne dépasse pas le nombre de bins
                if(sup[0]<nBin)
                    res[sup[0]][0]+=std::max<float>(0,(float)1-(sup[0]-nBin/256*prod[0]));
                if(sup[1]<nBin)
                    res[sup[1]][1]+=std::max<float>(0,(float)1-(sup[1]-nBin/256*prod[1]));
                if(sup[0]<nBin)
                    res[sup[2]][2]+=std::max<float>(0,(float)1-(sup[2]-nBin/256*prod[2]));

            }
        }
    }

    return res;
}

/*! \brief Fonction qui calcule la signature basée sur les invariants de Haar et intégrants les composantes suivantes :
 * - un noyau monomial passé en argument
 * - un noyau de pertinence défini par une couleur  ( 255,0,0 par défaut )
 * - un noyau de partition défini par les fonctions triangles
 * - un noyau structurel défini par le vecteur de points 3D M
 *
 * Dans cette version, le noyau de pertinence est défini par une couleur à ignorer (colPert)
 * \param [in] img image considérée (doit être sphérique à cause du sin)
 * \param [in] monomes vecteur définissant le noyau monomial considéré
 * \param [in] colPert couleur définissant la couleur des pixels non pertinents
 * \param [in] M coordonnées (X,Y) dans le repère monde de chaque radiale
 * \param [in] nBin nombre de bins pour le noyau de partition
 * \returns vecteur de taille nBin contenant un Vec3f par bin
 */
std::vector<cv::Vec3f> haarFmFpFtFs(const cv::Mat &img, const std::vector<cv::Vec2i> monomes, std::vector<cv::Point2f> M, cv::Vec3b colPert, int nBin)
{
    // Variable diverses
    cv::Vec3i inf,sup;
    cv::Vec3f prod;
    float coef;

    // On initialise l'histogramme en mettant chaque bin à 0
    std::vector<cv::Vec3f> res(nBin,0);

    // On gère le cas où :
    // - l'image fournie n'est pas au bon format
    // - le nombre de bins est négatif, nul ou supérieur à 256
    // - le nombre de bins n'est pas un diviseur de 256
    // - la taille du vecteur M diffère de img.cols
    if(img.type()!=CV_8UC3||nBin<=0||nBin>256||256%nBin!=0||img.cols!=M.size())
    {
        res.clear();
        return res;
    }

    // Si la fonction n'a pas quittée, les conditions d'exécution sont bonnes
    // On parcourt chaque pixel de l'image
    for(int j=0;j<img.cols;j++)
    {
        // On calcule le noyau structurel
        // qui est le même pour l'ensemble de la radiale
        // n_s=0.5*(d(M(j),M(j-1))+d(M(j),M(j+1))
        if(j==0)
            coef=(float)0.5*(sqrt((M[j].x-M[img.cols-1].x)*(M[j].x-M[img.cols-1].x)+(M[j].y-M[img.cols-1].y)*(M[j].y-M[img.cols-1].y))+
                    sqrt((M[j].x-M[j+1].x)*(M[j].x-M[j+1].x)+(M[j].y-M[j+1].y)*(M[j].y-M[j+1].y)));
        else if(j==img.cols-1)
            coef=(float)0.5*(sqrt((M[j].x-M[0].x)*(M[j].x-M[0].x)+(M[j].y-M[0].y)*(M[j].y-M[0].y))+
                    sqrt((M[j].x-M[j+1].x)*(M[j].x-M[j+1].x)+(M[j].y-M[j+1].y)*(M[j].y-M[j+1].y)));
        else
            coef=(float)0.5*(sqrt((M[j].x-M[j-1].x)*(M[j].x-M[j-1].x)+(M[j].y-M[j-1].y)*(M[j].y-M[j-1].y))+
                    sqrt((M[j].x-M[j+1].x)*(M[j].x-M[j+1].x)+(M[j].y-M[j+1].y)*(M[j].y-M[j+1].y)));

        for(int i=0;i<img.rows;i++)
        {
            // On initialise la valeur du monome
            prod=cv::Vec3f(1,1,1);

            // On calcule le produit des n termes du monome pour chaque canal
            // en vérifiant que les pixels considérés respectent le noyau de pertinence
            for(unsigned int k=0;k<monomes.size();k++)
            {
                if(img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)==colPert)
                {
                    prod=cv::Vec3f(-1,-1,-1);
                    break;
                }
                else
                {
                    prod[0]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[0];
                    prod[1]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[1];
                    prod[2]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[2];
                }
            }

            // Si le pixel est considéré par le noyau de pertinence
            // on l'ajoute à l'histogramme
            if(prod[0]!=-1)
            {
                // On applique la racine n-ieme
                prod[0]=std::pow(prod[0],(float)1./monomes.size());
                prod[1]=std::pow(prod[1],(float)1./monomes.size());
                prod[2]=std::pow(prod[2],(float)1./monomes.size());

                // On calcule le bin inférieur pour chaque canal
                inf=cv::Vec3i((int)floor((float)prod[0]*nBin/256),(int)floor((float)prod[1]*nBin/256),(int)floor((float)prod[2]*nBin/256));

                // Puis on le modifie en fonction de sa distance à la valeur du pixel
                res[inf[0]][0]+=std::max<float>(0,(float)1-nBin/256*prod[0]+inf[0]);
                res[inf[1]][1]+=std::max<float>(0,(float)1-nBin/256*prod[1]+inf[1]);
                res[inf[2]][2]+=std::max<float>(0,(float)1-nBin/256*prod[2]+inf[2]);

                // On calcule le bin supérieur pour chaque canal
                sup[0]=inf[0]+1;
                sup[1]=inf[1]+1;
                sup[2]=inf[2]+1;

                // Et on le modifie en fonction de sa distance à la valeur du pixel
                // En vérifiant qu'on ne dépasse pas le nombre de bins
                if(sup[0]<nBin)
                    res[sup[0]][0]+=coef*std::max<float>(0,(float)1-(sup[0]-nBin/256*prod[0]));
                if(sup[1]<nBin)
                    res[sup[1]][1]+=coef*std::max<float>(0,(float)1-(sup[1]-nBin/256*prod[1]));
                if(sup[0]<nBin)
                    res[sup[2]][2]+=coef*std::max<float>(0,(float)1-(sup[2]-nBin/256*prod[2]));

            }
        }
    }

    return res;
}

/*! \brief Fonction qui calcule la signature basée sur les invariants de Haar et intégrants les composantes suivantes :
 * - un noyau monomial passé en argument
 * - un noyau de pertinence défini par un masque binaire
 * - un noyau de partition défini par les fonctions triangles
 * - un noyau structurel défini par le vecteur de points 3D M
 *
 * Dans cette version, le noyau de pertinence est défini par un masque binaire
 * \param [in] img image considérée (doit être sphérique à cause du sin)
 * \param [in] monomes vecteur définissant le noyau monomial considéré
 * \param [in] mask Masque binaire définissant les pixels non pertinents
 * \param [in] M coordonnées (X,Y) dans le repère monde de chaque radiale
 * \param [in] nBin nombre de bins pour le noyau de partition
 * \returns vecteur de taille nBin contenant un Vec3f par bin
 */
std::vector<cv::Vec3f> haarFmFpFtFs(const cv::Mat &img, const std::vector<cv::Vec2i> monomes, std::vector<cv::Point2f> M, const cv::Mat &mask, int nBin)
{
    // Variable diverses
    cv::Vec3i inf,sup;
    cv::Vec3f prod;
    float coef;

    // On initialise l'histogramme en mettant chaque bin à 0
    std::vector<cv::Vec3f> res(nBin,0);

    // On gère le cas où :
    // - l'image fournie n'est pas au bon format
    // - le nombre de bins est négatif, nul ou supérieur à 256
    // - le nombre de bins n'est pas un diviseur de 256
    // - la taille du vecteur M diffère de img.cols
    if(img.type()!=CV_8UC3||nBin<=0||nBin>256||256%nBin!=0||img.cols!=M.size())
    {
        res.clear();
        return res;
    }

    // Si la fonction n'a pas quittée, les conditions d'exécution sont bonnes
    // On parcourt chaque pixel de l'image
    for(int j=0;j<img.cols;j++)
    {
        // On calcule le noyau structurel
        // qui est le même pour l'ensemble de la radiale
        // n_s=0.5*(d(M(j),M(j-1))+d(M(j),M(j+1))
        if(j==0)
            coef=(float)0.5*(sqrt((M[j].x-M[img.cols-1].x)*(M[j].x-M[img.cols-1].x)+(M[j].y-M[img.cols-1].y)*(M[j].y-M[img.cols-1].y))+
                    sqrt((M[j].x-M[j+1].x)*(M[j].x-M[j+1].x)+(M[j].y-M[j+1].y)*(M[j].y-M[j+1].y)));
        else if(j==img.cols-1)
            coef=(float)0.5*(sqrt((M[j].x-M[0].x)*(M[j].x-M[0].x)+(M[j].y-M[0].y)*(M[j].y-M[0].y))+
                    sqrt((M[j].x-M[j+1].x)*(M[j].x-M[j+1].x)+(M[j].y-M[j+1].y)*(M[j].y-M[j+1].y)));
        else
            coef=(float)0.5*(sqrt((M[j].x-M[j-1].x)*(M[j].x-M[j-1].x)+(M[j].y-M[j-1].y)*(M[j].y-M[j-1].y))+
                    sqrt((M[j].x-M[j+1].x)*(M[j].x-M[j+1].x)+(M[j].y-M[j+1].y)*(M[j].y-M[j+1].y)));

        for(int i=0;i<img.rows;i++)
        {
            // On initialise la valeur du monome
            prod=cv::Vec3f(1,1,1);

            // On calcule le produit des n termes du monome pour chaque canal
            // en vérifiant que les pixels considérés respectent le noyau de pertinence
            for(unsigned int k=0;k<monomes.size();k++)
            {
                if(mask.at<uchar>((i+monomes[k][0])%256,(j+monomes[k][1])%256)==0)
                {
                    prod=cv::Vec3f(-1,-1,-1);
                    break;
                }
                else
                {
                    prod[0]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[0];
                    prod[1]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[1];
                    prod[2]*=(float)img.at<cv::Vec3b>((i+monomes[k][0])%256,(j+monomes[k][1])%256)[2];
                }
            }

            // Si le pixel est considéré par le noyau de pertinence
            // on l'ajoute à l'histogramme
            if(prod[0]!=-1)
            {
                // On applique la racine n-ieme
                prod[0]=std::pow(prod[0],(float)1./monomes.size());
                prod[1]=std::pow(prod[1],(float)1./monomes.size());
                prod[2]=std::pow(prod[2],(float)1./monomes.size());

                // On calcule le bin inférieur pour chaque canal
                inf=cv::Vec3i((int)floor((float)prod[0]*nBin/256),(int)floor((float)prod[1]*nBin/256),(int)floor((float)prod[2]*nBin/256));

                // Puis on le modifie en fonction de sa distance à la valeur du pixel
                res[inf[0]][0]+=std::max<float>(0,(float)1-nBin/256*prod[0]+inf[0]);
                res[inf[1]][1]+=std::max<float>(0,(float)1-nBin/256*prod[1]+inf[1]);
                res[inf[2]][2]+=std::max<float>(0,(float)1-nBin/256*prod[2]+inf[2]);

                // On calcule le bin supérieur pour chaque canal
                sup[0]=inf[0]+1;
                sup[1]=inf[1]+1;
                sup[2]=inf[2]+1;

                // Et on le modifie en fonction de sa distance à la valeur du pixel
                // En vérifiant qu'on ne dépasse pas le nombre de bins
                if(sup[0]<nBin)
                    res[sup[0]][0]+=coef*std::max<float>(0,(float)1-(sup[0]-nBin/256*prod[0]));
                if(sup[1]<nBin)
                    res[sup[1]][1]+=coef*std::max<float>(0,(float)1-(sup[1]-nBin/256*prod[1]));
                if(sup[0]<nBin)
                    res[sup[2]][2]+=coef*std::max<float>(0,(float)1-(sup[2]-nBin/256*prod[2]));

            }
        }
    }

    return res;
}

/*! \brief Fonction qui mesure la distance entre deux signatures de type histogramme 1D
 * en utilisant la mesure d'intersection : sum_i(min(H_1(i),H_2(i))
 *
 * \warning Les histogrammes devant être normalisés, un paramètre optionnel permet de le
 * faire en début d'algorithme. La valeur retournée correspond à un pourcentage
 * de similarité (donc entre 0 et 1)
 * \param [in] histo1,histo2 histogrammes comparés
 * \param [in] isNormalized si 0, normalisation appliquée (1 par défaut)
 * \return similarité entre les distributions en %
 */
float intersectionHisto(std::vector<float> histo1, std::vector<float> histo2, bool isNormalized)
{
    // On vérifie que les deux histogrammes ont la même taille
    if(histo1.size()!=histo2.size())
    {
        return -1;
    }

    // On commence par normaliser les histogrammes si nécessaire
    if(!isNormalized)
    {
        histo1=normalizeHisto(histo1);
        histo2=normalizeHisto(histo2);
    }

    float som=0;

    // On parcourt l'ensemble des min et on ajoute à "som"
    // le min des deux histogrammes
    for(unsigned int i=0;i<histo1.size();i++)
    {
        som+=std::min<float>(histo1[i],histo2[i]);
    }

    return som;
}


/*! \brief Fonction qui mesure la distance entre deux signatures de type histogramme RGB
 * en utilisant la mesure d'intersection : sum_i(min(H_1(i),H_2(i))
 *
 * \warning Les histogrammes devant être normalisés, un paramètre optionnel permet de le
 * faire en début d'algorithme. La valeur retournée correspond à un pourcentage
 * de similarité (donc entre 0 et 1)
 * \param [in] histo1,histo2 histogrammes comparés
 * \param [in] isNormalized si 0, normalisation appliquée (1 par défaut)
 * \return similarité entre les distributions en %
 */
float intersectionHistoCouleur(std::vector<cv::Vec3f> histo1, std::vector<cv::Vec3f> histo2, bool isNormalized)
{
    // On vérifie que les deux histogrammes ont la même taille
    if(histo1.size()!=histo2.size())
    {
        return -1;
    }

    // On commence par normaliser les histogrammes si nécessaire
    if(!isNormalized)
    {
        histo1=normalizeHisto(histo1);
        histo2=normalizeHisto(histo2);
    }

    cv::Vec3f som=cv::Vec3f(0,0,0);

    // On parcourt l'ensemble des min et on ajoute à "som"
    // le min des deux histogrammes (pour chaque canal)
    for(unsigned int i=0;i<histo1.size();i++)
    {
        som[0]+=std::min<float>(histo1[i][0],histo2[i][0]);
        som[1]+=std::min<float>(histo1[i][1],histo2[i][1]);
        som[2]+=std::min<float>(histo1[i][2],histo2[i][2]);
    }

    return (som[0]+som[1]+som[2])/3;
}

/*! \brief Fonction qui normalise un histogramme 1D passé en argument
 * \param [in] histo histogramme à normaliser
 * \return histogramme normalisé
 */
std::vector<float> normalizeHisto(const std::vector<float> &histo)
{
    std::vector<float> res=histo;
    float som=0;

    // On calcule la somme totale des bins
    for(unsigned int i=0;i<res.size();i++)
    {
        som+=res[i];
    }

    // On divise chaque bin par la somme totale
    for(unsigned int i=0;i<res.size();i++)
    {
        res[i]/=som;
    }

    // On retourne le résultat
    return res;
}

/*! \brief Fonction qui normalise un histogramme RGB passé en argument
 * \param [in] histo histogramme à normaliser
 * \return histogramme normalisé
 */
std::vector<cv::Vec3f> normalizeHisto(const std::vector<cv::Vec3f> &histo)
{
    // On commence par calculer l'histogramme couleur non normalisé
    std::vector<cv::Vec3f> res=histo;

    // Puis on le normalise
    cv::Vec3f som=cv::Vec3f(0,0,0);

    // Pour chaque canal, on calcule la somme des bins
    for(unsigned int i=0;i<res.size();i++)
    {
        som[0]+=res[i][0];
        som[1]+=res[i][1];
        som[2]+=res[i][2];
    }


    // Avant de diviser chaque bin par la valeur trouvée
    for(unsigned int i=0;i<res.size();i++)
    {
        if(som[0]!=0)
            res[i][0]/=som[0];
        if(som[1]!=0)
            res[i][1]/=som[1];
        if(som[2]!=0)
            res[i][2]/=som[2];
    }

    // Enfin, on renvoit le résultat
    return res;
}

/*! \brief Fonction qui génère une image présentant les 3 histogrammes RGB
 * les uns en dessous des autres.
 * \param [in] histo histogramme couleur considéré (normalisé ou non)
 * \param [in] vMax valeur max affichée pour un bin (en % du nbre de pixels total)
 * \return représentation visuelle de l'histogramme
 */
cv::Mat printHistoCouleur(const std::vector<cv::Vec3f> &histo, float vMax)
{

    // Variables diverses
    int h; // Hauteur d'un bin

    // On déclare l'image construite
    cv::Mat res(304,258,CV_8UC3,cv::Scalar(255,255,255));

    // On normalise l'histogramme pour simplifier les calculs
    std::vector<cv::Vec3f> histNorm=normalizeHisto(histo);

    // On "dessine" sur l'image pour séparer les histogrammes
    // Chaque histogramme occupe un rectangle de 100*256
    // auquel s'ajoute 1pixel de bordure, dont un coté
    // partagé par deux histogrammes adjacents.
    // d'où la résolution de 304*258
    cv::rectangle(res,cv::Point(0,0),cv::Point(257,101),cv::Scalar(0,0,0));
    cv::rectangle(res,cv::Point(0,101),cv::Point(257,202),cv::Scalar(0,0,0));
    cv::rectangle(res,cv::Point(0,202),cv::Point(257,303),cv::Scalar(0,0,0));

    // La largeur de l'image étant 256, la largeur de chaque bin dépend du nombre total
    int lBin=256/histo.size();

    // Pour chaque canal, la (les) ligne(s) correspondant à chaque bin sont coloriées à hauteur
    // du ratio entre la valeur normalisée et vMax
    for(unsigned int i=0;i<histNorm.size();i++)
    {
        // Canal R (rectangle supérieur)
        h=std::min(100,(int)floor(100*histNorm[i][2]/vMax));
        cv::rectangle(res,cv::Point(i*lBin+1,100-h),cv::Point(i*lBin+lBin,100),cv::Scalar(0,0,255),CV_FILLED);

        // Canal G (rectangle intermédiaire)
        h=std::min(100,(int)floor(100*histNorm[i][1]/vMax));
        cv::rectangle(res,cv::Point(i*lBin+1,201-h),cv::Point(i*lBin+lBin,201),cv::Scalar(0,255,0),CV_FILLED);

        // Canal B (rectangle intermédiaire)
        h=std::min(100,(int)floor(100*histNorm[i][0]/vMax));
        cv::rectangle(res,cv::Point(i*lBin+1,302-h),cv::Point(i*lBin+lBin,302),cv::Scalar(255,0,0),CV_FILLED);
    }

    return res.clone();
}


}
}
