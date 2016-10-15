#include <open_rm/Blobs/blobs.h>

#include <iostream>
namespace rm{
namespace Blobs{

/*! \brief Recherche de tous les K-blobs dans une image binaire
 * \param[in] mask Masque binaire où blanc (255) est l'objet et noir (0) le background
 * \param[in] k taille du voisinage considéré pour la composante connexe
 * \return Vecteur contenant une liste des blobs (définis en tant que vecteurs de points)
 *
 * Par exemple, pour allKBlobs(mask,sqrt(2)) retourne la liste des composantes 8-connexes
 *
 * \warning le type de mask doit être cv::Mat<uchar> (CV_8U)
 */
std::vector<std::vector<cv::Point> > allKBlobs(const cv::Mat &mask, float k)
{

    std::vector<std::vector<cv::Point> > blobs; // liste final des blobs détectés
    cv::Mat tmp=mask.clone(); // tmp sera modifié pendant la construction des blobs, d'où le clone
    std::vector<cv::Point> list; // liste des points d'un blob pendant sa construction

    // pour chaque point du masque
    for(int i=0;i<mask.rows;i++)
    {
        for(int j=0;j<mask.cols;j++)
        {
            // si le point n'appartient pas au background dans tmp
            if(tmp.at<uchar>(i,j)>0)
            {
                std::vector<cv::Point> blob; // on initialise un blob
                blob.push_back(cv::Point(i,j)); // auquel on ajoute le point (i,j)
                list.clear();
                list.push_back(cv::Point(i,j)); // list va se vider au fur et à mesure, pas blob
                tmp.at<uchar>(i,j)=0; // on modifie tmp pour ne pas reconsidérer le point (i,j)

                while(list.size()>0) // tant que tous les points connus du blob n'ont pas été étudiés
                {
                    cv::Point p=list[list.size()-1]; // On en charge un dans la variable p
                    list.pop_back(); // On le retire de list

                    // Puis on regarde son k-voisinage
                    for(int v=-(int)floor(k);v<=(int)floor(k);v++)
                    {
                        for(int l=-(int)floor(k);l<=(int)floor(k);l++)
                        {

                            // Si le point (p.x+v,p.y+l) appartient au k-voisinage de p,
                            // est dans l'image,
                            // et n'appartient pas au background
                            if((float)sqrt((double)v*v+l*l)<=k&&
                                    p.x+v>=0&&p.x+v<mask.rows&&
                                    p.y+l>=0&&p.y+l<mask.cols&&
                                    tmp.at<uchar>(p.x+v,p.y+l)>0)
                            {
                                // alors :
                                // - on l'ajoute à list
                                // - on modifie tmp pour ne plus en tenir compte
                                // - et on l'ajoute au blob en cours
                                list.push_back(cv::Point(p.x+v,p.y+l));
                                tmp.at<uchar>(p.x+v,p.y+l)=0;
                                blob.push_back(cv::Point(p.x+v,p.y+l));
                            }
                        }
                    }
                }

                // Lorsque list est vide, blob est complet.
                // On l'ajoute donc à blobs
                blobs.push_back(blob);
            }
        }
    }

    // Ici, toute l'image à été parcourue, donc tous les blobs détectés
    // On renvoie le vecteur blobs qui les contient.
    return blobs;
}

/*! \brief Recherche des n plus gros K-blobs dans une image binaire
 * \param[in] mask Masque binaire où blanc (255) est l'objet et noir (0) le background
 * \param[in] n nombre de blobs recherchés
 * \param[in] k taille du voisinage considéré pour la composante connexe
 * \return Vecteur contenant les n plus gros blobs de mask (définis en tant que vecteurs de points)
 * \warning le type de mask doit être cv::Mat<uchar> (CV_8U)
 */
std::vector<std::vector<cv::Point> > nLargestKBlobs(const cv::Mat &mask, int n, float k)
{
    // On commence par identifier tous les k-composantes connexes
    // et on les stocke dans blobs
    std::vector<std::vector<cv::Point> > blobs=allKBlobs(mask,k);

    // On sélectionne ensuite les n plus longues
    std::vector<std::vector<cv::Point> > longest(n); // On commence par définir un vecteur de la bonne taille

    // Puis tri par insertion
    // A noter que tous les éléments de "longest" sont vides,
    // d'où longest[j].size()==0 à l'état initial
    // /!\ optimisable
    for(unsigned int i=0;i<blobs.size();i++)
    {
        for(int j=0;j<n;j++)
        {
            if(longest[j].size()<blobs[i].size())
            {
                for(int k=n-1;k>j;k--)
                {
                    longest[k]=longest[k-1];
                }
                longest[j]=blobs[i];
                break;
            }
        }
    }
    return longest;
}

/*! \brief Recherche de la composante k-connexe auquel appartient le point s
 * \param[in] mask masque binaire où blanc (255) est l'objet et noir (0) le background
 * \param[in] s seed utilisé pour identifier le blob recherché
 * \param[in] k taille du voisinage considéré pour la composante connexe
 * \return Vecteur de points définissant le k-blob auquel appartient s
 * \warning le type de mask doit être cv::Mat<uchar> (CV_8U)
 */
std::vector<cv::Point> seedKBlob(const cv::Mat &mask, cv::Point s, float k)
{
    cv::Mat tmp=mask.clone(); // tmp sera modifié, d'où le clone
    std::vector<cv::Point> blob; // vecteur de points dans lequel va être stocké le blob
    blob.push_back(cv::Point(s.x,s.y)); // On y ajoute le seed s
    std::vector<cv::Point> list; // list temporaire des points à étudier
    list.push_back(cv::Point(s.x,s.y)); // On y ajoute le seed s aussi
    tmp.at<uchar>(s.x,s.y)=0; // On supprime s de tmp pour ne l'étudier qu'une fois

    // tant qu'il y a des points à étudier
    while(list.size()>0)
    {
        cv::Point p=list[list.size()-1]; // On en charge un
        list.pop_back(); // puis on le retire de la liste

        // Pour chaque point de son k-voisinage
        for(int v=-(int)floor(k);v<=(int)floor(k);v++)
        {
            for(int l=-(int)floor(k);l<=(int)floor(k);l++)
            {

                // Si le point (p.x+v,p.y+l) appartient au k-voisinage de p,
                // est dans l'image,
                // et n'appartient pas au background
                if((float)sqrt((double)v*v+l*l)<=k&&
                        p.x+v>=0&&p.x+v<mask.rows&&
                        p.y+l>=0&&p.y+l<mask.cols&&
                        tmp.at<uchar>(p.x+v,p.y+l)>0)
                {
                    // alors :
                    // - on l'ajoute à list
                    // - on modifie tmp pour ne plus en tenir compte
                    // - et on l'ajoute au blob
                    list.push_back(cv::Point(p.x+v,p.y+l));
                    tmp.at<uchar>(p.x+v,p.y+l)=0;
                    blob.push_back(cv::Point(p.x+v,p.y+l));
                }
            }
        }
    }

    // A ce stade, tous les points du blob sont connus
    // on le retourne donc
    return blob;
}

/*! \brief  Recherche du plus gros k-blob dans une image binaire
 * \param[in] mask masque binaire où blanc (255) est l'objet et noir (0) le background
 * \param[in] k taille du voisinage considéré pour la composante connexe
 * \return Vecteur de points définissant le plus gros k-blob
 * \warning le type de mask doit être cv::Mat<uchar> (CV_8U)
 */
std::vector<cv::Point> largestKBlob(const cv::Mat &mask, float k)
{
    // On appelle la fonction nLargestKBlobs avec n=1,
    // et on retourne le résultat obtenu
    return nLargestKBlobs(mask,1,k)[0];
}

/*! \brief Recherche de tous les blobs 4-connexes (voisinage courant)
 * \param[in] mask masque binaire où blanc (255) est l'objet et noir (0) le background
 * \return Vecteur contenant tous les 4-blobs de mask (définis par des vecteurs de points)
 * \warning le type de mask doit être cv::Mat<uchar> (CV_8U)
 */
std::vector<std::vector<cv::Point> > allBlob4Connexe(const cv::Mat &mask)
{
    // On appelle la fonction allKBlobs avec k=1
    // et on retourne le résultat obtenu
    return allKBlobs(mask,1.);
}

/*! \brief Recherche de tous les blobs 8-connexes (voisinage courant)
 * \param[in] mask masque binaire où blanc (255) est l'objet et noir (0) le background
 * \return Vecteur contenant tous les 8-blobs de mask (définis par des vecteurs de points)
 * \warning le type de mask doit être cv::Mat<uchar> (CV_8U)
 */
std::vector<std::vector<cv::Point> > allBlob8Connexe(const cv::Mat &mask)
{
    // On appelle la fonction AllKBlobs avec k=sqrt(2)
    // et on retourne le résultat obtenu
    return allKBlobs(mask,(float)sqrt(2.));
}

/*! \brief Recherche des n plus gros blobs 4-connexes
 * \param[in] mask masque binaire où blanc (255) est l'objet et noir (0) le background
 * \param[in] n nombre de blobs recherchés
 * \return Vecteur contenant les n plus gros 4-blobs de mask (définis par des vecteurs de points)
 * \warning le type de mask doit être cv::Mat<uchar> (CV_8U)
 */
std::vector<std::vector<cv::Point> > nLargestBlob4Connexe(const cv::Mat &mask, int n)
{   
    // On appelle la fonction nnLargestKBlobs avec k=1
    // et on retourne le résultat obtenu
    return nLargestKBlobs(mask,n,1.);
}

/*! \brief Recherche des n plus gros blobs 8-connexes
 * \param[in] mask masque binaire où blanc (255) est l'objet et noir (0) le background
 * \param[in] n nombre de blobs recherchés
 * \return Vecteur contenant les n plus gros 8-blobs de mask (définis par des vecteurs de points)
 * \warning le type de mask doit être cv::Mat<uchar> (CV_8U)
 */
std::vector<std::vector<cv::Point> > nLargestBlob8Connexe(const cv::Mat &mask, int n)
{
    // On appelle la fonction nLargestKblobs avec k=sqrt(2)
    // et on retourne le résultat obtenu
    return nLargestKBlobs(mask,n,(float)sqrt(2.));
}

/*! \brief Recherche du plus gros blob 4-connexes
 * \param[in] mask masque binaire où blanc (255) est l'objet et noir (0) le background
 * \return Vecteur de points définissant le plus gros 4-blob
 * \warning le type de mask doit être cv::Mat<uchar> (CV_8U)
 */
std::vector<cv::Point> largestBlob4Connexe(const cv::Mat &mask)
{
    // On appelle la fonction largestKblobs avec k=1
    // et on retourne le résultat obtenu
    return largestKBlob(mask,1.);
}

/*! \brief Recherche du plus gros blob 8-connexes
 * \param[in] mask masque binaire où blanc (255) est l'objet et noir (0) le background
 * \return Vecteur de points définissant le plus gros 8-blob
 * \brief le type de mask doit être cv::Mat<uchar> (CV_8U)
 */
std::vector<cv::Point> largestBlob8Connexe(const cv::Mat &mask)
{

    // On appelle la fonction largestKBlob avec k=sqrt(2)
    // et on retourne le résultat obtenu
    return largestKBlob(mask,(float)sqrt(2.));
}

/*! \brief Recherche du blob 4-connexes à partir d'une souche s
 * \param[in] mask masque binaire où blanc (255) est l'objet et noir (0) le background
 * \param[in] s souche appartenant au 4-blob recherché
 * \return Vecteur de points définissant 4-blob auquel appartient s
 * \warning le type de mask doit être cv::Mat<uchar> (CV_8U)
 */
std::vector<cv::Point> seedBlob4Connexe(const cv::Mat &mask, cv::Point s)
{
    // On appelle la fonction seedKBlob avec k=1
    // et on retourne le résultat obtenu
    return rm::Blobs::seedKBlob(mask,s,(float)1.);
}

/*! \brief Recherche du blob 8-connexes à partir d'une souche s
 * \param[in] mask masque binaire où blanc (255) est l'objet et noir (0) le background
 * \param[in] s souche appartenant au 8-blob recherché
 * \return Vecteur de points définissant 8-blob auquel appartient s
 * \warning le type de mask doit être cv::Mat<uchar> (CV_8U)
 */
std::vector<cv::Point> seedBlob8Connexe(const cv::Mat &mask, cv::Point s)
{
    // On appelle la fonction seedKBlob avec k=sqrt(2)
    // et on retourne le résultat obtenu
    return rm::Blobs::seedKBlob(mask,s,(float)sqrt(2.));
}


/*! \brief Fonction qui affiche en couleur les blobs définis dans l'image
 * \param[in] img image porteuse des blobs (CV8U ou CV_16S)
 * \return image coloriée
 */
cv::Mat colorizeBlobs(const cv::Mat &img)
{
    cv::Mat res(img.size(),CV_8UC3);

    // On définit un ensemble de couleurs à afficher
    // Peut être étendu...
    std::vector<cv::Vec3b> couleurs;
    couleurs.clear();

    couleurs.push_back(cv::Vec3b(255,0,0));     couleurs.push_back(cv::Vec3b(0,255,0));     couleurs.push_back(cv::Vec3b(0,0,255));
    couleurs.push_back(cv::Vec3b(255,255,0));   couleurs.push_back(cv::Vec3b(0,255,255));   couleurs.push_back(cv::Vec3b(255,0,255));
    couleurs.push_back(cv::Vec3b(255,128,0));   couleurs.push_back(cv::Vec3b(0,255,128));   couleurs.push_back(cv::Vec3b(128,0,255));
    couleurs.push_back(cv::Vec3b(128,255,0));   couleurs.push_back(cv::Vec3b(0,128,255));   couleurs.push_back(cv::Vec3b(255,0,128));
    couleurs.push_back(cv::Vec3b(128,0,0));     couleurs.push_back(cv::Vec3b(0,128,0));     couleurs.push_back(cv::Vec3b(0,0,128));
    couleurs.push_back(cv::Vec3b(128,255,255)); couleurs.push_back(cv::Vec3b(255,128,255)); couleurs.push_back(cv::Vec3b(255,255,128));
    couleurs.push_back(cv::Vec3b(128,128,255)); couleurs.push_back(cv::Vec3b(255,128,128)); couleurs.push_back(cv::Vec3b(128,255,128));


    // Pour chaque pixel de l'image entrée,
    // la couleur est défini par le modulo de son ndg par la taille de couleurs
    for(int i=0;i<img.rows;i++)
    {
        for(int j=0;j<img.cols;j++)
        {
            // On gère les cas où l'image en entrée contient des int ou des uchar
            switch(img.type())
            {
            case CV_8U:
                if(img.at<uchar>(i,j)>0)
                    res.at<cv::Vec3b>(i,j)=couleurs[(int)img.at<uchar>(i,j)%couleurs.size()];
                else
                    res.at<cv::Vec3b>(i,j)=cv::Vec3b(0,0,0);
                break;
            case cv::DataType<int>::type:
                if(img.at<int>(i,j)>0)
                    res.at<cv::Vec3b>(i,j)=couleurs[(int)img.at<int>(i,j)%couleurs.size()];
                else
                    res.at<cv::Vec3b>(i,j)=cv::Vec3b(0,0,0);
                break;
            }
        }
    }

    return res.clone();
}

/*!
 * \brief Détermine le nombre de composantes connexes d'un masque
 * \param mask Masque binaire dont on cherche le nombre de composantes connexes
 * \return Nombre de composantes connexes
 */
unsigned int nBlobs8Connexe(const cv::Mat &mask)
{
    return allBlob8Connexe(mask).size();
}



}
}
