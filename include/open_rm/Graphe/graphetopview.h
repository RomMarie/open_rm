#ifndef GRAPHETOPVIEW
#define GRAPHETOPVIEW

#include <open_rm/Graphe/graphe.h>
#include <opencv2/opencv.hpp>
#include <vector>
namespace rm{
namespace Graphe{

/*! Structure définissant un état distinct pour vue aérienne
 */
typedef struct DStateTopView{
    cv::Point coord; ///< Coordonnées du dstate dans l'image
    int arete; ///< Indice de l'arete adjacente

    void print(){ ///< Affichage du dstate
        std::cout<<"DState | coordonnees : "<<coord<<" | Arete connectee : "<<arete<<std::endl;
    }
}DStateTopView;

/*! Structure définissant un noeud topologique pour vue aérienne
 */
typedef struct NoeudTopView{
    int index; ///< Indice du noeud
    cv::Point desc; ///< Position du noeud dans l'image
    std::vector<DStateTopView> dstates; ///< Ensemble des dstates du noeud

    /*!
     * \brief Opérateur de recopie
     * \param dest noeud à recopier
     * \return noeud copié
     */
    NoeudTopView& operator=(NoeudTopView dest)
    {
        index=dest.index;
        desc=dest.desc;
        dstates=dest.dstates;

        return *this;
    }

    void print(){ ///< Affichage du noeud
        std::cout<<"Noeud "<<index<<", coordonnees : "<<desc<<std::endl;
        for(unsigned int i=0;i<dstates.size();i++)
            dstates[i].print();
    }
}NoeudTopView;

/*! Structure définissant une arete topologique pour vue aérienn
 */
typedef struct AreteTopView{
    int index; ///< Indice de l'arete

    void print(){ ///< Affichage de l'arete
        std::cout<<"Arete : "<<index<<std::endl;
    }
}AreteTopView;

}}
#endif // GRAPHETOPVIEW

