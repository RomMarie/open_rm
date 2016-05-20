#ifndef GRAPHETOPVIEW
#define GRAPHETOPVIEW

#include <open_rm/Graphe/graphe.h>
#include <opencv2/opencv.hpp>
#include <vector>
namespace rm{

/*! \brief Namespace gérant l'aspect graphe en général
 */
namespace Graphe{
/*! \brief Namespacegérant la structure de graphe
 * pour un robot au sol équipé d'une caméra omnidirectionnelle
 */

namespace Sol{
/*! Structure définissant un état distinct
 */
typedef struct DState{
    int arete; ///< indice de l'arête connectée
    bool heading; ///< Sens du DState (arrivée ou départ)
    float orientation; ///< Angle du DState dans le repère du noeud
}DState;

/*! Structure définissant un noeud topologique
 */
typedef struct Noeud{
    int index; ///< Indice du noeud
    std::vector<cv::Vec3f> sig; ///< Signature associée au noeud
    cv::Mat patch; ///< Patch visuel définissant le repère du noeud
    std::vector<DState> dstates; ///< Liste des DStates connectés au noeud

    /*!
     * \brief Opérateur de recopie
     * \param dest Noeud à copier
     * \return Noeud après copie
     */
    Noeud& operator=(Noeud dest)
    {
        index=dest.index;
        sig=dest.sig;
        patch=dest.patch;
        dstates=dest.dstates;

        return *this;
    }
}Noeud;

/*! Structure définissant une arete topologique
 */
typedef struct Arete{
    int index; ///< Indice de l'arete
    float longueur; ///< Longueur estimée de l'arete
    std::vector<std::vector<cv::Vec3f>> sigs; ///< Ensemble des signatures observées pendant le parcours

}Arete;

}}}
#endif // GRAPHETOPVIEW

