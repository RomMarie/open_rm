#ifndef ROBOTVIRTUEL_H
#define ROBOTVIRTUEL_H
#include <opencv2/core/core.hpp>

namespace rm{
namespace Graphe{

/*!
 * \brief Classe définissant un robot virtuel évoluant dans une vue aérienne
 */
class RobotVirtuel
{
public:
    RobotVirtuel(cv::Point posCur, cv::Point posPrec, float d, int n,int b);
    cv::Point getPosInit();
    cv::Point getPosCur();
    cv::Point getPosPrec();
    int getIndNoeudDepart();
    int getBrancheDepart();
    float getDistTraveled();
    void move(cv::Point posNext,float dist);
    ~RobotVirtuel();
private:
    cv::Point m_posInit; ///< Pose initiale du robot virtuel
    cv::Point m_posPrec; ///< Pose précédente
    cv::Point m_posCur; ///< Pose courante
    int m_indNoeudDepart; ///< Indice du noeud de départ
    int m_brancheDepart; ///< Indice de la branche de départ
    float m_distTraveled; ///< Distance parcourue

};


}
}

#endif // ROBOTVIRTUEL_H
