#include <open_rm/Graphe/robotvirtuel.h>

namespace rm{
namespace Graphe{

/*!
 * \brief Constructeur principal d'un robot virtuel
 * \param posCur Pose courante
 * \param posPrec Pose précédente
 * \param d distance parcourue
 * \param n Indice du noeud de départ
 * \param b Indice de la branche de départ
 */
RobotVirtuel::RobotVirtuel(cv::Point posCur, cv::Point posPrec, float d,int n,int b):
    m_posCur(posCur),m_posPrec(posPrec),m_posInit(posPrec),m_distTraveled(d),m_indNoeudDepart(n),
    m_brancheDepart(b)
{
}

/*!
 * \brief Accesseur de la pose initiale
 * \return Position initiale du robot virtuel
 */
cv::Point RobotVirtuel::getPosInit()
{
    return m_posInit;
}

/*!
 * \brief Accesseur de la pose courante
 * \return Position courante du robot virtuel
 */
cv::Point RobotVirtuel::getPosCur()
{
    return m_posCur;
}

/*!
 * \brief Accesseur de la pose précédente
 * \return  Position précédente du robot virtuel
 */
cv::Point RobotVirtuel::getPosPrec()
{
    return m_posPrec;
}

/*!
 * \brief Accesseur du noeud de départ
 * \return Indice du noeud de départ
 */
int RobotVirtuel::getIndNoeudDepart()
{
    return m_indNoeudDepart;
}

/*!
 * \brief Accesseur de la branche de départ
 * \return Indice de la branche de départ
 */
int RobotVirtuel::getBrancheDepart()
{
    return m_brancheDepart;
}

/*!
 * \brief Accesseur de la distance parcourue
 * \return Distance parcourue par le robot virtuel
 */
float RobotVirtuel::getDistTraveled()
{
    return m_distTraveled;
}

/*!
 * \brief Déplace le robot
 * \param posNext Position à destination du mouvement
 * \param dist Amplitude du mouvement
 */
void RobotVirtuel::move(cv::Point posNext, float dist)
{
    m_posPrec=m_posCur;
    m_posCur=posNext;
    m_distTraveled+=dist;
}

/*!
 * \brief Destructeur de la classe
 */
RobotVirtuel::~RobotVirtuel()
{

}

}
}

