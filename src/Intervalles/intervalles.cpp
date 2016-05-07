#include <open_rm/Intervalles/intervalles.h>

namespace rm{
namespace Intervalles{

/*!
 * \brief Constructeur par défaut
 */
template <class T>
Intervalle<T>::Intervalle()
{

}

template<class T>
Intervalle<T>::Intervalle(double borneInf, double borneSup, T data)
{
    _borneInf=borneInf;
    _borneSup=borneSup;
    _data=data;
}

/*!
 * \brief Définit les propriétés de l'intervalle
 * \param borneInf Borne inférieure
 * \param borneSup Borne supérieure
 * \param data (optionnelle) données stockées par l'intervalle
 */
template<class T>
void Intervalle<T>::set(double borneInf, double borneSup, rm::Intervalles::T data)
{
    _borneInf=borneInf;
    _borneSup=borneSup;
    _data=data;
}


}
}
