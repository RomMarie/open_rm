#ifndef INTERVALLES_H
#define INTERVALLES_H

namespace rm{
namespace Intervalles{


template <class T=int>
/*!
 * \brief Classe définissant un intervalle dans R et capable d'y stocker une information quelconque
 */
class Intervalle{
public:
    Intervalle();
    Intervalle(double borneInf,double borneSup, T data=T());

    void set(double borneInf,double borneSup, T data=T());
private:
    double _borneInf; ///< Borne inférieure de l'intervalle
    double _borneSup; ///< Borne supérieure de l'intervalle
    T _data; ///< Information portée par l'intervalle
};

}
}

#endif // INTERVALLES_H
