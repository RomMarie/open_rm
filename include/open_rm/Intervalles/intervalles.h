#ifndef INTERVALLES_H
#define INTERVALLES_H

#include <opencv2/opencv.hpp>

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
    Intervalle(const Intervalle &interv);

    void set(double borneInf,double borneSup, T data=T());

    double borneInf() const;
    void setBorneInf(double borneInf);

    double borneSup() const;
    void setBorneSup(double borneSup);

    T data() const;
    void setData(const T& data);

    bool isValid();
    double largeur();
    double milieu();

    /*!
     * \brief Surcharge de l'opérateur << (pour affichage via cout)
     * \param os Stream d'entrée
     * \param poly Intervalle à afficher
     * \return Stream de sortie
     */
    friend std::ostream& operator<<(std::ostream& os, const Intervalle<T>& poly){
        os<<"["<<poly._borneInf<<" ---> "<<poly._borneSup<<" | "<<poly._data<<"]";
        return os;

    }

private:
    double _borneInf; ///< Borne inférieure de l'intervalle
    double _borneSup; ///< Borne supérieure de l'intervalle
    T _data; ///< Information portée par l'intervalle
};


/*!
 * \brief Constructeur par défaut
 */
template <class T>
Intervalle<T>::Intervalle()
{
    _borneInf=0;
    _borneSup=0;
}

/*!
 * \brief Constructeur principal qui définit les propriétés de l'intervalle
 * \param borneInf Borne inférieure
 * \param borneSup Borne supérieure
 * \param data (optionnelle) données stockées par l'intervalle
 */
template<class T>
Intervalle<T>::Intervalle(double borneInf, double borneSup, T data)
{
    _borneInf=borneInf;
    _borneSup=borneSup;
    _data=data;
}

/*!
 * \brief Constructeur de recopie
 */
template<class T>
Intervalle<T>::Intervalle(const Intervalle &interv)
{
    _borneInf=interv.borneInf();
    _borneSup=interv.borneSup();
    _data=interv.data();
}

/*!
 * \brief Définit les propriétés de l'intervalle
 * \param borneInf Borne inférieure
 * \param borneSup Borne supérieure
 * \param data (optionnelle) données stockées par l'intervalle
 */
template<class T>
void Intervalle<T>::set(double borneInf, double borneSup, T data)
{
    _borneInf=borneInf;
    _borneSup=borneSup;
    _data=data;
}

/*!
 * \brief Indique la borne inférieure de l'intervalle
 * \return Borne inférieure de l'intervalle
 */
template<class T>
double Intervalle<T>::borneInf() const
{
    return _borneInf;
}

/*!
 * \brief Définit la borne inférieure de l'intervalle
 * \param borneInf Borne inférieure désirée
 */
template<class T>
void Intervalle<T>::setBorneInf(double borneInf)
{
    _borneInf = borneInf;
}

/*!
 * \brief Indique la borne supérieure de l'intervalle
 * \return Borne supérieure de l'intervalle
 */
template<class T>
double Intervalle<T>::borneSup() const
{
    return _borneSup;
}

/*!
 * \brief Définit la borne supérieure de l'intervalle
 * \param borneSup Borne supérieure désirée
 */
template<class T>
void Intervalle<T>::setBorneSup(double borneSup)
{
    _borneSup = borneSup;
}

/*!
 * \brief Met à disposition les informations portées par l'intervalle
 * \return Données portées par l'intervalle
 */
template<class T>
T Intervalle<T>::data() const
{
    return _data;
}

/*!
 * \brief Attribue un jeu de données à l'intervalle
 * \param data données à enregistrer
 */
template<class T>
void Intervalle<T>::setData(const T &data)
{
    _data=data;
}

/*!
 * \brief Vérifie si l'intervalle est valide (borneInf<borneSup)
 * \return true si l'intervalle est valide, false sinon
 */
template<class T>
bool Intervalle<T>::isValid()
{
    if(_borneInf<=_borneSup)return true;
    else return false;
}

/*!
 * \brief Indique la largeur de l'intervalle
 * \return borneSup-borneInf
 */
template <class T>
double Intervalle<T>::largeur()
{
    return _borneSup-_borneInf;
}

/*!
 * \brief Indique le point milieu de l'intervalle
 * \return (borneSup+borneInf)/2
 */
template <class T>
double Intervalle<T>::milieu()
{
    return (_borneSup+_borneInf)/2.;
}


}
}

#endif // INTERVALLES_H
