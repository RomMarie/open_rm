#ifndef CALIBRAGE_H
#define CALIBRAGE_H



namespace rm{
namespace Vision{

/*!
 * \brief Structure regroupant les paramètres intrinsèques d'une caméra à point de vue unique
 */
struct paramsCalib{
    /*!
     * \brief Constructeur principal de la structure paramsCalib
     * \param U0 coordonnée verticale (en pixels) du centre optique
     * \param V0 coordonnée horizontale (en pixels) du centre optique
     * \param Au facteur de conversion m/pixels vertical
     * \param Av facteur de conversion m/pixels horizontal
     * \param Xi paramètre de miroir (modèle unifié)
     */
    paramsCalib(double U0,double V0,double Au,double Av,double Xi);

    /*!
     * \brief coordonnée verticale (en pixels) du centre optique
     */
    double u0;
    /*!
     * \brief coordonnée horizontale (en pixels) du centre optique
     */
    double v0;
    /*!
     * \brief facteur de conversion m/pixels vertical
     */
    double au;
    /*!
     * \brief facteur de conversion m/pixels horizontal
     */
    double av;
    /*!
     * \brief paramètre de miroir (modèle unifié)
     */
    double xi;
};

}
}


#endif // CALIBRAGE_H
