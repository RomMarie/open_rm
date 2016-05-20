#ifndef CALIBRAGE_H
#define CALIBRAGE_H



namespace rm{
/*!
 * Namespace gérant les aspects vision
 */
namespace Vision{

/*!
 * \brief Structure regroupant les paramètres intrinsèques d'une caméra à point de vue unique
 */
struct paramsCalib{
    paramsCalib(double U0,double V0,double Au,double Av,double Xi);

    double u0; ///< Coordonnée verticale (en pixels) du centre optique
    double v0; ///< Coordonnée horizontale (en pixels) du centre optique
    double au; ///< Facteur de conversion m/pixels vertical
    double av; ///< Facteur de conversion m/pixels horizontal
    double xi; ///< Paramètre de miroir (modèle unifié)
};

}
}


#endif // CALIBRAGE_H
