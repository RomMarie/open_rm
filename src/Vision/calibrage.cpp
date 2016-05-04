#include <open_rm/Vision/calibrage.h>

namespace rm{
namespace Vision{

/*!
 * \brief Constructeur principal de la structure paramsCalib
 * \param U0 coordonnée verticale (en pixels) du centre optique
 * \param V0 coordonnée horizontale (en pixels) du centre optique
 * \param Au facteur de conversion m/pixels vertical
 * \param Av facteur de conversion m/pixels horizontal
 * \param Xi paramètre de miroir (modèle unifié)
 */
paramsCalib::paramsCalib(double U0, double V0, double Au, double Av, double Xi){
    u0=U0;
    v0=V0;
    au=Au;
    av=Av;
    xi=Xi;
}

}
}
