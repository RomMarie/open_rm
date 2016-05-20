#include <open_rm/VisionOmni/paramscalib.h>

#include <QFile>
#include <QSettings>

namespace rm{

namespace VisionOmni{

/*! \brief Constructeur par défaut
 *
 * Initialise les paramètres de calibration
 * \warning n'a aucun sens d'utiliser l'instance de la classe
 * sans avoir appelé l'une des méthodes loadParams
 */
ParamsCalib::ParamsCalib()
{
    _u0=0;
    _v0=0;
    _au=0;
    _av=0;
    _xi=0;
    _resU=0;
    _resV=0;
    _h=0;
}

/*! \brief Constructeur de la classe à partir d'un fichier
 *
 * \param fic Nom du fichier où sont stockés les paramètres intrinsèques
 *
 * Un exemple de fichier de configuration est donné ci-dessous :
 * \code
 * au=290
 * av=300
 * u0=306
 * v0=411
 * xi=1
 * resU=600
 * resV=800
 * h=1.02
 * \endcode
 */
ParamsCalib::ParamsCalib(std::string fic)
{
    if(!loadParams(fic))
    {
        std::cout<<"Fin prématurée : Erreur lors du chargement des paramètres de calibration"<<std::endl;
        exit(0);
    }
}

/*! \brief Constructeur de la classe à partir de variables
 */
ParamsCalib::ParamsCalib(double u0, double v0, double au, double av, double xi, int resU, int resV, double h)
{
    _u0=u0;
    _v0=v0;
    _au=au;
    _av=av;
    _xi=xi;
    _resU=resU;
    _resV=resV;
    _h=h;
}

/*! \brief Destructeur de la classe.
 *
 * N'a aucun intérêt particulière
 */
ParamsCalib::~ParamsCalib()
{

}

/*! \brief Effectue le chargement d'un fichier de paramètres de calibration
 *
 * \param fic Nom du fichier où sont stockés les paramètres intrinsèques
 * \return vrai si la lecture s'est correctement effectuée, faux sinon
 *
 * Un exemple de fichier de configuration est donné ci-dessous :
 * \code
 * au=290
 * av=300
 * u0=306
 * v0=411
 * xi=1
 * resU=600
 * resV=800
 * h=1.02
 * \endcode
 */
bool ParamsCalib::loadParams(std::string fic)
{
    QFile checkFic(fic.c_str());
    if(!checkFic.exists())
        return false;

    QSettings settings(fic.c_str(),QSettings::IniFormat);

    _u0=settings.value("u0","").toDouble();
    if(_u0==0)return false;
    _v0=settings.value("v0","").toDouble();
    if(_v0==0)return false;
    _au=settings.value("au","").toDouble();
    if(_au==0)return false;
    _av=settings.value("av","").toDouble();
    if(_av==0)return false;
    _xi=settings.value("xi","").toDouble();
    if(_xi==0)return false;
    _h=settings.value("h","").toDouble();
    if(_h==0)return false;

    return true;
}

/*! \brief Effectue le chargement des paramètres à partir de variables
 */
bool ParamsCalib::loadParams(double u0, double v0, double au, double av, double xi, int resU, int resV, double h)
{
    _u0=u0;
    _v0=v0;
    _au=au;
    _av=av;
    _xi=xi;
    _resU=resU;
    _resV=resV;
    _h=h;

    return true;
}

/*! \brief getter
 */
double ParamsCalib::u0() const
{
    return _u0;
}

/*! \brief getter
 */
double ParamsCalib::v0() const
{
    return _v0;
}

/*! \brief getter
 */
double ParamsCalib::au() const
{
    return _au;
}

/*! \brief getter
 */
double ParamsCalib::av() const
{
    return _av;
}

/*! \brief getter
 */
double ParamsCalib::xi() const
{
    return _xi;
}

/*! \brief getter
 */
int ParamsCalib::resU() const
{
    return _resU;
}

/*! \brief getter
 */
int ParamsCalib::resV() const
{
    return _resV;
}

/*! \brief getter
 */
double ParamsCalib::h() const
{
    return _h;
}


}

}





