#ifndef PARAMSCALIB_H
#define PARAMSCALIB_H

#include <string>
#include <iostream>
#include <cstdlib>
//#include <qt5/QtCore/QFile>
//#include <qt5/QtCore/QSettings>

namespace rm
{

namespace VisionOmni
{

/*! \brief Charge et met à disposition les paramètres d'une caméra omni
 *
 * La classe peut être initialisée soit par un ensemble de variables, soit par un fichier
 * de configuration. Elle est ensuite fournie en entrée aux autres classes manipulant
 * la vision Omni.
 */
class ParamsCalib
{
public:
    ParamsCalib();
    ParamsCalib(std::string fic);
    ParamsCalib(double u0,double v0,double au,double av,double xi,int resU,int resV,double h=1);
    ~ParamsCalib();

    bool loadParams(std::string fic);
    bool loadParams(double u0,double v0,double au,double av,double xi,int resU,int resV,double h=1);

    double u0() const;
    double v0() const;
    double au() const;
    double av() const;
    double xi() const;
    int resU() const;
    int resV() const;
    double h() const;



private:
    double _u0;
    double _v0;
    double _au;
    double _av;
    double _xi;
    int _resU;
    int _resV;
    double _h;
};

}
}

#endif // PARAMSCALIB_H
