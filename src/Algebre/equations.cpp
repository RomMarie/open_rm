#include <open_rm/Algebre/equations.h>

namespace rm{
namespace Algebre{

/*!
 * \brief Calcule les solutions réelles d'une équation du 3ème ordre
 * \note basé sur la méthode de Cardan : https://fr.wikipedia.org/wiki/M%C3%A9thode_de_Cardan
 * \param a coefficient d'ordre 3
 * \param b coefficient d'ordre 2
 * \param c coefficient d'ordre 1
 * \param d coefficient d'ordre 0
 * \return vecteur contenant les solutions de l'équation
 */
std::vector<double> thirdOrderEq(double a, double b, double c, double d)
{
    if(a==0)return secondOrderEq(b,c,d);

    std::vector<double> res;

    double z=a;
    a=b/z;
    b=c/z;
    c=d/z;

    double p=b-a*a/3.;
    double q=a*(2.*a*a-9.*b)/27.+c;
    double p3=p*p*p;
    double D=q*q+4.*p3/27.;
    double offset=-a/3;

    if(D>0){
        z=sqrt(D);
        double u=(-q+z)/2.;
        double v=(-q-z)/2.;
        if(u>=0)
            u=pow(u,1/3.);
        else
            u=-pow(-u,1/3.);
        if(v>=0)
            v=pow(v,1/3.);
        else
            v=-pow(-v,1/3.);
        res.push_back(u+v+offset);
        return res;
    }
    else if(D<0){
        double u=2*sqrt(-p/3.);
        double v=acos(-sqrt(-27./p3)*q/2.)/3.;
        res.push_back(u*cos(v)+offset);
        res.push_back(u*cos(v+2*CV_PI/3.)+offset);
        res.push_back(u*cos(v+4*CV_PI/3.)+offset);
        return res;
    }
    else{
        double u;
        if(q<0)
            u=pow(-q/2.,1/3.);
        else
            u=-pow(q/2.,1/3.);
        res.push_back(2.*u+offset);
        res.push_back(-u+offset);
        return res;
    }
}

/*!
 * \brief Calcule les solutions réelles d'une équation du second ordre
 * \param a coefficient d'ordre 2
 * \param b coefficient d'ordre 1
 * \param c coefficient d'ordre 0
 * \return vecteur contenant les solutions de l'équation
 */
std::vector<double> secondOrderEq(double a, double b, double c)
{
    std::vector<double> res;
    if(a==0){
        if(b==0)return res;
        else{
            res.push_back(-c/b);
            return res;
        }
    }

    double delta=b*b-4*a*c;
    if(delta<0)return res;
    else if(delta==0){
        res.push_back(-b/(2*a));
        return res;
    }
    else{
        double sqrtdelta=sqrt(delta);
        res.push_back((-b-sqrtdelta)/(2*a));
        res.push_back((-b+sqrtdelta)/(2*a));
        return res;
    }
}


}
}
