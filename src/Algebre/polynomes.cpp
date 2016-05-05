#include <open_rm/Algebre/polynomes.h>

namespace rm{
namespace Algebre{

using namespace std;

/*!
 * \brief Effectue la division euclidienne d'un polynome \a N par un autre \a D
 *
 * La fonction retourne \a q et \a r tels que N=qD+r
 * \note basé sur https://fr.wikipedia.org/wiki/Division_d'un_polyn%C3%B4me et https://rosettacode.org/wiki/Polynomial_long_division#C.2B.2B
 *
 * \param N Polynome servant de numérateur
 * \param D Polynome servant de dénominateur
 * \param q division euclidienne de N par D
 * \param r reste de la division euclidienne de N par D
 */
void divisionPolynomiale(std::vector<double> N, std::vector<double> D, std::vector<double> &q, std::vector<double> &r)
{
    std::vector<double> d(N.size());

    int dd, dq, dr;
    int i;

    unsigned int dN=N.size()-1;
    unsigned int dD=D.size()-1;
    dq = dN-dD;
    dr = dN-dD;
    q.resize(dq+1);
    r.resize(dr+1);
    if( dN >= dD ) {
        while(dN >= dD) {
            for( i = 0 ; i < dN + 1 ; i++ ) {
                d[i] = 0;
            }

            for( i = 0 ; i < dD + 1 ; i++ ) {
                d[i+dN-dD] = D[i];
            }
            dd = dN;

            q[dN-dD] = N[dN]/d[dd];

            for( i = 0 ; i < dq + 1 ; i++ ) {
                d[i] = d[i] * q[dN-dD];
            }

            for( i = 0 ; i < dN + 1 ; i++ ) {
                N[i] = N[i] - d[i];
            }
            dN--;

        }

    }

    for( i = 0 ; i < dN + 1 ; i++ ) {
        r[i] = N[i];
    }

}

}
}
