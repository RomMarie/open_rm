#ifndef POLYNOMES_H
#define POLYNOMES_H
#include <opencv2/opencv.hpp>
#include <open_rm/Intervalles/intervalles.h>

namespace rm{
namespace Algebre{

/*!
 * \brief Classe permettant la manipulation d'un polynome d'ordre quelconque
 *
 */
class Polynome{
public:
    // Constructeurs
    Polynome();
    Polynome(std::vector<double> coefs);
    Polynome(const Polynome& poly);


    // Opérations sur polynome
    void invert();
    double compute(double x);
    Polynome derivate();
    Polynome operator+(const Polynome& poly);
    Polynome operator-(const Polynome& poly);
    Polynome operator*(const Polynome& poly);
    Polynome operator/(const Polynome& poly);
    Polynome operator%(const Polynome& poly);
    friend std::ostream& operator<<(std::ostream& os, const Polynome& poly);

    void divisionPolynomiale(const Polynome& den, Polynome& Q, Polynome& R);
    std::vector<rm::Intervalles::Intervalle<int> > sturmSequence(double start,double end, double step);

    // Méthodes get/set
    void set(std::vector<double> coefs);
    std::vector<double> coefs() const;
private:
    std::vector<double> _coefs; ///< Coefficients du polynome (ordre croissant)
};

}
}

#endif // POLYNOMES_H
