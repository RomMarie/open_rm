#ifndef POLYNOMES_H
#define POLYNOMES_H
#include <opencv2/opencv.hpp>
namespace rm{
namespace Algebre{

void divisionPolynomiale(std::vector<double> N,std::vector<double> D,
                         std::vector<double>& q,std::vector<double>& r);

}
}

#endif // POLYNOMES_H
