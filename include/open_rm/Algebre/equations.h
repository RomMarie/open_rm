#ifndef EQUATIONS_H
#define EQUATIONS_H

#include <opencv2/opencv.hpp>

namespace rm{
namespace Algebre{

std::vector<double> secondOrderEq(double a,double b, double c);
std::vector<double> thirdOrderEq(double a,double b,double c,double d);

}
}

#endif // EQUATIONS_H
