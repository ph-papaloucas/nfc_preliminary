#pragma once


template <size_t deg = 0> //deg = degree of polynomial
class Poly
{
public:
    Poly():_coeffs({}){};
    Poly(std::array<double, deg> polynomial_coefficients):_coeffs(polynomial_coefficients){};


    std::array<double, deg> getCoefficients(){
        return _coeffs;
    }


    double getValue(double x){
        double value = 0;
        for (int i=0; i < deg; ++i){
            value+=_coeffs[i]*pow(x, i);
        }
        return value;
    }

private:
    std::array<double, deg> _coeffs;

};