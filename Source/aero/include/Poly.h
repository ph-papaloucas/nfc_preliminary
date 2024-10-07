#pragma once

template < typename T, size_t deg = 0> //deg = degree of polynomial
class Poly
{
public:
    Poly():_coeffs({}){};
    Poly(std::array<T, deg> polynomial_coefficients):_coeffs(polynomial_coefficients){};


    std::array<T, deg> getCoefficients(){
        return _coeffs;
    }


    T getValue(T x){
        T value = 0;
        for (int i=0; i < deg; ++i){
            value+=_coeffs[i]*pow(x, i);
        }
        return value;
    }

private:
    std::array<T, deg> _coeffs;
};