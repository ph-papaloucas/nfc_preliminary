#pragma once
#include <array>
#include <cmath>
#include <cppad/cppad.hpp>


namespace utils{

    template <typename T>
    std::array<T, 2> rotateFromEarth2Bodyframe(std::array<double, 2> vector, T theta)
    {
        return {cos(theta)*vector[0] - sin(theta)*vector[1],
                sin(theta)*vector[0] + cos(theta)*vector[1]};

    }

    std::array<double, 2> rotateFromBody2Earthframe(std::array<double, 2> vector, double theta){
        return {cos(theta)*vector[0] + sin(theta)*vector[1],
                -sin(theta)*vector[0] + cos(theta)*vector[1]};

    }

    template <typename T1, typename T2>
    std::array< decltype(T1()+T2()),2 > rotateFromWind2Earthframe(std::array<T1, 2> vector, T2 a1){
        //Rotate clockwise. (-a1) angle is possitive anticlocwise
        return {cos(-a1)*vector[0] - sin(-a1)*vector[1],
                sin(-a1)*vector[0] + cos(-a1)*vector[1]};
    }


    double rad2deg(double rad){
        return rad*180/M_PI;
    }

    double deg2rad(double deg){
        return deg*M_PI/180;
    }

}

