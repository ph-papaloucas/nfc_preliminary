#pragma once
#include "UAV.h"
#include "State.h"
#include <array>
#include <cppad/cppad.hpp>
using CppAD::AD; 

#include <cmath>


class Control; //forward declaration
class Aerodynamics{
public:
    Aerodynamics(){};
    Aerodynamics(const UAV& uav);

    AeroState getCruiseState(double velocity);
    double getThetaForTrim(std::array<double, 2> velocity,const Control& control, bool apply_ground_effect, double height);
    std::array<double, 2> getCoeffs(std::array<double, 2> velocity, double theta, bool apply_ground_effect, double height);
    template <typename T>
    static std::array<T, 2> rotateFromEarth2Bodyframe(std::array<double, 2> vector, T theta);



    static std::array<double,2 > rotateFromWind2Earthframe(std::array<double, 2> vector, std::array<double, 2> velocity_vector);
    static std::array<double, 2> rotateFromBody2Earthframe(std::array<double, 2> vector, double theta);
    static double rad2deg(double rad);
    static double deg2rad(double deg);
    template <typename T>
    static T getAoa1(std::array<T, 2> velocity){//this is without theta
        T aoa = -atan2(velocity[1], velocity[0]);
        return aoa;
    }


    template <typename T>
    static T ground_effect_cl_increase_factor(T height, double wingspan, double AR){
        return     1 + (288 * pow(height / wingspan, 0.787) *
                exp(-9.14 * pow(height / wingspan, 0.327)) /
                pow(AR, 0.882));
    }



    /////////////////////// FUNCTIONS FOR getAeroForces //////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////

    //Overload 1, to return array
    std::array<double, 2> getAeroForcesEarthframe(std::array<double, 2> velocity, double theta,  bool apply_ground_effect, double height){
        std::vector<double> forces = getAeroForcesEarthframeVector(velocity, theta, apply_ground_effect, height);
        return {forces[0], forces[1]};
    }

    //Overload 2, if you dont want to explicitly tell it to not apply ground effect
        std::array<double, 2> Aerodynamics::getAeroForcesEarthframe(std::array<double, 2> velocity, double theta){
        std::vector<double> forces = getAeroForcesEarthframeVector(velocity, theta, false, 0);
        return {forces[0], forces[1]};
    }

    //Tempalted function. Can be called both with T=double and T=CppAD::AD<double>
    template<typename T1, typename T2, typename T3>
    auto getAeroForcesEarthframeVector(
                    std::array<T1, 2> velocity, T2 theta, 
                    bool apply_ground_effect, T3 height){
        auto velocity_norm = sqrt( pow(velocity[0],2) + pow(velocity[1],2));
        auto qinf = _qinf(velocity_norm);

        // // get coeffs
        ///////////////////////////////////////
        auto a1 = getAoa1(velocity);
        auto aoa = theta + a1;
        auto cl = 2*M_PI*aoa - _uav.getCl0();
        if (apply_ground_effect){
            auto cl_increase_factor = ground_effect_cl_increase_factor(height, _uav.getWingspan(), _uav.getAR());
            cl = cl_increase_factor * cl;
            #ifdef DEBUG
            if(VERBOSITY_LEVEL >=4)
                std::cout << "Ground effect (Aerodynamics::_Z): " << "height = " << height << "cl_increase_factor = " << cl_increase_factor << std::endl;
            #endif
        }
        auto cd = _uav.getCd0() + pow(cl,2)/(2*M_PI*_uav.getAR()*_uav.get_e());
        ///////////////////////////////////////


        auto lift = qinf*_uav.getSurface()*cl;
        auto drag = qinf*_uav.getSurface()*cd;


        auto Xaero = cos(-a1)*(-drag) + sin(-a1)*lift;
        auto Zaero = -sin(-a1)*(-drag) + cos(-a1)*lift;


        return  std::vector<decltype(T1()+T2())>{Xaero, Zaero};
    }
    //////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////




private:
    template <typename T>
    T _qinf(T velocity){
        return 0.5*1.225*pow(velocity,2);
    }

    UAV _uav;
};