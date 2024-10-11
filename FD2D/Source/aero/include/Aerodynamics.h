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

    //AeroState getCruiseState(double velocity);

    ////////////STATIC FUNCTIONS//////////////////
    static std::array<double, 2> rotateFromBody2Earthframe(std::array<double, 2> vector, double theta);
    static double rad2deg(double rad);
    static double deg2rad(double deg);

    template <typename T1, typename T2>
    static std::array<decltype(T1()+T2()),2 > rotateFromWind2Earthframe(std::array<T1, 2> vector, std::array<T2, 2> velocity){
        auto a1 = getAoa1(velocity);
        //Rotate clockwise. (-a1) angle is possitive anticlocwise
        return {cos(-a1)*vector[0] - sin(-a1)*vector[1],
                sin(-a1)*vector[0] + cos(-a1)*vector[1]};
    }

    template <typename T>
    static T getAoa1(std::array<T, 2> velocity){//this is without theta
        T aoa = -atan2(velocity[1], velocity[0]);
        return aoa;
    }

    template <typename T>
    static std::array<T, 2> rotateFromEarth2Bodyframe(std::array<double, 2> vector, T theta){
        return {cos(theta)*vector[0] - sin(theta)*vector[1],
                sin(theta)*vector[0] + cos(theta)*vector[1]};

    }

    template <typename T>
    static T ground_effect_cl_increase_factor(T height, double wingspan, double AR){
        return     1 + (288 * pow(height / wingspan, 0.787) *
                exp(-9.14 * pow(height / wingspan, 0.327)) /
                pow(AR, 0.882));
    }


    ////////////////EVERYTHING THAT NEEDS UAV ///////////////
    double getThetaForTrim(const UAV& uav, std::array<double, 2> velocity, const Control& control, bool apply_ground_effect, double height);
    std::array<double, 2> getCoeffs(const UAV& uav, std::array<double, 2> velocity, double theta, bool apply_ground_effect, double height);


    /////////////////////// FUNCTIONS FOR getAeroForces //////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////

    //Overload 1, to return array
    std::array<double, 2> getAeroForcesEarthframe(const UAV& uav, std::array<double, 2> velocity, double theta,  bool apply_ground_effect, double height){
        std::vector<double> forces = getAeroForcesEarthframeVector(uav, velocity, theta, apply_ground_effect, height);
        return {forces[0], forces[1]};
    }

    //Overload 2, if you dont want to explicitly tell it to not apply ground effect
    std::array<double, 2> Aerodynamics::getAeroForcesEarthframe(const UAV& uav, std::array<double, 2> velocity, double theta){
        std::vector<double> forces = getAeroForcesEarthframeVector(uav, velocity, theta, false, 0e0);
        return {forces[0], forces[1]};
    }

    //Tempalted function. Can be called both with T=double and T=CppAD::AD<double>
    template<typename T1, typename T2, typename T3>
    auto getAeroForcesEarthframeVector(
                    const UAV& uav, std::array<T1, 2> velocity, T2 theta, 
                    bool apply_ground_effect, T3 height){
        auto velocity_norm = sqrt( pow(velocity[0],2) + pow(velocity[1],2));
        auto qinf = _qinf(velocity_norm);

        // // get coeffs
        ///////////////////////////////////////
        auto a1 = getAoa1(velocity);
        auto aoa = theta + a1;

        // //Check if aoa is AD or double
        // if constexpr (std::is_same<decltype(aoa), AD<double>>::value) {
        //     // aoa is of type AD<double>
        //     AD<double>  stallAoa = _uav.getStallAoa();
        //     aoa = CppAD::CondExpLe(aoa, stallAoa, aoa, stallAoa);
        // } else {
        //     // aoa is of type double
        //     if (aoa > _uav.getStallAoa()){
        //         std::cout <<" STALL AOA. OLD AOA = "<< aoa;
        //     }
        //     aoa = std::min(aoa, _uav.getStallAoa());
        //     std::cout <<" NEW AOA = "<< aoa << std::endl;
            
        // }

        // if constexpr ( std::is_same<decltype(aoa), double>::value){
        //     aoa = std::min(aoa, uav.getStallAoa());
        //     std::cout <<" NEW AOA = "<< aoa << std::endl;
        // }



        auto cl = 2*M_PI*aoa - uav.getCl0();
        if (apply_ground_effect){
            auto cl_increase_factor = ground_effect_cl_increase_factor(height, uav.getWingspan(), uav.getAR());
            cl = cl_increase_factor * cl;
            #ifdef DEBUG
            if(VERBOSITY_LEVEL >=4)
                std::cout << "Ground effect (Aerodynamics::_Z): " << "height = " << height << "cl_increase_factor = " << cl_increase_factor << std::endl;
            #endif
        }
        auto cd = uav.getCd0() + pow(cl,2)/(M_PI*uav.getAR()*uav.get_e());
        ///////////////////////////////////////


        auto lift = qinf*uav.getSurface()*cl;
        auto drag = qinf*uav.getSurface()*cd;

        std::array<decltype(T1()+T2()), 2> Fwind = {-drag, lift};
        
        std::array<decltype(T1()+T2()), 2> Fearth =  Aerodynamics::rotateFromWind2Earthframe(Fwind, velocity);



        return  std::vector<decltype(T1()+T2())>{Fearth[0], Fearth[1]};
    }
    //////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////




private:
    template <typename T>
    T _qinf(T velocity){
        return 0.5*1.225*pow(velocity,2);
    }
};