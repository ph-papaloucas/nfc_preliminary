#include "Aerodynamics.h"
#include "Control.h"

Aerodynamics::Aerodynamics(const UAV& uav):_uav(uav){};


AeroState Aerodynamics::getCruiseState(double velocity){
    double lift = _uav.getTotalMass()*9.81;

    double cl = lift/(_qinf(velocity)*_uav.getSurface());
    double aoa = (cl - _uav.getCl0())/(2*M_PI);
    

    return AeroState(aoa, cl, velocity);
}

template <typename T>
std::array<T, 2> Aerodynamics::rotateFromEarth2Bodyframe(std::array<double, 2> vector, T theta){
    return {cos(theta)*vector[0] - sin(theta)*vector[1],
            sin(theta)*vector[0] + cos(theta)*vector[1]};

}


std::array<double, 2> Aerodynamics::rotateFromBody2Earthframe(std::array<double, 2> vector, double theta){
    return {cos(theta)*vector[0] + sin(theta)*vector[1],
            -sin(theta)*vector[0] + cos(theta)*vector[1]};

}


double Aerodynamics::rad2deg(double rad){
    return rad*180/M_PI;
}

double Aerodynamics::deg2rad(double deg){
    return deg*M_PI/180;
}




double Aerodynamics::getThetaForTrim(std::array<double, 2> velocity,const Control& control, bool apply_ground_effect, double height){
    // //prepare CppAd Fun and independent var
    size_t n = 1;//1 independent variable
    std::vector<CppAD::AD<double>> th(n);
    th[0] = 0.1;
    CppAD::Independent(th);
    AD<double> faero = Aerodynamics::getAeroForcesEarthframeVector(velocity, th[0], apply_ground_effect, height)[1];
    AD<double>  thrust = control.getThrust(Aerodynamics::rotateFromEarth2Bodyframe(velocity, th[0]));
    AD<double> fz =  faero - _uav.getTotalMass()*9.81 + thrust*CppAD::sin(th[0]);
    CppAD::ADFun<double> fun(th, {fz}); // Create function object


        #ifdef DEBUG
        if (VERBOSITY_LEVEL >=0){
            std::vector<double> theta_test = {0.1};
            std::array<double, 2> f_aero =  getAeroForcesEarthframe(velocity, theta_test[0], apply_ground_effect, height);
            double thrust = control.getThrust(Aerodynamics::rotateFromEarth2Bodyframe(velocity, theta_test[0]));
            double fz = f_aero[1] - _uav.getTotalMass()*9.81 + thrust*sin(theta_test[0]);
            std::vector<double> fun_val_test = fun.Forward(0, theta_test);
            std::cout << "(f_aero-mg+thrust*sin(theta) vs _Z" << std::endl;
            std::cout << fz << "  vs  " << fun_val_test[0] << std::endl;
        }
        #endif

    // // NEWTON RAPHSON SETTINGS
    double iterations = 0;
    const double tolerance= 1e-8; 
    const int max_iterations = 900;
    double relax = 0.7;


    // //initialize guess and prepare vars
    std::vector<double> theta(n);
    double theta_prev = 10000;
    std::vector<double> fun_val  = {10000}; //this is FZ
    double theta_max = _uav.getStallAoa() - getAoa1(velocity);

    // // NEWTON RAPHSON TO FIND THETA, SO Fz = Z = 0
    while( abs(fun_val[0]) > tolerance ){
        iterations++;
        if (iterations == 200)
            relax = 0.15;

        fun_val = fun.Forward(0, theta);
        std::vector<double> fun_der = fun.Jacobian(theta);
        theta_prev = theta[0];
        theta[0] = theta_prev - fun_val[0]/fun_der[0]; 
        //aoa = theta + a1
        theta[0] = std::min(theta[0],theta_max);

        /* MORE CONDITIONS TO STOP NEWTON - RAPHSON*/
        if(theta[0] == theta_max){
            #ifdef DEBUG
            if (VERBOSITY_LEVEL >=2){
                std::cout << "theta max reached. Cant trim at the moment ... Fz = " << fz << std::endl;
            }
            #endif

            break;
        }

        if (iterations >= max_iterations){
            #ifdef DEBUG
            if (VERBOSITY_LEVEL >=2){
                std::cout << "max iterations reached. Cant trim at the moment ... Fz = " << fz << std::endl;
            }
            #endif

            break;
        }

    }

    #ifdef DEBUG
    if (VERBOSITY_LEVEL >=3){
        std::cout << "theta tolerance reached. Fz = " << fun_val[0] << std::endl;
    }
    #endif
    return theta[0];
}

