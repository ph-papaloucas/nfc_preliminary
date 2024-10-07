#include "Aerodynamics.h"
#include "Control.h"

Aerodynamics::Aerodynamics(const UAV& uav):_uav(uav){};

double Aerodynamics::_qinf(double velocity){
    return 0.5*1.225*pow(velocity,2);
}

AeroState Aerodynamics::getCruiseState(double velocity){
    double lift = _uav.getTotalMass()*9.81;

    double cl = lift/(_qinf(velocity)*_uav.getSurface());
    double aoa = (cl - _uav.getCl0())/(2*M_PI);
    

    return AeroState(aoa, cl, velocity);
}

std::array<CppAD::AD<double>, 2> Aerodynamics::rotateFromEarth2Bodyframe(std::array<double, 2> vector, CppAD::AD<double> theta){
    return {CppAD::cos(theta)*vector[0] - CppAD::sin(theta)*vector[1],
            CppAD::sin(theta)*vector[0] + CppAD::cos(theta)*vector[1]};

}

std::array<double, 2> Aerodynamics::rotateFromEarth2Bodyframe(std::array<double, 2> vector, double theta){
    return {CppAD::cos(theta)*vector[0] - CppAD::sin(theta)*vector[1],
            CppAD::sin(theta)*vector[0] + CppAD::cos(theta)*vector[1]};
}


std::array<double, 2> Aerodynamics::rotateFromBody2Earthframe(std::array<double, 2> vector, double theta){
    return {cos(theta)*vector[0] + sin(theta)*vector[1],
            -sin(theta)*vector[0] + cos(theta)*vector[1]};

}
std::array<double,2 > Aerodynamics::rotateFromWind2Earthframe(std::array<double, 2> vector, std::array<double, 2> velocity){
    double a1 = _getAoa1(velocity);
    double theta = -a1; //because a1 is negative (see aerodynamic aoa positive)
    return {cos(theta)*vector[0] + sin(theta)*vector[1],
            -sin(theta)*vector[0] + cos(theta)*vector[1]};
}

double Aerodynamics::rad2deg(double rad){
    return rad*180/M_PI;
}

double Aerodynamics::deg2rad(double deg){
    return deg*M_PI/180;
}


std::array<double, 2> Aerodynamics::getAeroForcesEarthframe(std::array<double, 2> velocity, double theta,  bool apply_ground_effect, double height){
    double velocity_norm = sqrt( pow(velocity[0],2) + pow(velocity[1],2));
    double qinf = _qinf(velocity_norm);

    std::array<double, 2> cl_cd= getCoeffs(velocity, theta, apply_ground_effect, height);

    double lift = qinf*_uav.getSurface()*cl_cd[0];
    double drag = qinf*_uav.getSurface()*cl_cd[1];

    std::array<double, 2> F = {-drag, lift};

	return   Aerodynamics::rotateFromWind2Earthframe(F, velocity);
}

std::array<double, 2> Aerodynamics::getAeroForcesEarthframe(std::array<double, 2> velocity, double theta){
    return getAeroForcesEarthframe(velocity, theta, false, 0);

}

std::array<double, 2> Aerodynamics::getCoeffs(std::array<double, 2> velocity, double theta, bool apply_ground_effect, double height){
    double aoa = theta + _getAoa1(velocity);
    double cl = 2*M_PI*aoa - _uav.getCl0();
    if (apply_ground_effect){
        double cl_increase_factor =
            1 + (288 * pow(height / _uav.getWingspan(), 0.787) *
                    exp(-9.14 * pow(height / _uav.getWingspan(), 0.327)) /
                    pow(_uav.getAR(), 0.882));
        cl = cl_increase_factor * cl;
        #ifdef DEBUG
        if(VERBOSITY_LEVEL >=2)
            std::cout << "Ground effect: " << "height = " << height << "cl_increase_factor = " << cl_increase_factor << std::endl;
        #endif
    }
    double cd = _uav.getCd0() + pow(cl,2)/(2*M_PI*_uav.getAR()*_uav.get_e());

    #ifdef DEBUG
        if(VERBOSITY_LEVEL >=3){
            std::cout << "velocity = " << velocity[0] << " " <<velocity[1] << std::endl;
            std::cout << "aoa  = " << Aerodynamics::rad2deg(aoa) << " [deg]" << std::endl;
            std::cout << " cl = " << cl << "  cd = " << cd <<std::endl;
        }
    #endif


    return{cl, cd};
}

double Aerodynamics::getThetaForTrim(std::array<double, 2> velocity,const Control& control, bool apply_ground_effect, double height){
    // //prepare CppAd Fun and independent var
    size_t n = 1;//1 independent variable
    std::vector<CppAD::AD<double>> th(n);
    th[0] = 0.1;
    CppAD::Independent(th);
    CppAD::AD<double> fz = Aerodynamics::_Z(th, velocity, apply_ground_effect, height, control);
    CppAD::ADFun<double> fun(th, {fz}); // Create function object


        #ifdef DEBUG
        if (VERBOSITY_LEVEL >=0){
            std::vector<double> theta_test = {0.1};
            std::array<double, 2> f_aero =  getAeroForcesEarthframe(velocity, theta_test[0]);
            double thrust = control.getThrust(Aerodynamics::rotateFromEarth2Bodyframe(velocity, theta_test[0]));
            double fz = f_aero[1] - _uav.getTotalMass()*9.81 + thrust*sin(theta_test[0]);
            std::vector<double> fun_val_test = fun.Forward(0, theta_test);
            std::cout << "(f_aero-mg+thrust*sin(theta) vs _Z " << std::endl;
            std::cout << fz << "  vs  " << fun_val_test[0] << std::endl;
            }
        #endif

    // // solver settings (its just a newton raphson)
    double iterations = 0;
    const double tolerance_for_fz = 1e-7; 
    const double tolerance_for_theta = 1e-7; 
    const int max_iterations = 900;
    double relax = 0.7;


    // //initialize guess and prepare vars
    std::vector<double> theta(n);
    double theta_prev = 10000;

    bool fz_tolerance_reached = false;
    bool theta_max_reached = false;
    double theta_max = _uav.getStallAoa() - _getAoa1(velocity);
    while(  ( (std::abs(theta_prev - theta[0]) > tolerance_for_theta) || (!fz_tolerance_reached) )&& (!theta_max_reached)){
        iterations++;
            if (iterations == 200)
                relax = 0.15;

            std::vector<double> fun_val = fun.Forward(0, theta);
            std::vector<double> fun_der = fun.Jacobian(theta);
            theta_prev = theta[0];
            theta[0] = theta_prev - fun_val[0]/fun_der[0]; 

            //aoa = theta + a1
            theta[0] = std::min(theta[0],theta_max);
    
            
            if (std::abs(theta_prev - theta[0]) < tolerance_for_theta){
                //check tolerance for Fz force..
                std::array<double, 2> f_aero =  getAeroForcesEarthframe(velocity, theta[0]);
                double thrust = control.getThrust(Aerodynamics::rotateFromEarth2Bodyframe(velocity, theta[0]));
                double fz = f_aero[1] - _uav.getTotalMass()*9.81 + thrust*sin(theta[0]);
                if (abs(fz) < tolerance_for_fz)
                    fz_tolerance_reached = true;
                #ifdef DEBUG
                    if (VERBOSITY_LEVEL >=2){
                        std::cout << "theta tolerance reached. Fz = " << fz << std::endl;
                    }
                #endif
                if(theta[0] == theta_max){
                    #ifdef DEBUG
                    if (VERBOSITY_LEVEL >=2){
                        std::cout << "theta max reached. Cant trim at the moment ... Fz = " << fz << std::endl;
                    }
                    #endif
                    theta_max_reached = true;
                }
            }
            if (iterations >= max_iterations){
                std::cerr << "Aerodynamics::getThetaForTrim UNDEFINED BEHAVIOR. Max iterations reached without trimming\n";
                break;
            }


    }
    // const double a1 = _getAoa1(velocity);
    // const double velocity_norm = sqrt(pow(velocity[0],2) + pow(velocity[1],2));
    // const double qinf = _qinf(velocity_norm);

    // //
    // double iterations = 0;
    // const double tolerance_for_fz = 1e-7; 
    // const double tolerance_for_theta = 1e-50;
    // const int max_iterations = 900;
    // double relax = 0.3;

    // //Initialize guess and prepare variables
    // double theta = 0.0;
    // double theta_prev = 1;
    // std::array<double, 2> cl_cd = {0,0};
    // double drag = 0;
    // double thrust = 0;
    // double aoa = 0;

    // bool fz_tolerance_reached = false;

    // while( (std::abs(theta_prev - theta) > tolerance_for_theta) || (!fz_tolerance_reached) ){
    //     iterations++;

    //     if (iterations == 200)
    //         relax = 0.15;

    //     // Solve for aoa so that theta_prev = theta
    //     theta_prev = theta;
    //     cl_cd = getCoeffs({velocity[0], velocity[1]}, theta, false, 0);
    //     drag = cl_cd[1]*qinf*_uav.getSurface();
    //     thrust = control.getThrust(Aerodynamics::rotateFromEarth2Bodyframe(velocity, theta));
    //     theta = relax*
    //             (
    //             + drag*sin(a1)
    //             + _uav.getTotalMass()*9.81  
    //             - thrust*sin(theta) 
    //             - (2*M_PI*a1 - _uav.getCl0())*qinf*_uav.getSurface()*cos(a1)
    //             )
    //             /
    //             (
    //                 2*M_PI*qinf*_uav.getSurface()*cos(a1)
    //             ) 
    //             + (1-relax)*theta;

    //     //aoa = theta + a1
    //     theta = std::min(theta, _uav.getStallAoa() - a1);
    //     std::array<double, 2> u =  getAeroForcesEarthframe(velocity, theta);
    //     #ifdef DEBUG
    //         if (VERBOSITY_LEVEL >=4){
    //             std::cout << "iterations = " << iterations <<std::endl;
    //             std::cout << "error = " << theta - theta_prev << std::endl;
    //             std::cout << "theta = " << theta << std::endl;
    //             std::cout << "theta_prev " << theta_prev << std::endl;
    //         }
    //     #endif

    //     if (std::abs(theta_prev - theta) < tolerance_for_theta){
    //         //check tolerance for Fz force..
    //         std::array<double, 2> u = getAeroForcesEarthframe(velocity, theta);
    //         double fz = u[1] - _uav.getTotalMass()*9.81 + thrust*sin(theta);
    //         if (abs(fz) < tolerance_for_fz)
    //             fz_tolerance_reached = true;
    //         #ifdef DEBUG
    //             if (VERBOSITY_LEVEL >=4){
    //                 std::cout << "theta tolerance reached. Fz = " << fz << std::endl;
    //             }
    //         #endif
    //     }
    //     if (iterations >= max_iterations){
    //         std::cerr << "Aerodynamics::getThetaForTrim UNDEFINED BEHAVIOR. Max iterations reached without trimming\n";
    //         break;
    //     }
    // }



    std::cout << "test " << std::endl;
    return theta[0];
}

double Aerodynamics::_getAoa1(std::array<double, 2> velocity){ //this is without theta
    double aoa = -atan2(velocity[1], velocity[0]);
    #ifdef DEBUG
        if(VERBOSITY_LEVEL>=3)
            std::cout << "_getAoa1 = " << Aerodynamics::rad2deg( aoa) <<std::endl;
    #endif
    return aoa;
}


CppAD::AD<double> Aerodynamics::_Z(
const std::vector< AD<double> >& th, std::array<double, 2> velocity, 
bool apply_ground_effect, double height,
const Control& control){
    double velocity_norm = sqrt( pow(velocity[0],2) + pow(velocity[1],2));
    double qinf = _qinf(velocity_norm);

    // // get coeffs
    ///////////////////////////////////////
    double a1 = _getAoa1(velocity);
    CppAD::AD<double> aoa = th[0] + a1;
    CppAD::AD<double> cl = 2*M_PI*aoa - _uav.getCl0();
    if (apply_ground_effect){
        double cl_increase_factor =
            1 + (288 * pow(height / _uav.getWingspan(), 0.787) *
                    exp(-9.14 * pow(height / _uav.getWingspan(), 0.327)) /
                    pow(_uav.getAR(), 0.882));
        cl = cl_increase_factor * cl;
        #ifdef DEBUG
        if(VERBOSITY_LEVEL >=2)
            std::cout << "Ground effect (Aerodynamics::_Z): " << "height = " << height << "cl_increase_factor = " << cl_increase_factor << std::endl;
        #endif
    }
    CppAD::AD<double> cd = _uav.getCd0() + pow(cl,2)/(2*M_PI*_uav.getAR()*_uav.get_e());
    ///////////////////////////////////////


    CppAD::AD<double> lift = qinf*_uav.getSurface()*cl;
    CppAD::AD<double> drag = qinf*_uav.getSurface()*cd;


    //CppAD::AD<double> Xaero = CppAD::cos(-a1)*(-drag) + CppAD::sin(-a1)*lift;
    CppAD::AD<double> Zaero = -CppAD::sin(-a1)*(-drag) + CppAD::cos(-a1)*lift;

    // // i want velocity_bodyframe to get thrust...
    CppAD::AD<double>  thrust = control.getThrust(Aerodynamics::rotateFromEarth2Bodyframe(velocity, th[0]));
    CppAD::AD<double> Z =  Zaero - _uav.getTotalMass()*9.81 + thrust*CppAD::sin(th[0]);

	return   Z;
}