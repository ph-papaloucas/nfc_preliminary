#include "Jacobians.h"

Jacobians::Jacobians()
:ts(0), tf(0), _u(0), _b(0){
    p = {};
    b = {};
    // AR = 0;
    // S = 0;
    // cl0 = 0;
    // cd0 = 0;
    // e = 0;
    // mass = 0;

}
Jacobians::Jacobians(UAV uav, std::array<double, 6> p, std::array<double, r> b, double ts, double tf)
:_uav(uav), _aero(Aerodynamics(uav)), p(p), b(b), ts(ts), tf(tf), _u(n), _b(r){
    // #ifdef DEBUG
    // if (VERBOSITY_LEVEL >=3){
    // }
    // #endif

    _u = {1,1,1,1};


    std::cout<< "Jacobians::Jacobians constructor checks if it recieved UAV stats without a problem\n";
    // AR = uav.getAR();
    // S = uav.getSurface();
    // cl0 = uav.getCl0();
    // cd0 = uav.getCd0();
    // e = uav.get_e();
    //mass = uav.getTotalMass();
    //_aero = Aerodynamics(uav);



    // #ifdef DEBUG
    //     if (VERBOSITY_LEVEL >=3){
    //             // Print the values to the console
    //         std::cout << "AR = " << AR << std::endl;
    //         std::cout << "S = " << S << std::endl;
    //         std::cout << "cl0 = " << cl0 << std::endl;
    //         std::cout << "cd0 = " << cd0 << std::endl;
    //         std::cout << "e = " << e << std::endl;
    //     }
    // #endif

}


//std::array<std::array<double, Jacobians::n>, Jacobians::m> 
std::vector<double> Jacobians::F_u(std::array<double, n> u, std::array<double, nc> c){

    std::vector<double> uv(u.begin(), u.end()); //needs to be vector to be passsed to jacad.Jacobian
    CppAD::ADFun<double> fun = F_u_fun(c);
    std::vector<double>  jacvect = fun.Jacobian(uv);

    return jacvect;// Jacobians::getJacMatrix(jacvect);
}

//std::array<std::array<double, Jacobians::r>, Jacobians::m> 
std::vector<double> Jacobians::F_b(std::array<double, n> u, std::array<double, nc> c, double t){

    std::vector<double> bv(Jacobians::b.begin(), Jacobians::b.end()); //needs to be vector to be passsed to jacad.Jacobian
    CppAD::ADFun<double> fun = F_b_fun(u, c, t);
    std::vector<double>  jacvect = fun.Jacobian(bv);

    return jacvect;// Jacobians::getJacMatrix(jacvect);
}

std::vector<double> Jacobians::R_b(std::array<double, n> u, std::array<double, nc> c, double t){

    std::vector<double> bv(Jacobians::b.begin(), Jacobians::b.end()); //needs to be vector to be passsed to jacad.Jacobian
    CppAD::ADFun<double> fun = R_b_fun(u, c, t);
    std::vector<double>  jacvect = fun.Jacobian(bv);

    return jacvect;// Jacobians::getJacMatrix(jacvect);
}


//std::array<std::array<double, Jacobians::r>, Jacobians::1> 
std::vector<double> Jacobians::J_b(std::array<double, n> u, std::array<double, nc> c, double t){

    std::vector<double> bv(Jacobians::b.begin(), Jacobians::b.end()); //needs to be vector to be passsed to jacad.Jacobian
    CppAD::ADFun<double> fun = J_b_fun(u);
    std::vector<double>  jacvect = fun.Jacobian(bv);
    
    return jacvect;// Jacobians::getJacMatrix(jacvect);
}


//std::array<std::array<double, 1>, Jacobians::n> 
std::vector<double> Jacobians::J_u(std::array<double, n> u, std::array<double, nc> c){
    std::vector<double> uv(u.begin(), u.end()); //needs to be vector to be passsed to jacad.Jacobian
    CppAD::ADFun<double> fun = J_u_fun();
    std::vector<double>  jacvect = fun.Jacobian(uv);

    return jacvect; //Jacobians::getJacMatrix1(jacvect);
}

CppAD::ADFun<double> Jacobians::F_u_fun(std::array<double, nc> c){
    CppAD::Independent(_u);              //declare independent variables and start recording operation sequence

    std::array<AD<double>, 2> velocity = {_u[2], _u[3]};
    std::array<AD<double>, 2> control_vector = {c[0], c[1]};
    AD<double> height = _u[1] + _uav.getWheelOffset();

    std::vector<AD<double>> F = _aero.getAeroForcesEarthframeVector(velocity, control_vector[0], true, height);

    AD<double> thrust = _getThrust(_u, control_vector);
    F[0] = F[0]  + thrust*cos(c[0]);  
    F[1] = F[1] - _uav.getTotalMass()*9.81 + thrust*sin(c[0]);

    CppAD::ADFun<double> f(_u, F);   // store operation sequence in f: X -> Y and stop recording (x = states/independent vars -> Y = outputs)
    return f;  
}

CppAD::ADFun<double> Jacobians::F_b_fun(std::array<double, n> u, std::array<double, nc> c, double t){
    CppAD::Independent(_b);              //declare independent variables and start recording operation sequence


    Poly <AD<double>, 4> theta_poly({_b[0], _b[1], _b[2], _b[3]});
    AD<double> theta = theta_poly.getValue(t);

    std::array<AD<double>, 2> control_vector = {theta, c[1]};

    std::array<AD<double>, 2> velocity = {u[2], u[3]};
    AD<double> height = u[1] + _uav.getWheelOffset();
    std::vector<AD<double>> F = _aero.getAeroForcesEarthframeVector(velocity, control_vector[0], true, height);
    AD<double> thrust = _getThrust(_u, control_vector);
    F[0] = F[0]  + thrust*cos(c[0]);  
    F[1] = F[1] - _uav.getTotalMass()*9.81 + thrust*sin(c[0]);

    CppAD::ADFun<double> f(_b, F);   // store operation sequence in f: X -> Y and stop recording (x = states/independent vars -> Y = outputs)
    return f;  
}


CppAD::ADFun<double> Jacobians::J_u_fun(){
    // Create state(domain) space vector

    //parameters that change each iteration:

    CppAD::Independent(_u);              //declare independent variables and start recording operation sequence

    CppAD::ADFun<double> f(_u, {_J(_u)});   // store operation sequence in f: X -> Y and stop recording (x = states/independent vars -> Y = outputs)

    return f;  
}

CppAD::ADFun<double> Jacobians::J_b_fun(std::array<double, n> u){
    // Create state(domain) space vector

    //parameters that change each iteration:

    CppAD::Independent(_b);              //declare independent variables and start recording operation sequence

    std::vector<AD<double>> uv = {u[0], u[1], u[2], u[3]};
    CppAD::ADFun<double> f(_b, {_J(uv)});   // store operation sequence in f: X -> Y and stop recording (x = states/independent vars -> Y = outputs)

    return f;  
}

CppAD::AD<double> Jacobians::_getThrust(const std::vector< AD<double> >& u, std::array<AD<double>, nc> c){
    AD<double> theta = c[0];
    AD<double> I= c[1]; //current
    // p00 = p[0], p10 = p[1], p20 = p[2], p01 = p[3], p02 = p[4], p11 = p[5] see engine map coefficients
    CppAD::AD<double> thrust = 
        + (p[0] + p[1]*sqrt(u[2]*u[2] + u[3]*u[3]) + p[2]*(u[2]*u[2] + u[3]*u[3]) + p[3]*I + p[4]*I*I + p[5]*I*sqrt(u[2]*u[2] + u[3]*u[3]) )
        ;

    return thrust;
}

// CppAD::AD<double> Jacobians::_X(const std::vector< AD<double> >& u, std::array<double, nc> c){
//     double theta = c[0];
//     double I= c[1]; //current
//     // p00 = p[0], p10 = p[1], p20 = p[2], p01 = p[3], p02 = p[4], p11 = p[5] see engine map coefficients
//     CppAD::AD<double> X1 = 
//         -0.5 * S * rho * u[2] * (cd0 + ( (-cl0 + 2 * M_PI * (theta - CppAD::atan2(u[3], u[2]))) * 
//                     (-cl0 + 2 * M_PI * (theta - CppAD::atan2(u[3], u[2]))) ) / (2 * M_PI * AR * e)) * sqrt(u[2] * u[2]  + u[3]  * u[3])
//         +0.5 * S * rho * u[3] * (-cl0 + 2 * M_PI * (theta - CppAD::atan2(u[3], u[2])) * CppAD::sqrt(u[2] * u[2]  + u[3]  * u[3]))
//         + (p[0] + p[1]*sqrt(u[2]*u[2] + u[3]*u[3]) + p[2]*(u[2]*u[2] + u[3]*u[3]) + p[3]*I + p[4]*I*I + p[5]*I*sqrt(u[2]*u[2] + u[3]*u[3]) )*cos(theta)
//         ;

//     return X1;
// }


// CppAD::AD<double> Jacobians::_Z(const std::vector< AD<double> >& u, std::array<double, nc> c){
//     double theta = c[0];
//     double I= c[1]; //current
//     CppAD::AD<double> Z = 
//     0.5 * S * rho * u[2] * (-cl0 + 2 * M_PI * (theta - atan2(u[3], u[2]))) * sqrt(u[2] * u[2] + u[3] * u[3])
//     + 0.5 * S * rho * u[3] * (cd0 + pow((-cl0 + 2 * M_PI * (theta - atan2(u[3], u[2]))), 2) / (2 * M_PI * AR * e)) * sqrt(u[2] * u[2] + u[3] * u[3])
//     - g * mass
//     + (I * I * p[4] + I * p[3] + I * p[5] * sqrt(u[2] * u[2] + u[3] * u[3]) + p[0] + p[1] * sqrt(u[2] * u[2] + u[3] * u[3]) + p[2] * (u[2] * u[2] + u[3] * u[3])) * sin(theta)
//     ;

//     return Z;
// }

CppAD::AD<double> Jacobians::_J(const std::vector< AD<double> >& u){
    return -u[3]/u[2];
    //return -(1.0 / (tf- ts)) * CppAD::atan2(u[3], u[2]);
}



void Jacobians::printJacobian(const std::vector<double>& jac, int rows_outputs, int columns_states){
    for (int row=0; row< rows_outputs; ++row){
        for (int col=0; col<columns_states;++col){
           std::cout << jac[row*columns_states+col] << ", ";
        }
        std::cout << "\n";
    }
}

void Jacobians::printJacobian(const std::array<std::array<double, n>, m>& jac){
    for (int row=0; row< Jacobians::m; ++row){
        for (int col=0; col< Jacobians::n;++col){
           std::cout << jac[row][col] << ", ";
        }
        std::cout << "\n";
    }
}

std::array<std::array<double, Jacobians::n>, Jacobians::m> Jacobians::getJacMatrix(const std::vector<double>& jac){
    std::array<std::array<double, n>, m> matrix;
    for (int row=0; row< m; ++row){
        for (int col=0; col<n;++col){
           matrix[row][col] =  jac[row*n+col];
        }
    }

    return matrix;
}

std::array<std::array<double, 1>, Jacobians::n> Jacobians::getJacMatrix1(const std::vector<double>& jac1){
        std::array<std::array<double, 1>, n> matrix;
    for (int row=0; row< m; ++row){
        for (int col=0; col<1;++col){
           matrix[row][col] =  jac1[row*n+col];
        }
    }

    return matrix;
}