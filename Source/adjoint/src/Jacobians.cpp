#include "Jacobians.h"

Jacobians::Jacobians(UAV uav, std::array<double, 6> p, std::array<double, r> b, double ts, double tf)
:p(p), b(b), ts(ts), tf(tf), _u(n){
    AR = uav.getAR();
    S = uav.getSurface();
    cl0 = uav.getCl0();
    cd0 = uav.getCd0();
    e = uav.get_e();
    mass = uav.getTotalMass();


    // Print the values to the console
    std::cout << "AR = " << AR << std::endl;
    std::cout << "S = " << S << std::endl;
    std::cout << "cl0 = " << cl0 << std::endl;
    std::cout << "cd0 = " << cd0 << std::endl;
    std::cout << "e = " << e << std::endl;
}

CppAD::ADFun<double> Jacobians::F_u(std::array<double, nc> c){
    // Create state(domain) space vector
    //std::vector< AD<double> > u(n);     // x z u w states // vector of domain space variables
    //u= {0,0,0,0};                           // value at which function is recorded

    //parameters that change each iteration:

    CppAD::Independent(_u);              //declare independent variables and start recording operation sequence
    // Create output(range) space vector
    // std::vector< AD<double> > F(m);     // (X and Z forces) // vector of ranges space variables 
    // // (Equation of X and Z forces) // Record operation that gives the output space vector
    // F[0] = _X(u);
    // F[1] = _Z(u);
    CppAD::ADFun<double> f(_u, {_X(_u, c), _Z(_u, c)});   // store operation sequence in f: X -> Y and stop recording (x = states/independent vars -> Y = outputs)

    return f;  
}

CppAD::AD<double> Jacobians::_X(const std::vector< AD<double> >& u, std::array<double, nc> c){
    double theta = c[0];
    double I= c[1]; //current
    // p00 = p[0], p10 = p[1], p20 = p[2], p01 = p[3], p02 = p[4], p11 = p[5] see engine map coefficients
    CppAD::AD<double> X1 = 
        -0.5 * S * rho * u[2] * (cd0 + ( (-cl0 + 2 * M_PI * (theta - CppAD::atan2(u[3], u[2]))) * 
                    (-cl0 + 2 * M_PI * (theta - CppAD::atan2(u[3], u[2]))) ) / (2 * M_PI * AR * e)) * sqrt(u[2] * u[2]  + u[3]  * u[3])
        +0.5 * S * rho * u[3] * (-cl0 + 2 * M_PI * (theta - CppAD::atan2(u[3], u[2])) * CppAD::sqrt(u[2] * u[2]  + u[3]  * u[3]))
        + (p[0] + p[1]*sqrt(u[2]*u[2] + u[3]*u[3]) + p[2]*(u[2]*u[2] + u[3]*u[3]) + p[3]*I + p[4]*I*I + p[5]*I*sqrt(u[2]*u[2] + u[3]*u[3]) )*cos(theta)
        ;

    return X1;
}

CppAD::ADFun<double> Jacobians::J_u(){
    // Create state(domain) space vector

    //parameters that change each iteration:

    CppAD::Independent(_u);              //declare independent variables and start recording operation sequence

    CppAD::ADFun<double> f(_u, {_J(_u)});   // store operation sequence in f: X -> Y and stop recording (x = states/independent vars -> Y = outputs)

    return f;  
}

CppAD::AD<double> Jacobians::_Z(const std::vector< AD<double> >& u, std::array<double, nc> c){
    double theta = c[0];
    double I= c[1]; //current
    CppAD::AD<double> Z = 
    0.5 * S * rho * u[2] * (-cl0 + 2 * M_PI * (theta - atan2(u[3], u[2]))) * sqrt(u[2] * u[2] + u[3] * u[3])
    + 0.5 * S * rho * u[3] * (cd0 + pow((-cl0 + 2 * M_PI * (theta - atan2(u[3], u[2]))), 2) / (2 * M_PI * AR * e)) * sqrt(u[2] * u[2] + u[3] * u[3])
    - g * mass
    + (I * I * p[4] + I * p[3] + I * p[5] * sqrt(u[2] * u[2] + u[3] * u[3]) + p[0] + p[1] * sqrt(u[2] * u[2] + u[3] * u[3]) + p[2] * (u[2] * u[2] + u[3] * u[3])) * sin(theta)
    ;

    return Z;
}

CppAD::AD<double> Jacobians::_J(const std::vector< AD<double> >& u){
    return -(1.0 / (tf- ts)) * CppAD::atan2(u[3], u[2]);
}



void Jacobians::printJacobian(const std::vector<double>& jac, int rows_outputs, int columns_states){
    for (int row=0; row< rows_outputs; ++row){
        for (int col=0; col<columns_states;++col){
           std::cout << jac[row*columns_states+col] << ", ";
        }
        std::cout << "\n";
    }
}