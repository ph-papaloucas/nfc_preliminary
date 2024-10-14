#include "State.h"

AeroState::AeroState(double angle_of_attack, double velocity, double cl): 
    angle_of_attack(angle_of_attack), velocity(velocity), cl(cl){};

ControlState::ControlState():theta(0e0), thrust(0e0){};
ControlState::ControlState(double theta, double thrust): theta(theta), thrust(thrust){};
ControlState::ControlState(std::array<double, 2> control_state): theta(control_state[0]), thrust(control_state[1]){};



State::State(): x(0), z(0), u(0), w(0), Fx(0), Fz(0){};
State::State(double x, double z, double u, double w, double Fx, double Fz): x(x), z(z), u(u), w(w), Fx(Fx), Fz(Fz){};
State::State(std::array<double, 4> x, std::array<double, 2> u): x(x[0]), z(x[1]), u(x[2]), w(x[3]), Fx(u[0]), Fz(u[1]){};


void State::setState(double x, double z, double u, double w){
    this->x = x;
    this->z = z;
    this->u = u;
    this->w = w;

}
void State::setForces(double Fx, double Fz){
    this->Fx = Fx;
    this->Fz = Fz;
}


std::array<double,4 > State::getStatesArray(){
    return {x, z, u, w};
}
std::array<double,2 > State::getForcesArray(){
    return {Fx, Fz};
}

StateHistory::StateHistory() {};

void StateHistory::prepareHistoryMatrix(int total_timesteps, double t0, const State& initial_state, const ControlState& control_state){
    _history.reserve(total_timesteps + 1);
    _control_history.reserve(total_timesteps + 1);
    _time.reserve(total_timesteps + 1);

    appendState(t0, initial_state, control_state);
}


void StateHistory::appendState(double t, const State& state, const ControlState& control_state){
    _history.emplace_back(state);
    _control_history.emplace_back(control_state);
    _time.emplace_back(t);

}


std::ostream& operator<<(std::ostream& out, const StateHistory& data){
        out << std::fixed << std::setprecision(6); // Set fixed-point notation and precision to 2 decimal places
        out << "time     x        z        u        w     Fx       Fz     theta    thrust\n"; 
        for (int i=0;i < data._time.size(); ++i){
            out     <<data._time[i] << ' '
                    << data._history[i].x << ' ' 
                    << data._history[i].z << ' ' 
                    << data._history[i].u << ' ' 
                    << data._history[i].w << ' '
                    << data._history[i].Fx << ' '
                    << data._history[i].Fz << ' '
                    << data._control_history[i].theta << ' '
                    << data._control_history[i].thrust << "\n";
        }
    
        return out;
}

void StateHistory::printToCsv(const std::string& filename){
 // Open the file in output mode
    std::ofstream file(filename + ".csv");

    // Check if the file is successfully opened
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + filename);
    }

    // Set the precision for floating-point values
    file << std::fixed << std::setprecision(6);

    // Write the CSV header
    file << "time,x,z,u,w,Fx,Fz,theta,thrust\n";

    // Write the data to the file in CSV format
    for (int i = 0; i < _time.size(); ++i) {
        file    << _time[i] << ','
                << _history[i].x << ','
                << _history[i].z << ','
                << _history[i].u << ','
                << _history[i].w << ','
                << _history[i].Fx << ','
                << _history[i].Fz << ','
                << _control_history[i].theta << ','
                << _control_history[i].thrust << '\n';
    }

    // Close the file

    file.close();

}