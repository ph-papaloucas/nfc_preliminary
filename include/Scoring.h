#pragma once
#include <cmath>
#include <iostream>

class ManeuverScores{
public:
    ManeuverScores()
        : x_takeoff(1e10), a_climb(0), glide_ratio(0.0), v_max(0.0), v_min(1e10), a_landing(0.0), x_landing(1e10) {}

    ManeuverScores
    (double x_takeoff, double a_climb, double glide_ratio, double v_max, double v_min, double a_landing, double x_landing)
        : x_takeoff(x_takeoff), a_climb(a_climb), glide_ratio(glide_ratio), v_max(v_max), v_min(v_min), a_landing(a_landing), x_landing(x_landing) {}

    double x_takeoff, a_climb, glide_ratio, v_max, v_min, a_landing, x_landing;


    static const double w_x_takeoff;
    static const double w_a_climb;
    static const double w_glide_ratio;
    static const double w_v_max;
    static const double w_v_min;
    static const double w_a_landing;
    static const double w_x_landing;

    //overload std::cout function
    friend std::ostream& operator<<(std::ostream& os, const ManeuverScores& ms) {
            os << "x_takeoff: " << ms.x_takeoff << "\n"
       << "a_climb: " << ms.a_climb << "\n"
       << "glide_ratio: " << ms.glide_ratio << "\n"
       << "v_max: " << ms.v_max << "\n"
       << "v_min: " << ms.v_min << "\n"
       << "a_landing: " << ms.a_landing << "\n"
       << "x_landing: " << ms.x_landing;
    return os;
    }
};

class EnduranceScores {
public:
    // Default constructor
    EnduranceScores()
        : x_endurance_flight(0), ec_specific(1e10), mass_payload(0.0), volume_payload(0) {}

    // Parameterized constructor
    EnduranceScores(double x_endurance_flight, double ec_specific, double mass_payload, double volume_payload)
        : x_endurance_flight(x_endurance_flight), ec_specific(ec_specific), mass_payload(mass_payload), volume_payload(volume_payload) {}

    double x_endurance_flight, ec_specific, mass_payload, volume_payload;

    // Static const member variables
    static const double w_x_endurance_flight;
    static const double w_ec_specific;
    static const double w_mass_payload;
    static const double w_volume_payload;
    
     //overload std::cout function
    friend std::ostream& operator<<(std::ostream& os, const EnduranceScores& es) {
        os << "x_endurance_flight: " << es.x_endurance_flight<< "\n"
       << "ec_specific: " << es.ec_specific << "\n"
       << "mass_payload: " << es.mass_payload << "\n"
       << "volume_payload: " << es.volume_payload << "\n";
    return os;
    }
};

class Scoring{
public:
    Scoring(const ManeuverScores& reference_maneuver_scores, const EnduranceScores& reference_endurance_scores);

    double maneuverScore(const ManeuverScores& maneuver_score);
    double enduranceScore(const EnduranceScores& endurance_scores);

private:
ManeuverScores _reference_maneuver_scores;
EnduranceScores _reference_endurance_scores;

};