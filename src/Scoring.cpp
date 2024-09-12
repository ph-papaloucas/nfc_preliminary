#include "Scoring.h"

const double ManeuverScores::w_x_takeoff = -4;
const double ManeuverScores::w_a_climb = 7;
const double ManeuverScores::w_glide_ratio = 25;
const double ManeuverScores::w_v_max = 4;
const double ManeuverScores::w_v_min = -4;
const double ManeuverScores::w_a_landing = 5;
const double ManeuverScores::w_x_landing = -3;

const double EnduranceScores::w_x_endurance_flight = 8;
const double EnduranceScores::w_ec_specific = -12;
const double EnduranceScores::w_mass_payload = 10;
const double EnduranceScores::w_volume_payload = 18;

Scoring::Scoring(const ManeuverScores& reference_maneuver_scores, const EnduranceScores& reference_endurance_scores)
:_reference_maneuver_scores(reference_maneuver_scores), _reference_endurance_scores(reference_endurance_scores){};

double Scoring::maneuverScore(const ManeuverScores& maneuver_scores) {
    const ManeuverScores& s = maneuver_scores;              //these are just for easier notation
    const ManeuverScores& r = _reference_maneuver_scores;   //these are just for easier notation


    double Smaneuver = 
        ManeuverScores::w_x_takeoff * tanh(s.x_takeoff / r.x_takeoff - 1) +
        ManeuverScores::w_a_climb * tanh(s.a_climb / r.a_climb - 1) +
        ManeuverScores::w_glide_ratio * tanh(s.glide_ratio / r.glide_ratio - 1) +
        ManeuverScores::w_v_max * tanh(s.v_max / r.v_max - 1) +
        ManeuverScores::w_v_min * tanh(s.v_min / r.v_min - 1) +
        ManeuverScores::w_a_landing * tanh(s.a_landing / r.a_landing - 1) +
        ManeuverScores::w_x_landing * tanh(s.x_landing / r.x_landing - 1);

    return Smaneuver;
}

double Scoring::enduranceScore(const EnduranceScores& endurance_scores) {
    // Use references for easier notation
    const EnduranceScores& e = endurance_scores;
    const EnduranceScores& r = _reference_endurance_scores;  // Assuming you have a reference for comparison

    double Sendurance = 
        EnduranceScores::w_x_endurance_flight * tanh(e.x_endurance_flight / r.x_endurance_flight - 1) +
        EnduranceScores::w_ec_specific * tanh(e.ec_specific / r.ec_specific - 1) +
        EnduranceScores::w_mass_payload * tanh(e.mass_payload / r.mass_payload - 1) +
        EnduranceScores::w_volume_payload * tanh(e.volume_payload / r.volume_payload - 1);

    return Sendurance;
}
