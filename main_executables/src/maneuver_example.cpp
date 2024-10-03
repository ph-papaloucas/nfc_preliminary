#include <iostream>

#include "header_files_catalog.h"
int main(){
    std::cout << "Manuver simulation 1\n"; 
    std::cout << "==================================\n\n\n";

    double x_takeoff = 0;
    double a_climb = Aerodynamics::deg2rad(90) ;
    double vmax = 20;
    double vmin = 5;
    double glide_ratio = 4;
    double a_landing = Aerodynamics::deg2rad(90);
    double x_landing = 30;

    ManeuverScores reference_maneuver_scores(40, Aerodynamics::deg2rad(45), 5, 20, 10, 45, 20);
    EnduranceScores reference_endurance_scores;
    ManeuverScores competitor_maneuver_scores(x_takeoff, a_climb, glide_ratio, vmax, vmin, a_landing, x_landing);
    //ManeuverScores competitor_maneuver_scores = reference_maneuver_scores;
    Scoring scoring(reference_maneuver_scores, reference_endurance_scores);

    std::cout << "Reference Maneuver Scores:\n" << reference_maneuver_scores << std::endl;
    std::cout << "\nCompetitor Maneuver Scores:\n" << competitor_maneuver_scores << std::endl;
    std::cout << "\n\n\nManeuver Score: = " << scoring.maneuverScore(competitor_maneuver_scores);

    return 0;
}