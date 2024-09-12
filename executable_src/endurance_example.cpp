#include <iostream>

#include "header_files_catalog.h"
int main(){
    std::cout << "Manuver simulation 1\n"; 
    std::cout << "==================================\n\n\n";

    double x_endurance = 5000;
    double ec_specific = 0.5;
    double m_payload = 4;
    double payload_volume = 5;

    ManeuverScores reference_maneuver_scores;
    EnduranceScores reference_endurance_scores(5000, 0.5, 3, 5);
    EnduranceScores competitor_endurance_scores(x_endurance, ec_specific, m_payload, payload_volume);

    Scoring scoring(reference_maneuver_scores, reference_endurance_scores);

    std::cout << "Reference Endurance Scores:\n" << reference_endurance_scores << std::endl;
    std::cout << "\nCompetitor Endurance Scores:\n" << competitor_endurance_scores << std::endl;
    std::cout << "\n\n\nEndurance Score: = " << scoring.enduranceScore(competitor_endurance_scores);

    return 0;
}