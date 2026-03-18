#pragma once
#ifndef NEIGHBORHOOD_H
#define NEIGHBORHOOD_H

#include "input_data.h"
#include <vector>
#include <random>
#include <functional>

double calculateTotalCost(const std::vector<Vehicle>& vehicles, const Config& config);
double euclideanDistance(int x1, int y1, int x2, int y2);

// Declare the Neighborhood class
class Neighborhood {
public:
    // Swap Neighborhood (N1)
    static void intraRouteSwap(Vehicle& vehicle, std::mt19937& gen, const Config& config);
    static void interRouteSwap(std::vector<Vehicle>& vehicles, std::mt19937& gen, const Config& config);
    static void adjacentInterRouteSwap(std::vector<Vehicle>& vehicles, std::mt19937& gen, const Config& config);
    static void nonAdjacentInterRouteSwap(std::vector<Vehicle>& vehicles, std::mt19937& gen, const Config& config);

    // Relocate Neighborhood (N2)
    static void relocateCustomer(std::vector<Vehicle>& vehicles, std::mt19937& gen, const Config& config);
    static void relocateTwoCustomers(std::vector<Vehicle>& vehicles, std::mt19937& gen, const Config& config);
    static void intraRouteRelocate(Vehicle& vehicle, std::mt19937& gen, const Config& config);
    static void intraRouteRelocateTwo(Vehicle& vehicle, std::mt19937& gen, const Config& config);


    // Segment Shift Neighborhood (N3)
    static void segmentShift(Vehicle& vehicle, std::mt19937& gen, const Config& config);

    // Remove-Insert Neighborhood (N4)
    static void removeInsert(std::vector<Vehicle>& vehicles, std::mt19937& gen, const Config& config, int& lambda, int lambda_min, int lambda_max);


    // Local Search Function
    static void twoOptInterRoute(std::vector<Vehicle>& vehicles, std::mt19937& gen, const Config& config);
    static void relocateInterRoute(std::vector<Vehicle>& vehicles, std::mt19937& gen, const Config& config);
    static void twoOptStarIntraRoute(Vehicle& vehicle, std::mt19937& gen, const Config& config);
    static void removeTwoInsertOneBestPosition(std::vector<Vehicle>& vehicles, std::mt19937& gen, const Config& config);

    ////////////////////

    static void localSearch(std::vector<Vehicle>& vehicles, std::mt19937& gen, const Config& config, int totalCustomers);


    static bool isFeasible(Vehicle& vehicle, const Config& config);
    static bool isTemporallyFeasibleSavelsbergh(Vehicle& vehicle, const Config& config);

};

#endif // NEIGHBORHOOD_H
