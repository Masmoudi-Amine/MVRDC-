#include "input_data.h"
#include "constructive.h"
#include "neighborhoods.h"
#include "VehicleOptimizer.h"
#include <iostream>
#include <random>
#include <vector>
#include <algorithm>
#include <functional>
#include <fstream>
#include <chrono>
#include <cmath>
#include <numeric>

//Algorithm parameters - CHANGED TO SINGLE ITERATION
const int n_MS_VNS = 100000;
const int n_it = 5;
const double alpha = 0.99;
const double pi1 = 1.0;
const double pi2 = 0.0;
const double pi3 = 5.0;

// Function to print detailed route information including time windows and service times
void printRouteDetails(const Config& config, const std::vector<Vehicle>& vehicles) {
    std::ofstream routeFile("route_details.txt");
    if (!routeFile.is_open()) {
        std::cerr << "Error: Could not create route details file\n";
        return;
    }

    routeFile << "=== DETAILED ROUTE ANALYSIS ===\n\n";

    for (const auto& v : vehicles) {
        if (v.route.empty()) continue;

        routeFile << "Vehicle " << v.id << " (" << v.type << ")\n";
        routeFile << "Route: Depot(0)";

        double currentTime = 0.0;
        bool feasible = true;

        // Process each customer in the route
        for (size_t i = 0; i < v.route.size(); ++i) {
            const auto& rp = v.route[i];
            const auto& customer = rp.customer;

            // Calculate travel time
            double travelTime = 0.0;
            if (i == 0) {
                // From depot to first customer
                travelTime = euclideanDistance(config.depot_x, config.depot_y,
                    customer.x, customer.y);
            }
            else {
                // From previous customer to current customer
                const auto& prevCustomer = v.route[i - 1].customer;
                travelTime = euclideanDistance(prevCustomer.x, prevCustomer.y,
                    customer.x, customer.y);
            }

            currentTime += travelTime;

            // Check if arrived before ready time (need to wait)
            double arrivalTime = currentTime;
            double waitTime = 0.0;
            if (arrivalTime < customer.ready) {
                waitTime = customer.ready - arrivalTime;
                currentTime = customer.ready;
            }

            double serviceStart = currentTime;
            double serviceEnd = serviceStart + customer.service;
            currentTime = serviceEnd;

            // Check time window feasibility
            bool twFeasible = (arrivalTime <= customer.due);
            if (!twFeasible) feasible = false;

            routeFile << " -> Customer " << customer.id << "\n";
            routeFile << "   Travel Time: " << travelTime << "\n";
            routeFile << "   Arrival: " << arrivalTime << " (Ready: " << customer.ready
                << ", Due: " << customer.due << ") "
                << (twFeasible ? "?" : "? VIOLATION") << "\n";
            routeFile << "   Wait Time: " << waitTime << "\n";
            routeFile << "   Service: " << customer.service << " units ("
                << serviceStart << " to " << serviceEnd << ")\n";
        }

        // Return to depot
        if (!v.route.empty()) {
            const auto& lastCustomer = v.route.back().customer;
            double returnTime = euclideanDistance(lastCustomer.x, lastCustomer.y,
                config.depot_x, config.depot_y);
            double returnArrival = currentTime + returnTime;

            routeFile << " -> Depot(0)\n";
            routeFile << "   Return Travel Time: " << returnTime << "\n";
            routeFile << "   Return to Depot: " << returnArrival << "\n";

            // Check route duration constraint
            bool durationFeasible = (returnArrival <= config.max_route_duration);
            if (!durationFeasible) feasible = false;
            routeFile << "   Max Route Duration: " << config.max_route_duration
                << " " << (durationFeasible ? "?" : "? VIOLATION") << "\n";
        }

        // Check capacity constraints
        int totalFresh = 0, totalFrozen = 0;
        for (const auto& rp : v.route) {
            totalFresh += rp.customer.fresh;
            totalFrozen += rp.customer.frozen;
        }

        bool capacityFeasible = true;
        if (v.type == "Fresh") {
            capacityFeasible = (totalFresh <= v.fresh_cap);
        }
        else if (v.type == "Frozen") {
            capacityFeasible = (totalFrozen <= v.frozen_cap);
        }
        else if (v.type == "Mixed") {
            capacityFeasible = (totalFresh <= v.fresh_cap && totalFrozen <= v.frozen_cap);
        }

        routeFile << "   Capacity: Fresh=" << totalFresh << "/" << v.fresh_cap
            << ", Frozen=" << totalFrozen << "/" << v.frozen_cap
            << " " << (capacityFeasible ? "?" : "? VIOLATION") << "\n";

        routeFile << "   ROUTE FEASIBILITY: " << (feasible ? "FEASIBLE ?" : "INFEASIBLE ?") << "\n";
        routeFile << "   Total Route Time: " << currentTime << "\n\n";
    }

    // Overall feasibility check
    routeFile << "=== OVERALL FEASIBILITY CHECK ===\n";

    // Check if all customers are served
    int totalCustomers = 0;
    int servedCustomers = 0;
    std::vector<int> unservedCustomers;

    // This would require access to the original customers list
    // You might need to modify this part based on your data structure

    routeFile.close();
    std::cout << "Route details saved to 'route_details.txt'\n";
}


bool isSolutionFeasible(const Config& config, const std::vector<Vehicle>& vehicles) {
    int totalFrigoUsed = 0;

    for (const Vehicle& v : vehicles) {
        if (v.route.empty()) continue;

        int totalFresh = 0, totalFrozen = 0;
        for (const auto& rp : v.route) {
            totalFresh += rp.customer.fresh;
            totalFrozen += rp.customer.frozen;
        }

        if (v.type == "Fresh") {
            int requiredFrigo = ceil(static_cast<double>(totalFrozen) / config.frigobox_cap);
            if (requiredFrigo > v.max_frigo) return false;
            totalFrigoUsed += requiredFrigo;
        }
        else if (v.type == "Frozen") {
            int requiredFrigo = ceil(static_cast<double>(totalFresh) / config.frigobox_cap);
            if (requiredFrigo > v.max_frigo) return false;
            totalFrigoUsed += requiredFrigo;
        }
    }
    return (totalFrigoUsed <= config.total_frigo);
}

void variableNeighborhoodSearch(
    Config& config,
    std::vector<Customer>& customers,
    std::vector<Vehicle>& vehicles,
    std::mt19937& gen,
    const std::chrono::high_resolution_clock::time_point& start_time
) {
    std::ofstream logFile("vns_log.txt");


    int totalCustomers = customers.size() - 1;

    // Preserve original problem state
    const Config originalConfig = config;
    const std::vector<Vehicle> originalVehicles = vehicles;
    std::vector<Customer> originalCustomers = customers;

    int lambda_min = std::max(5, static_cast<int>(0.1 * totalCustomers));
    int lambda_max = std::min(100, static_cast<int>(0.3 * totalCustomers));
    int lambda = lambda_min;

    // Define neighborhood groups
    std::vector<std::vector<std::function<void(std::vector<Vehicle>&, std::mt19937&)>>> neighborhoodGroups = {
        // Group 0: Swap (N1)
        {
            [&](auto& v, auto& g) {
                if (!v.empty()) {
                    std::uniform_int_distribution<> veh_dist(0, v.size() - 1);
                    Neighborhood::intraRouteSwap(v[veh_dist(g)], g, config);
                }
            },
            [&](auto& v, auto& g) { Neighborhood::interRouteSwap(v, g, config); },
            [&](auto& v, auto& g) { Neighborhood::adjacentInterRouteSwap(v, g, config); },
            [&](auto& v, auto& g) { Neighborhood::nonAdjacentInterRouteSwap(v, g, config); }

        },
        // Group 1: Relocate (N2)
       {
            [&](auto& v, auto& g) { Neighborhood::relocateCustomer(v, g, config); },
            [&](auto& v, auto& g) { Neighborhood::relocateTwoCustomers(v, g, config); },
            [&](auto& v, auto& g) {
                if (!v.empty()) {
                    std::uniform_int_distribution<> veh_dist(0, v.size() - 1);
                    Neighborhood::intraRouteRelocate(v[veh_dist(g)], g, config);
                }
            },

        //// Add intraRouteRelocateTwo here
            [&](auto& v, auto& g) {
                if (!v.empty()) {
                    std::uniform_int_distribution<> veh_dist(0, v.size() - 1);
                    Neighborhood::intraRouteRelocateTwo(v[veh_dist(g)], g, config);
                }
            }

       },


        // Group 2: Segment Shift (N3)
        {
            [&](auto& v, auto& g) {
                if (!v.empty()) {
                    std::uniform_int_distribution<> veh_dist(0, v.size() - 1);
                    Neighborhood::segmentShift(v[veh_dist(g)], g, config);
                }
            }
        },


        //// Group 3: Remove-Insert (N4)
        {
             [&](auto& v, auto& g) {
                Neighborhood::removeInsert(v, g, config, lambda, lambda_min, lambda_max);

             }

        }

    };

    std::vector<std::string> groupNames = {
        "Swap (N1)", "Relocate (N2)", "Segment Shift (N3)", "Remove-Insert (N4)"
    };

    std::vector<double> groupScores(neighborhoodGroups.size(), 0.0);
    double bestOverallCost = std::numeric_limits<double>::max();
    std::vector<Vehicle> bestOverallSolution;
    std::chrono::duration<double> best_time = std::chrono::duration<double>::max();

    for (int ms_iter = 0; ms_iter < n_MS_VNS; ++ms_iter) {
        // Multi-start initialization
        config = originalConfig;
        vehicles = originalVehicles;
        customers = originalCustomers;
        for (auto& c : customers) c.served = false;
        constructSolution(config, customers, vehicles);

        double currentCost = calculateTotalCost(vehicles, config);
        std::vector<Vehicle> currentSolution = vehicles;
        std::vector<double> iterationScores(neighborhoodGroups.size(), 0.0);
        if (ms_iter == 0) {
            bestOverallCost = currentCost;
            bestOverallSolution = currentSolution;
        }
        //// Log iteration header
        logFile << "\n========== MS-VNS Iteration " << ms_iter << " ==========\n"
            << "Initial Solution Cost: " << currentCost << "\n";

        // Temperature initialization
        double T = (-0.05 / std::log(0.5)) * currentCost;
        std::uniform_real_distribution<> probDist(0.0, 1.0);

        // Neighborhood exploration
        size_t h = 0;
        while (h < neighborhoodGroups.size()) {
            int attempts = 0;
            bool improved = false;

            while (attempts < n_it && !improved) {
                // Shaking phase
                std::vector<Vehicle> shakenSolution = currentSolution;
                auto& currentGroup = neighborhoodGroups[h];
                if (!currentGroup.empty()) {
                    std::uniform_int_distribution<> op_dist(0, currentGroup.size() - 1);
                    currentGroup[op_dist(gen)](shakenSolution, gen);
                }


                if (!isSolutionFeasible(config, shakenSolution)) {
                    continue; // Skip acceptance
                }

                // --- NEW CODE: Vehicle Assignment using VehicleOptimizer ---
              // --- NEW CODE: Vehicle Assignment using VehicleOptimizer ---
                VehicleOptimizer optimizer(config);
                auto routeDemands = VehicleOptimizer::calculateRouteDemands(shakenSolution);
                auto candidateSets = optimizer.vehicleCapabilityAnalysis(routeDemands, originalVehicles);


                auto assignments = optimizer.selectBestVehicle(candidateSets, gen);

                if (!assignments) {
                    // Skip invalid solution
                    attempts++;
                    continue;
                }

                // Update shakenSolution with assigned vehicles
                std::vector<Vehicle> newShakenVehicles;
                std::vector<Vehicle> nonEmptyRoutes;
                for (const auto& v : shakenSolution) {
                    if (!v.route.empty()) {
                        nonEmptyRoutes.push_back(v);
                    }
                }

                for (size_t i = 0; i < assignments->size(); ++i) {
                    const auto& assignment = (*assignments)[i];
                    auto it = std::find_if(originalVehicles.begin(), originalVehicles.end(),
                        [&assignment](const Vehicle& v) { return v.id == assignment.vehicle_id; });
                    if (it != originalVehicles.end()) {
                        Vehicle assignedVehicle = *it;
                        assignedVehicle.route = nonEmptyRoutes[i].route;

                        newShakenVehicles.push_back(assignedVehicle);
                    }
                    else {
                        logFile << "[ERROR] Vehicle " << assignment.vehicle_id << " not found.\n";
                        continue; // Skip invalid assignment
                    }
                }
                shakenSolution = newShakenVehicles;
                // --- END OF NEW CODE ---

                // Direct evaluation
                double shakenCost = calculateTotalCost(shakenSolution, config);
                double deltaE = shakenCost - currentCost;

                if (shakenCost < currentCost || probDist(gen) < (T / (T * T + deltaE * deltaE))) {
                    std::vector<Vehicle> improvedSolution = shakenSolution;
                    Neighborhood::localSearch(improvedSolution, gen, config, totalCustomers);
                    double improvedCost = calculateTotalCost(improvedSolution, config);

                    if (!isSolutionFeasible(config, improvedSolution)) {
                        continue; // Skip acceptance
                    }

                    // Calculate vehicle counts
                    int improvedVehicleCount = std::count_if(improvedSolution.begin(), improvedSolution.end(),
                        [](const Vehicle& v) { return !v.route.empty(); });
                    int currentVehicleCount = std::count_if(currentSolution.begin(), currentSolution.end(),
                        [](const Vehicle& v) { return !v.route.empty(); });

                    double deltaImproved = improvedCost - currentCost;
                    double acceptanceProb = T / (T * T + deltaImproved * deltaImproved);

                    if (improvedCost < currentCost || probDist(gen) < acceptanceProb || improvedVehicleCount < currentVehicleCount) {
                        currentSolution = improvedSolution;
                        currentCost = improvedCost;
                        //improved = true;

                        // Update best solution considering vehicle count
                        int currentVehicleCount = std::count_if(currentSolution.begin(), currentSolution.end(),
                            [](const Vehicle& v) { return !v.route.empty(); });

                        int bestVehicleCount = std::count_if(bestOverallSolution.begin(), bestOverallSolution.end(),
                            [](const Vehicle& v) { return !v.route.empty(); });

                        if (currentCost < bestOverallCost) {
                            auto now = std::chrono::high_resolution_clock::now();
                            best_time = now - start_time;

                            bestOverallCost = currentCost;
                            bestOverallSolution = currentSolution;
                            groupScores[h] += pi1;
                            improved = true;

                            logFile << "[UPDATE] New Best Cost: " << bestOverallCost
                                << " | Found at: " << best_time.count() << " seconds\n"
                                << " NBR OF VEHICLE : " << bestVehicleCount << " vehicles\n";

                        }
                        else {
                            groupScores[h] += pi2;
                        }
                    }
                }

                if (improved) {
                    T *= alpha;
                    lambda = lambda_min;
                    logFile << "MS-VNS Iteration " << ms_iter
                        << " | Neighborhood " << h
                        << " | Attempt " << attempts
                        << " | Cost: " << currentCost
                        << " | Temp: " << T << "\n";
                }
                else {
                    groupScores[h] += pi3;
                    lambda = std::min(lambda + 1, lambda_max);
                }

                attempts++;
            }

            h++;     // Move to next neighborhood
        }

        // Adaptive reordering every 100 iterations
        if (ms_iter % 100 == 0) {
            std::vector<size_t> indices(neighborhoodGroups.size());
            std::iota(indices.begin(), indices.end(), 0);
            std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
                return groupScores[a] > groupScores[b];
                });

            // Rebuild neighborhood structures
            auto oldGroups = neighborhoodGroups;
            auto oldNames = groupNames;
            auto oldScores = groupScores;

            logFile << "Best Cost So Far: " << bestOverallCost << "\n"
                << "---------- Neighborhood Scores ----------\n";
            for (size_t i = 0; i < neighborhoodGroups.size(); ++i) {
                logFile << i << ". " << groupNames[i] << ": " << groupScores[i] << "\n";
            }
            logFile << "========================================\n";

            for (size_t i = 0; i < indices.size(); ++i) {
                neighborhoodGroups[i] = oldGroups[indices[i]];
                groupNames[i] = oldNames[indices[i]];
                groupScores[i] = oldScores[indices[i]];
            }
            std::fill(groupScores.begin(), groupScores.end(), 0.0);
        }
    }

    vehicles = bestOverallSolution;
    logFile << "========== FINAL BEST SOLUTION ==========\n";
    logFile << "Best Cost: " << bestOverallCost << "\n";
    std::cout << "Best Cost: " << bestOverallCost << "\n";

    // Optional: If you've saved best_time in a variable:
    logFile << "Time to Best Solution: " << best_time.count() << " seconds\n";

    logFile.close();
}

int main() {
    std::ofstream resultFile("final_result.txt");
    if (!resultFile.is_open()) {
        std::cerr << "Error: Could not create output file 'final_result.txt'\n";
        return 1;
    }

    try {
        // Check if the input file exists and can be opened
        std::ifstream testFile("input.txt");
        if (!testFile.is_open()) {
            std::cerr << "Error: Could not open input file 'input.txt'. Please check the file name and location.\n";
            return 1;
        }
        testFile.close(); // Close after checking

        Config config;
        std::vector<Customer> customers;
        std::vector<Vehicle> vehicles;
        read_input("input.txt", config, customers, vehicles);

        std::random_device rd;
        std::mt19937 gen(rd());

        auto start = std::chrono::high_resolution_clock::now();
        //constructSolution(config, customers, vehicles);

        variableNeighborhoodSearch(config, customers, vehicles, gen, start);
        auto end = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> duration = end - start;
        std::cout << "Optimization complete!\nRuntime: "
            << duration.count() << " seconds\n";

        // Post-process vehicle stats before printing
        for (auto& v : vehicles) {
            int totalFresh = 0, totalFrozen = 0;
            for (const auto& rp : v.route) {
                totalFresh += rp.customer.fresh;
                totalFrozen += rp.customer.frozen;
            }
            v.current_fresh = totalFresh;
            v.current_frozen = totalFrozen;

            if (v.type == "Fresh") {
                v.frigoboxes = ceil(static_cast<double>(totalFrozen) / config.frigobox_cap);
            }
            else if (v.type == "Frozen") {
                v.frigoboxes = ceil(static_cast<double>(totalFresh) / config.frigobox_cap);
            }
            else {
                v.frigoboxes = 0;
            }
        }

        //print_solution(config, customers, vehicles);

         // === Solution Summary ===
        // Print detailed route information for feasibility verification
        printRouteDetails(config, vehicles);

        int served = 0;
        int totalDistance = 0.0;
        resultFile << "=== Solution Summary ===\n";

        for (const auto& c : customers) {
            if (c.id != 0 && c.served) served++;
        }
        resultFile << "Customers served: " << served << "/" << (customers.size() - 1) << "\n";

        for (const auto& v : vehicles) {
            if (v.route.empty()) continue;

            double routeDist = 0.0;

            resultFile << "Vehicle " << v.id << " (" << v.type << ")\nRoute: 0";
            for (const auto& rp : v.route) {
                resultFile << " -> " << rp.customer.id;

            }
            resultFile << " -> 0\n";

            // Compute route distance
            if (!v.route.empty()) {
                routeDist += euclideanDistance(config.depot_x, config.depot_y, v.route[0].customer.x, v.route[0].customer.y);
                for (size_t i = 1; i < v.route.size(); ++i) {
                    routeDist += euclideanDistance(v.route[i - 1].customer.x, v.route[i - 1].customer.y,
                        v.route[i].customer.x, v.route[i].customer.y);
                }
                routeDist += euclideanDistance(v.route.back().customer.x, v.route.back().customer.y,
                    config.depot_x, config.depot_y);
            }

            // === Route Duration Calculation ===
            double max_route_duration_used = 0.0;

            for (const auto& v : vehicles) {
                if (v.route.empty()) continue;

                double currentTime = 0.0;
                for (size_t i = 0; i < v.route.size(); ++i) {
                    const auto& rp = v.route[i];
                    double travelTime = (i == 0)
                        ? euclideanDistance(config.depot_x, config.depot_y, rp.customer.x, rp.customer.y)
                        : euclideanDistance(v.route[i - 1].customer.x, v.route[i - 1].customer.y,
                            rp.customer.x, rp.customer.y);

                    currentTime += travelTime;
                    currentTime = std::max(currentTime, static_cast<double>(rp.customer.ready));
                    currentTime += rp.customer.service;
                }

                if (!v.route.empty()) {
                    double returnTime = euclideanDistance(v.route.back().customer.x,
                        v.route.back().customer.y,
                        config.depot_x, config.depot_y);
                    currentTime += returnTime;
                }

                if (currentTime > max_route_duration_used) {
                    max_route_duration_used = currentTime;
                }
            }

            resultFile << "Distance: " << routeDist << "\n";
            resultFile << "Fresh: " << v.current_fresh << "/" << v.fresh_cap
                << " | Frozen: " << v.current_frozen << "/" << v.frozen_cap << "\n";
            resultFile << "Frigoboxes used: " << v.frigoboxes << "\n\n";

            totalDistance += routeDist;
        }

        resultFile << "Total distance: " << totalDistance << "\n";

        int totalFrigo = 0;
        int freshVehicles = 0, frozenVehicles = 0, mixedVehicles = 0;

        for (const auto& v : vehicles) {
            if (!v.route.empty()) {
                totalFrigo += v.frigoboxes;
                if (v.type == "Fresh") freshVehicles++;
                else if (v.type == "Frozen") frozenVehicles++;
                else if (v.type == "Mixed") mixedVehicles++;
            }
        }

        resultFile << "\n=== Summary ===\n";
        resultFile << "Total Frigoboxes Used: " << totalFrigo << "\n";
        resultFile << "Fresh Vehicles Used: " << freshVehicles << "\n";
        resultFile << "Frozen Vehicles Used: " << frozenVehicles << "\n";
        resultFile << "Mixed Vehicles Used: " << mixedVehicles << "\n";

        // Unserved customers
        resultFile << "Unserved customers:";
        for (const auto& c : customers) {
            if (c.id != 0 && !c.served) {
                std::cout << " " << c.id;
            }
        }
        resultFile << "\n";
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    resultFile.close();

    return 0;
}