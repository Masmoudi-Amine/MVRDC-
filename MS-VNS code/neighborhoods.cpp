#include "neighborhoods.h"
#include <cmath>
#include <algorithm>
#include <random>
#include <functional>
#include <numeric>
#include <iostream>


using namespace std;

// Helper function to calculate Euclidean distance
double euclideanDistance(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// Calculate total cost of all routes
//double calculateTotalCost(const vector<Vehicle>& vehicles, const Config& config) {
//    double totalCost = 0.0;
//    double total_demand = 0.0;
//    double total_empty = 0.0;
//
//    for (const auto& vehicle : vehicles) {
//        if (vehicle.route.empty()) continue;
//
//        // From depot to first customer
//        totalCost += euclideanDistance(config.depot_x, config.depot_y,
//            vehicle.route[0].customer.x, vehicle.route[0].customer.y);
//
//        // Between customers
//        for (size_t i = 1; i < vehicle.route.size(); ++i) {
//            totalCost += euclideanDistance(vehicle.route[i - 1].customer.x, vehicle.route[i - 1].customer.y,
//                vehicle.route[i].customer.x, vehicle.route[i].customer.y);
//        }
//
//        // Back to depot
//        totalCost += euclideanDistance(vehicle.route.back().customer.x, vehicle.route.back().customer.y,
//            config.depot_x, config.depot_y);
//
//
//
//
//
//    }
//    return totalCost;
//}
//


double calculateTotalCost(const vector<Vehicle>& vehicles, const Config& config) {
    double totalCost = 0.0;
    double total_demand = 0.0;
    double total_empty = 0.0;
    int vehiclesUsed = 0;

    for (const auto& vehicle : vehicles) {
        // Skip empty vehicles entirely (no penalty)
        if (vehicle.route.empty()) continue;  // <-- Critical change here
        vehiclesUsed++; // <-- Count active vehicle

        // Distance cost calculation (for non-empty vehicles)
        totalCost += euclideanDistance(config.depot_x, config.depot_y,
            vehicle.route[0].customer.x, vehicle.route[0].customer.y);

        for (size_t i = 1; i < vehicle.route.size(); ++i) {
            totalCost += euclideanDistance(vehicle.route[i - 1].customer.x,
                vehicle.route[i - 1].customer.y, vehicle.route[i].customer.x,
                vehicle.route[i].customer.y);
        }



        totalCost += euclideanDistance(vehicle.route.back().customer.x,
            vehicle.route.back().customer.y, config.depot_x, config.depot_y);

        // Calculate route-specific demands
        int vehicleFresh = 0, vehicleFrozen = 0;
        for (const auto& rp : vehicle.route) {
            vehicleFresh += rp.customer.fresh;
            vehicleFrozen += rp.customer.frozen;
        }
        total_demand += vehicleFresh + vehicleFrozen;

        //// Empty space penalty (only for non-empty vehicles)
        if (vehicle.type == "Fresh") {
            int used_fresh_space = vehicleFresh + vehicleFrozen;
            total_empty += std::max(0, vehicle.fresh_cap - used_fresh_space);

        }
        else if (vehicle.type == "Frozen") {
            int used_frozen_space = vehicleFrozen + vehicleFresh;
            total_empty += std::max(0, vehicle.frozen_cap - used_frozen_space);

        }
        else if (vehicle.type == "Mixed") {
            total_empty += (vehicle.fresh_cap - vehicleFresh) + (vehicle.frozen_cap - vehicleFrozen);
        }
    }

    //// Penalize empty space only for used vehicles
    double penalty = (total_demand > 0) ? (total_empty / total_demand) : 0.0;
    totalCost += penalty;
    double vehiclePenalty = vehiclesUsed * 1000;
    totalCost += vehiclePenalty;

    return totalCost;
}



bool Neighborhood::isTemporallyFeasibleSavelsbergh(Vehicle& vehicle, const Config& config) {
    const auto& route = vehicle.route;
    int n = route.size();
    if (n == 0) return true;

    // Initialize vectors if needed
    vehicle.start_times.resize(n);
    vehicle.waiting_times.resize(n);
    const double EPS = 1e-9; // Add this for floating point comparisons

    // Forward pass
    double travel_time_to_first = euclideanDistance(config.depot_x, config.depot_y, route[0].customer.x, route[0].customer.y);
    vehicle.start_times[0] = std::max(static_cast<double>(route[0].customer.ready), travel_time_to_first);
    vehicle.waiting_times[0] = vehicle.start_times[0] - travel_time_to_first;

    double W_pref = vehicle.waiting_times[0]; // Cumulative waiting up to current
    double Phi = route[0].customer.due - vehicle.start_times[0]; // Forward slack

    for (int i = 1; i < n; i++) {
        // Travel time from previous to current
        double travel_time = euclideanDistance(route[i - 1].customer.x, route[i - 1].customer.y, route[i].customer.x, route[i].customer.y);
        double arrival_time = vehicle.start_times[i - 1] + route[i - 1].customer.service + travel_time;

        // Determine start time and waiting
        if (arrival_time < route[i].customer.ready) {
            vehicle.start_times[i] = route[i].customer.ready;
            vehicle.waiting_times[i] = route[i].customer.ready - arrival_time;
        }
        else {
            vehicle.start_times[i] = arrival_time;
            vehicle.waiting_times[i] = 0.0;
        }

        // Update cumulative waiting and forward slack
        W_pref += vehicle.waiting_times[i];
        Phi = std::min(Phi + vehicle.waiting_times[i],
            (route[i].customer.due - vehicle.start_times[i]) + (W_pref - vehicle.waiting_times[i]));

        // Early infeasibility detection
        if (vehicle.start_times[i] > route[i].customer.due && Phi < (vehicle.start_times[i] - route[i].customer.due)) {
            return false;
        }
    }

    // Check route duration
    double return_time = euclideanDistance(route.back().customer.x, route.back().customer.y, config.depot_x, config.depot_y);
    double total_duration = vehicle.start_times[n - 1] + route[n - 1].customer.service + return_time;
    if (total_duration > config.max_route_duration) {
        return false;
    }

    // Backward pass for waiting time reduction
    for (int i = n - 1; i > 0; i--) {
        if (vehicle.waiting_times[i] > 0) {
            double Delta = std::min(vehicle.waiting_times[i], route[i - 1].customer.due - vehicle.start_times[i - 1]);
            if (Delta > 0) {
                vehicle.start_times[i - 1] += Delta;
                vehicle.waiting_times[i - 1] += Delta;
                vehicle.waiting_times[i] -= Delta;
            }
        }
    }

    for (int i = 0; i < n; i++) {
        if (vehicle.start_times[i] < route[i].customer.ready - EPS ||
            vehicle.start_times[i] > route[i].customer.due + EPS) {
            return false;
        }
    }

    return true;
}


// Feasibility check with frigobox support
bool Neighborhood::isFeasible(Vehicle& vehicle, const Config& config) {
    // Capacity checks
    int totalFresh = 0;
    int totalFrozen = 0;
    for (const auto& rp : vehicle.route) {
        totalFresh += rp.customer.fresh;
        totalFrozen += rp.customer.frozen;
    }

    if (vehicle.type == "Fresh") {
        int required_frigo = ceil(static_cast<double>(totalFrozen) / config.frigobox_cap);
        int available_fresh = vehicle.fresh_cap - required_frigo * config.frigobox_weight;
        if (totalFresh > available_fresh || required_frigo > vehicle.max_frigo)
            return false;
    }
    else if (vehicle.type == "Frozen") {
        int required_frigo = ceil(static_cast<double>(totalFresh) / config.frigobox_cap);
        int available_frozen = vehicle.frozen_cap - required_frigo * config.frigobox_weight;
        if (totalFrozen > available_frozen || required_frigo > vehicle.max_frigo)
            return false;
    }
    else if (vehicle.type == "Mixed") {
        if (totalFresh > vehicle.fresh_cap || totalFrozen > vehicle.frozen_cap)
            return false;
    }

    // Temporal feasibility check
    return isTemporallyFeasibleSavelsbergh(vehicle, config);
}



//
//
//bool Neighborhood::checkMixedCapacityConstraint(const std::vector<Vehicle>& vehicles) const {
//    std::map<int, int> capacityCount;
//
//    for (const auto& v : vehicles) {
//        if (v.type == "Mixed" && !v.route.empty()) {
//            int totalCap = v.fresh_cap + v.frozen_cap;
//            if (++capacityCount[totalCap] > 1) {
//                return false;
//            }
//        }
//    }
//    return true;
//}
//




// Neighborhood operations
void Neighborhood::intraRouteSwap(Vehicle& vehicle, mt19937& gen, const Config& config) {
    if (vehicle.route.size() < 2) return;

    // Create temporary copy
    Vehicle temp = vehicle;

    // Perform swap on temp
    uniform_int_distribution<> dist(0, temp.route.size() - 1);
    int i = dist(gen), j = dist(gen);
    swap(temp.route[i], temp.route[j]);

    // Check feasibility on temp
    if (isFeasible(temp, config)) {
        vehicle = temp; // Commit only if valid
    }
}
void Neighborhood::interRouteSwap(vector<Vehicle>& vehicles, mt19937& gen, const Config& config) {
    if (vehicles.size() < 2) return;

    // Filter vehicles of the same type
    vector<Vehicle*> sameType;
    string target_type = vehicles[0].type;
    for (auto& v : vehicles) {
        if (v.type == target_type) sameType.push_back(&v);
    }
    if (sameType.size() < 2) return;

    // Select two distinct vehicles (using pointers to originals)
    uniform_int_distribution<> v_dist(0, sameType.size() - 1);
    Vehicle& original_v1 = *sameType[v_dist(gen)];
    Vehicle& original_v2 = *sameType[v_dist(gen)];
    if (&original_v1 == &original_v2) return;

    // Create temporary copies for modification
    Vehicle temp_v1 = original_v1;
    Vehicle temp_v2 = original_v2;

    if (temp_v1.route.empty() || temp_v2.route.empty()) return;

    // Select swap positions
    uniform_int_distribution<> p1(0, temp_v1.route.size() - 1);
    uniform_int_distribution<> p2(0, temp_v2.route.size() - 1);
    int i = p1(gen), j = p2(gen);

    // Perform swap on TEMPORARY routes
    swap(temp_v1.route[i], temp_v2.route[j]);

    // Check feasibility on BOTH modified copies
    if (isFeasible(temp_v1, config) && isFeasible(temp_v2, config)) {
        // Commit changes only if both are valid
        original_v1 = temp_v1;
        original_v2 = temp_v2;
    }
}


void Neighborhood::adjacentInterRouteSwap(vector<Vehicle>& vehicles, mt19937& gen, const Config& config) {
    if (vehicles.size() < 2) return;

    // Filter by vehicle type
    vector<Vehicle*> sameTypeVehicles;
    for (auto& v : vehicles) {
        if (v.type == vehicles[0].type) sameTypeVehicles.push_back(&v);
    }
    if (sameTypeVehicles.size() < 2) return;

    // Select two distinct vehicles
    uniform_int_distribution<> v_dist(0, sameTypeVehicles.size() - 1);
    int v1_idx = v_dist(gen), v2_idx;
    do { v2_idx = v_dist(gen); } while (v1_idx == v2_idx);

    // Create temporary copies of both vehicles
    Vehicle temp_v1 = *sameTypeVehicles[v1_idx];
    Vehicle temp_v2 = *sameTypeVehicles[v2_idx];

    if (temp_v1.route.size() < 2 || temp_v2.route.size() < 2) return;

    // Select positions in TEMPORARY routes
    uniform_int_distribution<> p1_dist(0, temp_v1.route.size() - 2);
    uniform_int_distribution<> p2_dist(0, temp_v2.route.size() - 2);
    int i = p1_dist(gen), j = p2_dist(gen);

    // Modify TEMPORARY copies
    swap(temp_v1.route[i], temp_v2.route[j]);
    swap(temp_v1.route[i + 1], temp_v2.route[j + 1]);

    // Validate before commit
    if (isFeasible(temp_v1, config) && isFeasible(temp_v2, config)) {
        // Atomic update to original vehicles
        *sameTypeVehicles[v1_idx] = temp_v1;
        *sameTypeVehicles[v2_idx] = temp_v2;
    }
}


void Neighborhood::nonAdjacentInterRouteSwap(vector<Vehicle>& vehicles, mt19937& gen, const Config& config) {
    if (vehicles.size() < 2) return;

    // Filter vehicles of the same type using first vehicle's type
    vector<Vehicle*> sameTypeVehicles;
    string targetType = vehicles[0].type;
    for (auto& v : vehicles) {
        if (v.type == targetType) sameTypeVehicles.push_back(&v);
    }
    if (sameTypeVehicles.size() < 2) return;

    // Select two distinct vehicles using temporary copies
    uniform_int_distribution<> vDist(0, sameTypeVehicles.size() - 1);
    int v1Idx = vDist(gen), v2Idx;
    do { v2Idx = vDist(gen); } while (v1Idx == v2Idx);

    Vehicle original_v1 = *sameTypeVehicles[v1Idx];  // Copy constructor
    Vehicle original_v2 = *sameTypeVehicles[v2Idx];
    Vehicle temp_v1 = original_v1;  // Working copies
    Vehicle temp_v2 = original_v2;

    // Validate temporary routes
    if (temp_v1.route.empty() || temp_v2.route.empty()) return;

    // Randomly select swap positions in TEMPORARY routes
    uniform_int_distribution<> pos1Dist(0, temp_v1.route.size() - 1);
    uniform_int_distribution<> pos2Dist(0, temp_v2.route.size() - 1);
    int pos1 = pos1Dist(gen);
    int pos2 = pos2Dist(gen);

    // Extract customers from TEMPORARY routes
    Customer c1 = temp_v1.route[pos1].customer;
    Customer c2 = temp_v2.route[pos2].customer;
    temp_v1.route.erase(temp_v1.route.begin() + pos1);
    temp_v2.route.erase(temp_v2.route.begin() + pos2);

    // Determine insertion positions in TEMPORARY routes
    uniform_int_distribution<> newPos1Dist(0, temp_v2.route.size());
    uniform_int_distribution<> newPos2Dist(0, temp_v1.route.size());
    int newPos1 = newPos1Dist(gen);
    int newPos2 = newPos2Dist(gen);

    // Apply reversal logic to TEMPORARY routes
    bernoulli_distribution revDist(0.5);
    bool reverse1 = revDist(gen);
    bool reverse2 = revDist(gen);

    if (reverse1) reverse(temp_v2.route.begin(), temp_v2.route.end());
    if (reverse2) reverse(temp_v1.route.begin(), temp_v1.route.end());

    // Insert into TEMPORARY routes
    temp_v2.route.insert(temp_v2.route.begin() + newPos1, { c1, 0.0, 0.0 });
    temp_v1.route.insert(temp_v1.route.begin() + newPos2, { c2, 0.0, 0.0 });

    // Restore original route order if reversed
    if (reverse1) reverse(temp_v2.route.begin(), temp_v2.route.end());
    if (reverse2) reverse(temp_v1.route.begin(), temp_v1.route.end());

    // Atomic validation check
    if (isFeasible(temp_v1, config) && isFeasible(temp_v2, config)) {
        // Commit changes to original vehicles
        *sameTypeVehicles[v1Idx] = temp_v1;
        *sameTypeVehicles[v2Idx] = temp_v2;
    }
    else {
        // Automatic rollback through copy destruction
    }
}

void Neighborhood::relocateCustomer(vector<Vehicle>& vehicles, mt19937& gen, const Config& config) {
    if (vehicles.size() < 2) return;

    // Create temporary copies of all vehicles and track original indices
    vector<Vehicle> tempVehicles = vehicles;
    vector<size_t> originalIndices;
    for (size_t i = 0; i < tempVehicles.size(); ++i) {
        originalIndices.push_back(i);
    }

    // Filter vehicles of the same type, preserving original indices
    vector<pair<Vehicle*, size_t>> sameTypeVehicles; // (temp vehicle, original index)
    string targetType = tempVehicles[0].type;
    for (size_t i = 0; i < tempVehicles.size(); ++i) {
        if (tempVehicles[i].type == targetType) {
            sameTypeVehicles.emplace_back(&tempVehicles[i], originalIndices[i]);
        }
    }
    if (sameTypeVehicles.size() < 2) return;

    // Select source and target vehicles from the filtered list
    uniform_int_distribution<> dist(0, sameTypeVehicles.size() - 1);
    int v1_idx = dist(gen), v2_idx;
    do { v2_idx = dist(gen); } while (v1_idx == v2_idx);

    auto& [srcTemp, srcOriginalIdx] = sameTypeVehicles[v1_idx];
    auto& [tgtTemp, tgtOriginalIdx] = sameTypeVehicles[v2_idx];

    if (srcTemp->route.empty()) return;

    // Select customer to move in the TEMPORARY source
    uniform_int_distribution<> dist1(0, srcTemp->route.size() - 1);
    int i = dist1(gen);
    Customer c = srcTemp->route[i].customer;

    // Remove from TEMPORARY source
    srcTemp->route.erase(srcTemp->route.begin() + i);

    // Insert into TEMPORARY target
    uniform_int_distribution<> dist2(0, tgtTemp->route.size());
    int j = dist2(gen);

    // Reverse logic on temporary target
    bernoulli_distribution reverseDist(0.5);
    bool reversed = reverseDist(gen);
    if (reversed) reverse(tgtTemp->route.begin(), tgtTemp->route.end());

    tgtTemp->route.insert(tgtTemp->route.begin() + j, { c, 0.0, 0.0 });

    // Restore original order for validation
    if (reversed) reverse(tgtTemp->route.begin(), tgtTemp->route.end());

    // Validate both temporary vehicles
    if (isFeasible(*srcTemp, config) && isFeasible(*tgtTemp, config)) {
        // Commit to original vehicles using tracked indices
        vehicles[srcOriginalIdx] = *srcTemp;
        vehicles[tgtOriginalIdx] = *tgtTemp;
    }
}

void Neighborhood::relocateTwoCustomers(vector<Vehicle>& vehicles, mt19937& gen, const Config& config) {
    if (vehicles.size() < 3) return;

    vector<Vehicle*> sameTypeVehicles;
    for (auto& v : vehicles) {
        if (v.type == vehicles[0].type) sameTypeVehicles.push_back(&v);
    }
    if (sameTypeVehicles.size() < 2) return;

    // Create working copies
    vector<Vehicle> tempVehicles;
    for (auto& v : vehicles) tempVehicles.push_back(v);

    uniform_int_distribution<> v_dist(0, sameTypeVehicles.size() - 1);
    int src_idx = v_dist(gen), tgt_idx;
    do { tgt_idx = v_dist(gen); } while (src_idx == tgt_idx);

    Vehicle& src = tempVehicles[src_idx];
    Vehicle& tgt = tempVehicles[tgt_idx];

    if (src.route.size() < 2) return;

    uniform_int_distribution<> pos_dist(0, src.route.size() - 2);
    int pos = pos_dist(gen);
    auto c1 = src.route[pos];
    auto c2 = src.route[pos + 1];

    // Modify TEMPORARY vehicles
    src.route.erase(src.route.begin() + pos, src.route.begin() + pos + 2);

    bernoulli_distribution rev_dist(0.5);
    if (rev_dist(gen)) swap(c1, c2);

    uniform_int_distribution<> ins_dist(0, tgt.route.size());
    int ins_pos = ins_dist(gen);

    tgt.route.insert(tgt.route.begin() + ins_pos, c1);
    tgt.route.insert(tgt.route.begin() + ins_pos + 1, c2);

    // Validate both modified vehicles
    if (isFeasible(src, config) && isFeasible(tgt, config)) {
        // Atomic commit to original vehicles
        vehicles[src_idx] = src;
        vehicles[tgt_idx] = tgt;
    }
}

void Neighborhood::intraRouteRelocate(Vehicle& vehicle, mt19937& gen, const Config& config) {
    if (vehicle.route.size() < 2) return;

    // Work on temporary copy
    Vehicle tempVehicle = vehicle;

    uniform_int_distribution<> posDist(0, tempVehicle.route.size() - 1);
    int oldPos = posDist(gen);
    int newPos;
    do { newPos = posDist(gen); } while (oldPos == newPos);

    // Modify temporary route
    auto customer = tempVehicle.route[oldPos];
    tempVehicle.route.erase(tempVehicle.route.begin() + oldPos);
    tempVehicle.route.insert(tempVehicle.route.begin() + newPos, customer);

    // Validate before commit
    if (isFeasible(tempVehicle, config)) {
        vehicle = tempVehicle;
    }
}


void Neighborhood::intraRouteRelocateTwo(Vehicle& vehicle, mt19937& gen, const Config& config) {
    if (vehicle.route.size() < 3) return;

    // Create a temporary copy to modify
    Vehicle tempVehicle = vehicle;

    // Select segment of 2 adjacent customers in the TEMPORARY route
    uniform_int_distribution<> posDist(0, tempVehicle.route.size() - 2);
    int oldPos = posDist(gen);

    // Extract segment from TEMPORARY route
    auto segmentStart = tempVehicle.route.begin() + oldPos;
    vector<RoutePoint> segment(segmentStart, segmentStart + 2);
    tempVehicle.route.erase(segmentStart, segmentStart + 2);

    // Select new insertion position in TEMPORARY route
    uniform_int_distribution<> newPosDist(0, tempVehicle.route.size());
    int newPos = newPosDist(gen);

    // Insert segment into TEMPORARY route
    tempVehicle.route.insert(tempVehicle.route.begin() + newPos, segment.begin(), segment.end());

    // Validate and commit changes atomically
    if (isFeasible(tempVehicle, config)) {
        vehicle = tempVehicle; // Replace original route only if valid
    }
}


void Neighborhood::relocateInterRoute(vector<Vehicle>& vehicles, mt19937& gen, const Config& config) {
    if (vehicles.empty()) return;

    // 1. Select a random vehicle
    uniform_int_distribution<> veh_dist(0, vehicles.size() - 1);
    Vehicle& vehicle = vehicles[veh_dist(gen)];

    // 2. Apply intra-route relocation to the selected vehicle
    intraRouteRelocate(vehicle, gen, config); // Now modifies the same vehicle's route
}



void Neighborhood::segmentShift(Vehicle& vehicle, mt19937& gen, const Config& config) {
    if (vehicle.route.size() < 2) return;

    // Create temporary copy to modify
    Vehicle tempVehicle = vehicle;

    int n = tempVehicle.route.size();
    int d_min = max(1, static_cast<int>(0.3 * n));
    int d_max = min(n, static_cast<int>(0.7 * n));

    // Randomly select segment parameters
    uniform_int_distribution<> dist_d(d_min, d_max);
    int d = dist_d(gen);
    uniform_int_distribution<> dist_pos(0, n - d);
    int pos = dist_pos(gen);

    // Shuffle the segment in the TEMPORARY route
    auto start = tempVehicle.route.begin() + pos;
    auto end = start + d;
    shuffle(start, end, gen);

    // Validate the modified temporary route
    if (isFeasible(tempVehicle, config)) {
        vehicle = tempVehicle; // Atomic commit to original
    }
}

//void Neighborhood::removeInsert(std::vector<Vehicle>& vehicles, std::mt19937& gen,
//    const Config& config, int& lambda, int lambda_min, int lambda_max) {
//
//
//    std::vector<std::pair<Customer, std::string>> removedCustomers;
//
//    // Étape 1 : Supprimer λ clients
//    lambda = std::clamp(lambda, lambda_min, lambda_max); // Garantir λ ∈ [λ_min, λ_max]
//    for (int i = 0; i < lambda; ++i) {
//        std::uniform_int_distribution<> veh_dist(0, vehicles.size() - 1);
//        int v_idx = veh_dist(gen);
//
//        if (vehicles[v_idx].route.size() <= 1) continue; // Avoid emptying
//
//
//        std::uniform_int_distribution<> cust_dist(0, vehicles[v_idx].route.size() - 1);
//        int c_idx = cust_dist(gen);
//
//        removedCustomers.emplace_back(vehicles[v_idx].route[c_idx].customer, vehicles[v_idx].type);
//        vehicles[v_idx].route.erase(vehicles[v_idx].route.begin() + c_idx);
//    }
//
//    // Étape 2 :  (meilleure position)
//    for (const auto& [customer, originalType] : removedCustomers) {
//        double bestDelta = std::numeric_limits<double>::max();
//        int bestVehIdx = -1;
//        int bestPos = -1;
//
//        // Chercher dans toutes les routes du même type
//        for (size_t v_idx = 0; v_idx < vehicles.size(); ++v_idx) {
//            if (vehicles[v_idx].type != originalType) continue;
//
//            // Évaluer toutes les positions possibles
//            for (size_t pos = 0; pos <= vehicles[v_idx].route.size(); ++pos) {
//                Vehicle temp = vehicles[v_idx];
//                temp.route.insert(temp.route.begin() + pos, { customer, 0.0, 0.0 });
//
//                if (isFeasible(temp, config)) {
//                    double originalCost = calculateTotalCost({ vehicles[v_idx] }, config);
//                    double newCost = calculateTotalCost({ temp }, config);
//                    double delta = newCost - originalCost;
//
//                    if (delta < bestDelta) {
//                        bestDelta = delta;
//                        bestVehIdx = v_idx;
//                        bestPos = pos;
//                    }
//                }
//            }
//        }
//
//        // Réinsérer à la meilleure position trouvée
//        if (bestVehIdx != -1) {
//            vehicles[bestVehIdx].route.insert(vehicles[bestVehIdx].route.begin() + bestPos,
//                { customer, 0.0, 0.0 });
//        }
//    }
//}


void Neighborhood::removeInsert(std::vector<Vehicle>& vehicles, std::mt19937& gen,
    const Config& config, int& lambda, int lambda_min, int lambda_max) {

    // Create a copy of the original vehicles to modify
    std::vector<Vehicle> modifiedVehicles = vehicles;
    std::vector<std::pair<Customer, std::string>> removedCustomers;

    // Step 1: Remove λ customers from the modified vehicles
    lambda = std::clamp(lambda, lambda_min, lambda_max);
    for (int i = 0; i < lambda; ++i) {
        std::uniform_int_distribution<> veh_dist(0, modifiedVehicles.size() - 1);
        int v_idx = veh_dist(gen);

        if (modifiedVehicles[v_idx].route.size() <= 1) continue; // Avoid emptying

        std::uniform_int_distribution<> cust_dist(0, modifiedVehicles[v_idx].route.size() - 1);
        int c_idx = cust_dist(gen);

        removedCustomers.emplace_back(modifiedVehicles[v_idx].route[c_idx].customer, modifiedVehicles[v_idx].type);
        modifiedVehicles[v_idx].route.erase(modifiedVehicles[v_idx].route.begin() + c_idx);
    }

    // Step 2: Attempt to reinsert all removed customers into modifiedVehicles
    bool allInserted = true;
    for (const auto& [customer, originalType] : removedCustomers) {
        double bestDelta = std::numeric_limits<double>::max();
        int bestVehIdx = -1;
        int bestPos = -1;

        // Search in all vehicles of the same type
        for (size_t v_idx = 0; v_idx < modifiedVehicles.size(); ++v_idx) {
            //if (modifiedVehicles[v_idx].type != originalType) continue;

            // Evaluate all possible positions
            for (size_t pos = 0; pos <= modifiedVehicles[v_idx].route.size(); ++pos) {
                Vehicle temp = modifiedVehicles[v_idx];
                temp.route.insert(temp.route.begin() + pos, { customer, 0.0, 0.0 });

                if (isFeasible(temp, config)) {
                    double originalCost = calculateTotalCost({ modifiedVehicles[v_idx] }, config);
                    double newCost = calculateTotalCost({ temp }, config);
                    double delta = newCost - originalCost;

                    if (delta < bestDelta) {
                        bestDelta = delta;
                        bestVehIdx = v_idx;
                        bestPos = pos;
                    }
                }
            }
        }

        // Check if insertion is possible
        if (bestVehIdx != -1) {
            modifiedVehicles[bestVehIdx].route.insert(
                modifiedVehicles[bestVehIdx].route.begin() + bestPos,
                { customer, 0.0, 0.0 }
            );
        }
        else {
            allInserted = false;
            break; // Abort if any insertion fails
        }
    }

    // Step 3: Update original vehicles only if all insertions succeeded
    if (allInserted) {
        vehicles = std::move(modifiedVehicles);
    }
}

//2 - opt * (Potvin & Rousseau, 1995)

void Neighborhood::twoOptInterRoute(vector<Vehicle>& vehicles, mt19937& gen, const Config& config) {
    if (vehicles.size() < 2) return;

    // Filter vehicles of the same type
    vector<Vehicle*> sameTypeVehicles;
    string targetType = vehicles[0].type;
    for (auto& v : vehicles) {
        if (v.type == targetType) sameTypeVehicles.push_back(&v);
    }
    if (sameTypeVehicles.size() < 2) return;

    // Select two distinct vehicles
    uniform_int_distribution<> vDist(0, sameTypeVehicles.size() - 1);
    int v1Idx = vDist(gen), v2Idx;
    do { v2Idx = vDist(gen); } while (v1Idx == v2Idx);
    Vehicle& originalV1 = *sameTypeVehicles[v1Idx];
    Vehicle& originalV2 = *sameTypeVehicles[v2Idx];

    // Create temporary copies
    Vehicle tempV1 = originalV1;
    Vehicle tempV2 = originalV2;

    // Validate route lengths
    if (tempV1.route.size() < 2 || tempV2.route.size() < 2) return;

    // Randomly choose split points
    uniform_int_distribution<> split1(1, tempV1.route.size() - 1);
    uniform_int_distribution<> split2(1, tempV2.route.size() - 1);
    int splitPos1 = split1(gen);
    int splitPos2 = split2(gen);

    // Capture tails from TEMPORARY routes
    auto tail1 = vector<RoutePoint>(tempV1.route.begin() + splitPos1, tempV1.route.end());
    auto tail2 = vector<RoutePoint>(tempV2.route.begin() + splitPos2, tempV2.route.end());

    // Modify temporary routes
    tempV1.route.erase(tempV1.route.begin() + splitPos1, tempV1.route.end());
    tempV2.route.erase(tempV2.route.begin() + splitPos2, tempV2.route.end());
    tempV1.route.insert(tempV1.route.end(), tail2.begin(), tail2.end());
    tempV2.route.insert(tempV2.route.end(), tail1.begin(), tail1.end());

    // Validate both modified routes
    if (isFeasible(tempV1, config) && isFeasible(tempV2, config)) {
        originalV1 = tempV1; // Atomic commit
        originalV2 = tempV2;
    }
}


//2 opt operator of Lin(1965)
void Neighborhood::twoOptStarIntraRoute(Vehicle& vehicle, mt19937& gen, const Config& config) {
    if (vehicle.route.size() <= 3) return;

    // Work on a temporary copy
    Vehicle tempVehicle = vehicle;

    // Randomly select reversal points
    uniform_int_distribution<> dist(1, tempVehicle.route.size() - 2);
    int i = dist(gen), j = dist(gen);
    if (i > j) swap(i, j);

    // Skip if not a real segment
    if (j - i < 1) return;

    // Reverse segment in TEMPORARY route
    reverse(tempVehicle.route.begin() + i, tempVehicle.route.begin() + j + 1);

    // Validate before committing
    if (isFeasible(tempVehicle, config)) {
        vehicle = tempVehicle; // Atomic update
    }
}




//void Neighborhood::removeTwoInsertOneBestPosition(std::vector<Vehicle>& vehicles, std::mt19937& gen, const Config& config) {
//    if (vehicles.empty()) return;
//
//    // 1. Create temporary copies of ALL vehicles upfront
//    std::vector<Vehicle> tempVehicles = vehicles;
//    std::vector<std::pair<Customer, int>> removedCustomers; // <customer, original temp vehicle index>
//
//    // 2. Remove two customers from TEMPORARY vehicles
//    std::vector<int> candidateVehicles;
//    for (size_t i = 0; i < tempVehicles.size(); ++i) {
//        if (tempVehicles[i].route.size() >= 2) {
//            candidateVehicles.push_back(i);
//        }
//    }
//    if (candidateVehicles.empty()) return;
//
//    // Select source vehicle in TEMPORARY copy
//    std::uniform_int_distribution<> veh_dist(0, candidateVehicles.size() - 1);
//    int src_idx = candidateVehicles[veh_dist(gen)];
//    Vehicle& src_vehicle = tempVehicles[src_idx];
//
//    // Select two distinct positions in TEMPORARY route
//    std::uniform_int_distribution<> pos_dist(0, src_vehicle.route.size() - 1);
//    int pos1 = pos_dist(gen);
//    int pos2;
//    do { pos2 = pos_dist(gen); } while (pos2 == pos1);
//    if (pos1 > pos2) std::swap(pos1, pos2);
//
//    // Remove from TEMPORARY vehicle
//    Customer c1 = src_vehicle.route[pos1].customer;
//    Customer c2 = src_vehicle.route[pos2].customer;
//    src_vehicle.route.erase(src_vehicle.route.begin() + pos2);
//    src_vehicle.route.erase(src_vehicle.route.begin() + pos1);
//    removedCustomers.emplace_back(c1, src_idx);
//    removedCustomers.emplace_back(c2, src_idx);
//
//    // 3. Reinsert into TEMPORARY vehicles
//    bool allFeasible = true;
//    for (const auto& [customer, origVehIdx] : removedCustomers) {
//        double bestDelta = std::numeric_limits<double>::max();
//        int bestVehIdx = -1;
//        int bestPos = -1;
//        std::string targetType = tempVehicles[origVehIdx].type;
//
//        // Search TEMPORARY vehicles of same type
//        for (size_t v_idx = 0; v_idx < tempVehicles.size(); ++v_idx) {
//            if (tempVehicles[v_idx].type != targetType) continue;
//
//            // Check all insertion positions in TEMPORARY vehicle
//            for (size_t ins_pos = 0; ins_pos <= tempVehicles[v_idx].route.size(); ++ins_pos) {
//                Vehicle temp = tempVehicles[v_idx]; // Local copy for testing
//                temp.route.insert(temp.route.begin() + ins_pos, { customer, 0.0, 0.0 });
//
//                if (isFeasible(temp, config)) {
//                    double originalCost = calculateTotalCost({ tempVehicles[v_idx] }, config);
//                    double newCost = calculateTotalCost({ temp }, config);
//                    double delta = newCost - originalCost;
//
//                    if (delta < bestDelta) {
//                        bestDelta = delta;
//                        bestVehIdx = v_idx;
//                        bestPos = ins_pos;
//                    }
//                }
//            }
//        }
//
//        // Update TEMPORARY vehicles if valid insertion found
//        if (bestVehIdx != -1) {
//            tempVehicles[bestVehIdx].route.insert(
//                tempVehicles[bestVehIdx].route.begin() + bestPos,
//                { customer, 0.0, 0.0 }
//            );
//        }
//        else {
//            allFeasible = false;
//            break;
//        }
//    }
//
//    // 4. Global feasibility check before commit
//    if (allFeasible) {
//        allFeasible = std::all_of(tempVehicles.begin(), tempVehicles.end(),
//            [&](const Vehicle& v) { return isFeasible(v, config); });
//    }
//
//    // 5. Atomic commit to original vehicles
//    if (allFeasible) {
//        vehicles = std::move(tempVehicles);
//    }
//}


void Neighborhood::removeTwoInsertOneBestPosition(std::vector<Vehicle>& vehicles, std::mt19937& gen, const Config& config) {
    if (vehicles.empty()) return;

    // 1. Create temporary copies of ALL vehicles upfront
    std::vector<Vehicle> tempVehicles = vehicles;
    std::vector<std::pair<Customer, int>> removedCustomers; // <customer, original temp vehicle index>

    // 2. Remove two customers from TEMPORARY vehicles
    std::vector<int> candidateVehicles;
    for (size_t i = 0; i < tempVehicles.size(); ++i) {
        if (tempVehicles[i].route.size() >= 2) {
            candidateVehicles.push_back(i);
        }
    }
    if (candidateVehicles.empty()) return;

    // Select source vehicle in TEMPORARY copy
    std::uniform_int_distribution<> veh_dist(0, candidateVehicles.size() - 1);
    int src_idx = candidateVehicles[veh_dist(gen)];
    Vehicle& src_vehicle = tempVehicles[src_idx];

    // Select two distinct positions in TEMPORARY route
    std::uniform_int_distribution<> pos_dist(0, src_vehicle.route.size() - 1);
    int pos1 = pos_dist(gen);
    int pos2;
    do { pos2 = pos_dist(gen); } while (pos2 == pos1);
    if (pos1 > pos2) std::swap(pos1, pos2);

    // Remove from TEMPORARY vehicle
    Customer c1 = src_vehicle.route[pos1].customer;
    Customer c2 = src_vehicle.route[pos2].customer;
    src_vehicle.route.erase(src_vehicle.route.begin() + pos2);
    src_vehicle.route.erase(src_vehicle.route.begin() + pos1);
    removedCustomers.emplace_back(c1, src_idx);
    removedCustomers.emplace_back(c2, src_idx);

    // 3. Reinsert into TEMPORARY vehicles
    bool allFeasible = true;
    for (const auto& [customer, origVehIdx] : removedCustomers) {
        double bestDelta = std::numeric_limits<double>::max();
        int bestVehIdx = -1;
        int bestPos = -1;
        std::string targetType = tempVehicles[origVehIdx].type;

        // Search TEMPORARY vehicles of same type
        for (size_t v_idx = 0; v_idx < tempVehicles.size(); ++v_idx) {
            if (tempVehicles[v_idx].type != targetType) continue;

            // Check all insertion positions in TEMPORARY vehicle
            for (size_t ins_pos = 0; ins_pos <= tempVehicles[v_idx].route.size(); ++ins_pos) {
                Vehicle temp = tempVehicles[v_idx]; // Local copy for testing
                temp.route.insert(temp.route.begin() + ins_pos, { customer, 0.0, 0.0 });

                if (isFeasible(temp, config)) {
                    double originalCost = calculateTotalCost({ tempVehicles[v_idx] }, config);
                    double newCost = calculateTotalCost({ temp }, config);
                    double delta = newCost - originalCost;

                    if (delta < bestDelta) {
                        bestDelta = delta;
                        bestVehIdx = v_idx;
                        bestPos = ins_pos;
                    }
                }
            }
        }

        // Update TEMPORARY vehicles if valid insertion found
        if (bestVehIdx != -1) {
            tempVehicles[bestVehIdx].route.insert(
                tempVehicles[bestVehIdx].route.begin() + bestPos,
                { customer, 0.0, 0.0 }
            );
        }
        else {
            allFeasible = false;
            break;
        }
    }

    // 4. Global feasibility check before commit
    if (allFeasible) {
        // Use a non-const reference in the lambda to match isFeasible signature
        allFeasible = std::all_of(tempVehicles.begin(), tempVehicles.end(),
            [&](Vehicle& v) { return isFeasible(v, config); });
    }

    // 5. Atomic commit to original vehicles
    if (allFeasible) {
        vehicles = std::move(tempVehicles);
    }
}


void Neighborhood::localSearch(std::vector<Vehicle>& vehicles, std::mt19937& gen, const Config& config, int totalCustomers) {
    if (vehicles.empty()) return;

    // Define local search operators
    std::vector<std::function<void()>> operators = {
        [&]() { twoOptInterRoute(vehicles, gen, config); },
        [&]() { relocateInterRoute(vehicles, gen, config); },
        [&]() {
            std::uniform_int_distribution<> v_dist(0, vehicles.size() - 1);
            twoOptStarIntraRoute(vehicles[v_dist(gen)], gen, config);
        },
        [&]() { removeTwoInsertOneBestPosition(vehicles, gen, config); }
    };

    // Select one random operator
    std::uniform_int_distribution<int> op_dist(0, operators.size() - 1);
    int selectedOp = op_dist(gen);

    // Apply the chosen operator 10 times
    for (int i = 0; i < 50; ++i) {
        operators[selectedOp]();
    }
}



//void localSearch(vector<Vehicle>& vehicles, mt19937& gen, const Config& config, int totalCustomers) {
//    int maxIterations = 100;
//    double bestCost = calculateTotalCost(vehicles, config);
//
//    // Lambda bounds
//    int lambda_min = max(5, static_cast<int>(0.1 * totalCustomers));
//    int lambda_max = min(100, static_cast<int>(0.3 * totalCustomers));
//    int lambda = lambda_min;
//
//    vector<function<void()>> operators = {
//        [&]() { twoOptInterRoute(vehicles, gen, config); },
//        [&]() { relocateInterRoute(vehicles, gen, config); },
//        [&]() { twoOptStarIntraRoute(vehicles[0], gen, config); },
//        [&]() { removeTwoInsertOne(vehicles[0], gen, config); }
//    };
//
//    for (int iter = 0; iter < maxIterations; ++iter) {
//        bool improved = false;
//        shuffle(operators.begin(), operators.end(), gen);
//
//        // Apply core operators
//        for (auto& op : operators) op();
//
//        // Apply Remove-Insert with adaptive lambda
//        lambda = clamp(lambda, lambda_min, lambda_max); // Enforce bounds
//        removeInsert(vehicles, gen, config, lambda);
//
//        // Check improvement
//        double newCost = calculateTotalCost(vehicles, config);
//        if (newCost < bestCost - 1e-6) {
//            bestCost = newCost;
//            lambda = lambda_min; // Reset to minimum
//            improved = true;
//        }
//        else {
//            lambda = min(lambda + 1, lambda_max); // Increment
//        }
//    }
//}