#include "VehicleOptimizer.h"
#include <random>
#include <algorithm>
#include <set>
#include <fstream>
#include <chrono>
#include <unordered_map>
#include <unordered_set>


VehicleOptimizer::VehicleOptimizer(const Config& config)
    : w_(config.frigobox_weight), F_(config.total_frigo) {
}

std::string VehicleOptimizer::extractGroupId(const std::string& vehicle_id) const {
    // For mixed vehicles, group is the first character (e.g., "C" from "C1")
    // For other vehicles, the vehicle ID itself is the group
    if (vehicle_id.empty()) return vehicle_id;

    // Check if it's a mixed vehicle ID (starts with letter, ends with number)
    if (std::isalpha(vehicle_id[0]) && vehicle_id.size() > 1 && std::isdigit(vehicle_id[1])) {
        return vehicle_id.substr(0, 1);
    }

    return vehicle_id;
}

// Add back the selectVehicles implementation
std::optional<std::vector<VehicleOptimizer::Assignment>> VehicleOptimizer::selectVehicles(
    const std::vector<RouteDemand>& routes,
    const std::vector<Vehicle>& vehicles,
    std::mt19937& gen
) {
    // Step 1: Vehicle Capability Analysis
    auto candidateSets = vehicleCapabilityAnalysis(routes, vehicles);

    // Check if any route has no feasible candidates
    for (size_t i = 0; i < candidateSets.size(); ++i) {
        if (candidateSets[i].empty()) {
            std::cout << "[DEBUG] Route " << i << " has no feasible vehicle assignment\n";
            return std::nullopt;
        }
    }

    // Step 2: Select Best Vehicles using the two-phase approach
    return selectBestVehicle(candidateSets, gen);
}

std::vector<std::vector<VehicleOptimizer::Candidate>> VehicleOptimizer::vehicleCapabilityAnalysis(
    const std::vector<RouteDemand>& routes,
    const std::vector<Vehicle>& vehicles
) {
    std::vector<std::vector<Candidate>> allCandidates;

    // Categorize vehicles
    std::vector<const Vehicle*> K1, K2;
    std::vector<MixedVehicleGroup> K3_groups;

    // Build mixed vehicle groups from vehicle data
    std::unordered_map<std::string, MixedVehicleGroup> mixedGroups;

    for (const auto& vehicle : vehicles) {
        if (vehicle.type == "Fresh") {
            K1.push_back(&vehicle);
        }
        else if (vehicle.type == "Frozen") {
            K2.push_back(&vehicle);
        }
        else if (vehicle.type == "Mixed") {
            std::string group_id = extractGroupId(vehicle.id);
            MixedVehicleConfig config{ vehicle.id, vehicle.fresh_cap, vehicle.frozen_cap };
            mixedGroups[group_id].group_id = group_id;
            mixedGroups[group_id].configurations.push_back(config);
        }
    }

    // Convert map to vector for K3 groups
    for (auto& pair : mixedGroups) {
        K3_groups.push_back(pair.second);
    }

    for (const auto& route : routes) {
        std::vector<Candidate> routeCandidates;
        int w1 = route.total_fresh;
        int w2 = route.total_frozen;

        // ALWAYS check Fresh vehicles (K1)
        for (const auto* vehicle : K1) {
            int d_p = w1;
            int d_s = w2;
            int O = ceilDiv(d_s, w_);
            int residual = vehicle->fresh_cap - (d_p + O * w_);

            if (O <= vehicle->max_frigo && residual >= 0) {
                routeCandidates.push_back(Candidate{
                    Assignment::Type::K1,
                    vehicle->id,
                    vehicle->id,  // Group ID same as vehicle ID for K1/K2
                    O,
                    residual
                    });
            }
        }

        // ALWAYS check Frozen vehicles (K2)
        for (const auto* vehicle : K2) {
            int d_p = w2;
            int d_s = w1;
            int O = ceilDiv(d_s, w_);
            int residual = vehicle->frozen_cap - (d_p + O * w_);

            if (O <= vehicle->max_frigo && residual >= 0) {
                routeCandidates.push_back(Candidate{
                    Assignment::Type::K2,
                    vehicle->id,
                    vehicle->id,  // Group ID same as vehicle ID for K1/K2
                    O,
                    residual
                    });
            }
        }

        // ALWAYS check Mixed vehicles (K3) - Removed the restrictive condition
        for (const auto& group : K3_groups) {
            std::optional<Candidate> bestConfigInGroup;
            int minResidualInGroup = std::numeric_limits<int>::max();

            // Evaluate ALL configurations in this group to find the best fit
            for (const auto& config : group.configurations) {
                if (w1 <= config.fresh_cap && w2 <= config.frozen_cap) {
                    int residual = (config.fresh_cap - w1) + (config.frozen_cap - w2);

                    // Keep track of the configuration with minimum residual
                    if (residual < minResidualInGroup) {
                        minResidualInGroup = residual;
                        bestConfigInGroup = Candidate{
                            Assignment::Type::K3,
                            config.config_id,
                            group.group_id,
                            0,  // Mixed vehicles don't use frigoboxes
                            residual
                        };
                    }
                }
            }

            // Add the best candidate from this group (if any feasible configuration exists)
            if (bestConfigInGroup.has_value()) {
                routeCandidates.push_back(bestConfigInGroup.value());
            }
        }

        allCandidates.push_back(routeCandidates);
    }

    return allCandidates;
}

std::optional<std::vector<VehicleOptimizer::Assignment>> VehicleOptimizer::selectBestVehicle(
    const std::vector<std::vector<Candidate>>& candidateSets,
    std::mt19937& gen
) {
    std::vector<Assignment> assignments(candidateSets.size());
    std::unordered_set<std::string> usedVehicles;
    std::unordered_set<std::string> usedVehicleGroups;  // Track used vehicle groups
    std::set<size_t> unservedRoutes;
    int remainingFrigoboxes = F_;

    // Initialize unserved routes
    for (size_t i = 0; i < candidateSets.size(); ++i) {
        unservedRoutes.insert(i);
    }

    // Helper function to check if a candidate is available
    auto isCandidateAvailable = [&](const Candidate& candidate) {
        if (usedVehicles.find(candidate.vehicle_id) != usedVehicles.end()) {
            return false;
        }

        // For mixed vehicles, check if the vehicle group is already used
        if (candidate.type == Assignment::Type::K3) {
            if (usedVehicleGroups.find(candidate.group_id) != usedVehicleGroups.end()) {
                return false;
            }
        }

        // Check frigobox constraints for boxed vehicles
        if (candidate.frigoboxes > 0 && candidate.frigoboxes > remainingFrigoboxes) {
            return false;
        }

        return true;
        };

    // PHASE 1: BOXED-ONLY phase
    bool hasBoxedCandidate = false;
    for (size_t r : unservedRoutes) {
        for (const auto& candidate : candidateSets[r]) {
            if (isCandidateAvailable(candidate) && candidate.frigoboxes > 0) {
                hasBoxedCandidate = true;
                break;
            }
        }
        if (hasBoxedCandidate) break;
    }

    if (hasBoxedCandidate) {
        while (!unservedRoutes.empty()) {
            // Check if we still have boxed-feasible candidates
            bool hasFeasibleCandidate = false;
            for (size_t r : unservedRoutes) {
                for (const auto& candidate : candidateSets[r]) {
                    if (isCandidateAvailable(candidate) && candidate.frigoboxes > 0) {
                        hasFeasibleCandidate = true;
                        break;
                    }
                }
                if (hasFeasibleCandidate) break;
            }

            if (!hasFeasibleCandidate) break;

            // Find route with minimum attainable box demand
            int minBoxDemand = std::numeric_limits<int>::max();
            std::vector<size_t> candidateRoutes;

            for (size_t r : unservedRoutes) {
                int routeMinBoxes = std::numeric_limits<int>::max();
                for (const auto& candidate : candidateSets[r]) {
                    if (isCandidateAvailable(candidate) && candidate.frigoboxes > 0) {
                        routeMinBoxes = std::min(routeMinBoxes, candidate.frigoboxes);
                    }
                }

                if (routeMinBoxes < std::numeric_limits<int>::max()) {
                    if (routeMinBoxes < minBoxDemand) {
                        minBoxDemand = routeMinBoxes;
                        candidateRoutes.clear();
                        candidateRoutes.push_back(r);
                    }
                    else if (routeMinBoxes == minBoxDemand) {
                        candidateRoutes.push_back(r);
                    }
                }
            }

            if (candidateRoutes.empty()) break;

            // Select route uniformly from candidates with minimum box demand
            std::uniform_int_distribution<size_t> routeDist(0, candidateRoutes.size() - 1);
            size_t selectedRoute = candidateRoutes[routeDist(gen)];

            // Find best candidate for selected route (minimum residual capacity)
            std::vector<Candidate> feasibleCandidates;
            int minResidual = std::numeric_limits<int>::max();

            for (const auto& candidate : candidateSets[selectedRoute]) {
                if (isCandidateAvailable(candidate) && candidate.frigoboxes > 0) {
                    minResidual = std::min(minResidual, candidate.residual_capacity);
                    feasibleCandidates.push_back(candidate);
                }
            }

            // Filter candidates with minimum residual capacity
            std::vector<Candidate> bestCandidates;
            for (const auto& candidate : feasibleCandidates) {
                if (candidate.residual_capacity == minResidual) {
                    bestCandidates.push_back(candidate);
                }
            }

            if (bestCandidates.empty()) {
                unservedRoutes.erase(selectedRoute);
                continue;
            }

            // Select candidate uniformly from best candidates
            std::uniform_int_distribution<size_t> candidateDist(0, bestCandidates.size() - 1);
            Candidate selectedCandidate = bestCandidates[candidateDist(gen)];

            // Commit assignment
            assignments[selectedRoute] = candidateToAssignment(selectedCandidate);
            usedVehicles.insert(selectedCandidate.vehicle_id);

            // For mixed vehicles, mark the entire group as used
            if (selectedCandidate.type == Assignment::Type::K3) {
                usedVehicleGroups.insert(selectedCandidate.group_id);
            }

            remainingFrigoboxes -= selectedCandidate.frigoboxes;
            unservedRoutes.erase(selectedRoute);
        }
    }

    // PHASE 2: MIXED-ONLY phase
    bool hasMixedCandidate = false;
    for (size_t r : unservedRoutes) {
        for (const auto& candidate : candidateSets[r]) {
            if (isCandidateAvailable(candidate) && candidate.frigoboxes == 0) {
                hasMixedCandidate = true;
                break;
            }
        }
        if (hasMixedCandidate) break;
    }

    if (hasMixedCandidate) {
        while (!unservedRoutes.empty()) {
            // Check if we still have mixed-feasible candidates
            bool hasFeasibleCandidate = false;
            for (size_t r : unservedRoutes) {
                for (const auto& candidate : candidateSets[r]) {
                    if (isCandidateAvailable(candidate) && candidate.frigoboxes == 0) {
                        hasFeasibleCandidate = true;
                        break;
                    }
                }
                if (hasFeasibleCandidate) break;
            }

            if (!hasFeasibleCandidate) break;

            // Find all routes that have at least one mixed-feasible candidate
            std::vector<size_t> feasibleRoutes;
            for (size_t r : unservedRoutes) {
                for (const auto& candidate : candidateSets[r]) {
                    if (isCandidateAvailable(candidate) && candidate.frigoboxes == 0) {
                        feasibleRoutes.push_back(r);
                        break;
                    }
                }
            }

            if (feasibleRoutes.empty()) break;

            // Select route uniformly from feasible routes
            std::uniform_int_distribution<size_t> routeDist(0, feasibleRoutes.size() - 1);
            size_t selectedRoute = feasibleRoutes[routeDist(gen)];

            // Find best candidate for selected route (minimum residual capacity)
            std::vector<Candidate> feasibleCandidates;
            int minResidual = std::numeric_limits<int>::max();

            for (const auto& candidate : candidateSets[selectedRoute]) {
                if (isCandidateAvailable(candidate) && candidate.frigoboxes == 0) {
                    minResidual = std::min(minResidual, candidate.residual_capacity);
                    feasibleCandidates.push_back(candidate);
                }
            }

            // Filter candidates with minimum residual capacity
            std::vector<Candidate> bestCandidates;
            for (const auto& candidate : feasibleCandidates) {
                if (candidate.residual_capacity == minResidual) {
                    bestCandidates.push_back(candidate);
                }
            }

            if (bestCandidates.empty()) {
                unservedRoutes.erase(selectedRoute);
                continue;
            }

            // Select candidate uniformly from best candidates
            std::uniform_int_distribution<size_t> candidateDist(0, bestCandidates.size() - 1);
            Candidate selectedCandidate = bestCandidates[candidateDist(gen)];

            // Commit assignment
            assignments[selectedRoute] = candidateToAssignment(selectedCandidate);
            usedVehicles.insert(selectedCandidate.vehicle_id);

            // For mixed vehicles, mark the entire group as used
            if (selectedCandidate.type == Assignment::Type::K3) {
                usedVehicleGroups.insert(selectedCandidate.group_id);
            }

            unservedRoutes.erase(selectedRoute);
        }
    }

    // Final check
    if (unservedRoutes.empty()) {
        return assignments;
    }
    else {
        return std::nullopt;
    }
}

VehicleOptimizer::Assignment VehicleOptimizer::candidateToAssignment(const Candidate& candidate) const {
    Assignment assignment;
    assignment.type = candidate.type;
    assignment.vehicle_id = candidate.vehicle_id;

    if (candidate.type == Assignment::Type::K3) {
        assignment.frigoboxes = std::nullopt;
    }
    else {
        assignment.frigoboxes = candidate.frigoboxes;
    }

    assignment.empty_space = candidate.residual_capacity;
    return assignment;
}

int VehicleOptimizer::ceilDiv(int dividend, int divisor) const {
    return (dividend + divisor - 1) / divisor;
}

std::vector<VehicleOptimizer::RouteDemand> VehicleOptimizer::calculateRouteDemands(
    const std::vector<Vehicle>& vehicles
) {
    std::vector<RouteDemand> demands;
    for (const auto& vehicle : vehicles) {
        if (!vehicle.route.empty()) {
            int totalFresh = 0, totalFrozen = 0;
            for (const auto& rp : vehicle.route) {
                totalFresh += rp.customer.fresh;
                totalFrozen += rp.customer.frozen;
            }
            demands.push_back({ totalFresh, totalFrozen });
        }
    }
    return demands;
}

