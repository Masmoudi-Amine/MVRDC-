#pragma once
#ifndef VEHICLE_OPTIMIZER_H
#define VEHICLE_OPTIMIZER_H

#include "input_data.h"
#include <optional>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>
#include <random>
#include <set>
#include <unordered_map>
#include <unordered_set>


class VehicleOptimizer {
public:
    VehicleOptimizer(const Config& config);

    struct Assignment {
        enum class Type { K1, K2, K3 };
        Type type;
        std::string vehicle_id;
        std::optional<int> frigoboxes;
        int empty_space;
    };

    struct RouteDemand {
        int total_fresh;
        int total_frozen;
    };

    struct Candidate {
        Assignment::Type type;
        std::string vehicle_id;
        std::string group_id;  // For mixed vehicles - group identifier
        int frigoboxes;
        int residual_capacity;
    };

    // Mixed vehicle structures
    struct MixedVehicleConfig {
        std::string config_id;
        int fresh_cap;
        int frozen_cap;
    };

    struct MixedVehicleGroup {
        std::string group_id;
        std::vector<MixedVehicleConfig> configurations;
    };

    // Add back the selectVehicles function
    std::optional<std::vector<Assignment>> selectVehicles(
        const std::vector<RouteDemand>& routes,
        const std::vector<Vehicle>& vehicles,
        std::mt19937& gen
    );

    // Make selectBestVehicle public
    std::optional<std::vector<Assignment>> selectBestVehicle(
        const std::vector<std::vector<Candidate>>& candidateSets,
        std::mt19937& gen
    );

    static std::vector<RouteDemand> calculateRouteDemands(const std::vector<Vehicle>& vehicles);


    // Make vehicleCapabilityAnalysis public so it can be called from main.cpp
    std::vector<std::vector<Candidate>> vehicleCapabilityAnalysis(
        const std::vector<RouteDemand>& routes,
        const std::vector<Vehicle>& vehicles
    );

private:
    const int w_;
    const int F_;

    int ceilDiv(int dividend, int divisor) const;
    Assignment candidateToAssignment(const Candidate& candidate) const;

    // Helper to extract group ID from vehicle ID
    std::string extractGroupId(const std::string& vehicle_id) const;
};

#endif // VEHICLE_OPTIMIZER_H