#include "constructive.h"
#include <algorithm>
#include <random>
#include <cmath>
#include <numeric>
#include <climits>
#include <set>
#include <unordered_map>
#include <unordered_set>

using namespace std;

struct InsertionInfo {
    int position;
    double cost;
    double arrival_time;
    bool feasible;
};

// Helper functions
static double euclidean(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

static InsertionInfo find_best_insertion(Vehicle& v, Customer& c, const Config& config) {
    InsertionInfo best{ -1, numeric_limits<double>::max(), 0.0, false };

    if (v.route.empty()) {
        double travel_time = euclidean(config.depot_x, config.depot_y, c.x, c.y);
        double arrival = travel_time;
        double departure = max(arrival, (double)c.ready) + c.service;
        double return_time = euclidean(c.x, c.y, config.depot_x, config.depot_y);
        double total_duration = departure + return_time;

        if (arrival <= c.due && total_duration <= config.max_route_duration) {
            best = { 0, travel_time + return_time, arrival, true };
        }
        return best;
    }

    for (size_t i = 0; i <= v.route.size(); ++i) {
        double prev_departure = (i == 0) ? 0 : v.route[i - 1].departure_time;
        int prev_x = (i == 0) ? config.depot_x : v.route[i - 1].customer.x;
        int prev_y = (i == 0) ? config.depot_y : v.route[i - 1].customer.y;

        // Calculate arrival time at new customer
        double travel_time_to = euclidean(prev_x, prev_y, c.x, c.y);
        double arrival_to = prev_departure + travel_time_to;
        arrival_to = max(arrival_to, (double)c.ready);
        if (arrival_to > c.due) continue;

        double departure_to = arrival_to + c.service;

        // Create temporary route with inserted customer
        vector<RoutePoint> temp_route = v.route;
        temp_route.insert(temp_route.begin() + i, RoutePoint{ c, arrival_to, departure_to });

        // Simulate subsequent customers' timing in TEMPORARY ROUTE
        bool feasible = true;
        double current_time = departure_to;
        for (size_t j = i + 1; j < temp_route.size(); ++j) {
            // Travel time from previous customer in TEMPORARY route
            double travel_time = euclidean(
                temp_route[j - 1].customer.x, temp_route[j - 1].customer.y,
                temp_route[j].customer.x, temp_route[j].customer.y
            );

            current_time += travel_time;
            current_time = max(current_time, (double)temp_route[j].customer.ready);

            if (current_time > temp_route[j].customer.due) {
                feasible = false;
                break;
            }
            current_time += temp_route[j].customer.service;
        }

        // Check total route duration (including return to depot)
        if (feasible) {
            double final_return = euclidean(
                temp_route.back().customer.x, temp_route.back().customer.y,
                config.depot_x, config.depot_y
            );
            if (current_time + final_return > config.max_route_duration) {
                feasible = false;
            }
        }

        // Calculate insertion cost if feasible
        if (feasible) {
            double travel_time_from = (i == v.route.size())
                ? euclidean(c.x, c.y, config.depot_x, config.depot_y)
                : euclidean(c.x, c.y, v.route[i].customer.x, v.route[i].customer.y);

            double cost = travel_time_to + travel_time_from;
            if (cost < best.cost) {
                best = { static_cast<int>(i), cost, arrival_to, true };
            }
        }
    }
    return best;
}

bool check_capacity(Vehicle& v, Customer& c, const Config& config, int& remaining_frigo) {
    if (v.type == "Fresh") {
        int required_frigo = ceil((v.current_frozen + c.frozen) / static_cast<double>(config.frigobox_cap));
        if (required_frigo > v.max_frigo)
            return false;

        // Check if additional frigoboxes can be allocated from total remaining ones
        int additional = required_frigo - v.frigoboxes;
        if (remaining_frigo - additional < 0)
            return false;

        // Ensure fresh capacity remains valid
        int fresh_available = v.fresh_cap - (required_frigo * config.frigobox_weight);
        return (v.current_fresh + c.fresh) <= fresh_available;
    }

    if (v.type == "Frozen") {
        int required_frigo = ceil((v.current_fresh + c.fresh) / static_cast<double>(config.frigobox_cap));
        if (required_frigo > v.max_frigo)
            return false;

        int additional = required_frigo - v.frigoboxes;
        if (remaining_frigo - additional < 0)
            return false;

        int frozen_available = v.frozen_cap - (required_frigo * config.frigobox_weight);
        return (v.current_frozen + c.frozen) <= frozen_available;
    }

    // Mixed vehicle (no frigobox constraints)
    return (v.current_fresh + c.fresh <= v.fresh_cap) &&
        (v.current_frozen + c.frozen <= v.frozen_cap);
}

static void insert_customer(Vehicle& v, Customer& c, int pos, double arrival_time, const Config& config, int& remaining_frigo)
{
    RoutePoint new_point{ c, arrival_time, max(arrival_time, (double)c.ready) + c.service };
    v.route.insert(v.route.begin() + pos, new_point);

    // Update capacities and frigoboxes
    if (v.type == "Fresh") {
        int prev_frigo = v.frigoboxes;
        int required_frigo = ceil((v.current_frozen + c.frozen) / static_cast<double>(config.frigobox_cap));

        if (required_frigo > v.max_frigo || (required_frigo - prev_frigo) > remaining_frigo) {
            v.route.erase(v.route.begin() + pos);
            return;
        }

        remaining_frigo -= (required_frigo - prev_frigo);
        v.frigoboxes = required_frigo;
    }
    else if (v.type == "Frozen") {
        int prev_frigo = v.frigoboxes;
        int required_frigo = ceil((v.current_fresh + c.fresh) / static_cast<double>(config.frigobox_cap));

        if (required_frigo > v.max_frigo || (required_frigo - prev_frigo) > remaining_frigo) {
            v.route.erase(v.route.begin() + pos);
            return;
        }

        remaining_frigo -= (required_frigo - prev_frigo);
        v.frigoboxes = required_frigo;
    }

    v.current_fresh += c.fresh;
    v.current_frozen += c.frozen;
    c.served = true;

    // Update arrival and departure times for subsequent customers
    for (size_t j = pos; j < v.route.size(); ++j) {
        if (j == 0) {
            // Handle first customer after depot
            double travel_time = euclidean(config.depot_x, config.depot_y, v.route[j].customer.x, v.route[j].customer.y);
            v.route[j].arrival_time = travel_time;
            v.route[j].departure_time = max(v.route[j].arrival_time, (double)v.route[j].customer.ready) + v.route[j].customer.service;
        }
        else {
            // Calculate based on previous customer's departure time
            double prev_departure = v.route[j - 1].departure_time;
            double travel_time = euclidean(v.route[j - 1].customer.x, v.route[j - 1].customer.y,
                v.route[j].customer.x, v.route[j].customer.y);
            double arrival = prev_departure + travel_time;
            arrival = max(arrival, (double)v.route[j].customer.ready);
            v.route[j].arrival_time = arrival;
            v.route[j].departure_time = arrival + v.route[j].customer.service;
        }
    }
}

// Function to completely fill a single vehicle
static bool fill_vehicle_completely(Vehicle& vehicle, vector<Customer*>& unserved,
    const Config& config, mt19937& gen, int& remaining_frigo) {
    bool vehicle_full = false;
    bool made_progress = false;

    while (!vehicle_full) {
        bool inserted_any = false;

        // Shuffle unserved customers to avoid bias
        shuffle(unserved.begin(), unserved.end(), gen);

        // Try to insert each unserved customer
        for (auto it = unserved.begin(); it != unserved.end(); ) {
            Customer* customer = *it;

            // Skip if customer is already served (safety check)
            if (customer->served) {
                ++it;
                continue;
            }

            // Check capacity constraints
            if (!check_capacity(vehicle, *customer, config, remaining_frigo)) {
                ++it;
                continue;
            }

            // Find best insertion position
            InsertionInfo best_insertion = find_best_insertion(vehicle, *customer, config);

            if (best_insertion.feasible) {
                // Insert the customer
                insert_customer(vehicle, *customer, best_insertion.position,
                    best_insertion.arrival_time, config, remaining_frigo);

                // Remove from unserved list
                it = unserved.erase(it);
                inserted_any = true;
                made_progress = true;
            }
            else {
                ++it;
            }
        }

        // If we didn't insert any customer in this pass, the vehicle is full
        if (!inserted_any) {
            vehicle_full = true;
        }
    }

    return made_progress;
}

void constructSolution(Config& config, vector<Customer>& customers, vector<Vehicle>& vehicles) {
    vector<Customer*> unserved;
    for (auto& c : customers) {
        if (c.id != 0 && !c.served) unserved.push_back(&c);
    }

    random_device rd;
    mt19937 gen(rd());
    int remaining_frigo = config.total_frigo;

    // Track which mixed vehicle groups we've already used
    unordered_set<char> used_mixed_groups;

    // ===== PHASE 1: RANDOM VEHICLE SELECTION =====

    // Create a list of all vehicles and shuffle them randomly
    vector<Vehicle*> all_vehicles;
    for (auto& vehicle : vehicles) {
        all_vehicles.push_back(&vehicle);
    }
    shuffle(all_vehicles.begin(), all_vehicles.end(), gen);

    // Process vehicles in random order
    for (auto vehicle_ptr : all_vehicles) {
        if (unserved.empty()) break;

        // Check if this is a mixed vehicle from a group we've already used
        if (vehicle_ptr->type == "Mixed") {
            char group_prefix = vehicle_ptr->id[0];
            if (used_mixed_groups.find(group_prefix) != used_mixed_groups.end()) {
                // Skip this vehicle - we already used one from its group
                continue;
            }
        }

        // Fill this vehicle completely
        fill_vehicle_completely(*vehicle_ptr, unserved, config, gen, remaining_frigo);

        // If it was a mixed vehicle, mark its group as used
        if (vehicle_ptr->type == "Mixed") {
            char group_prefix = vehicle_ptr->id[0];
            used_mixed_groups.insert(group_prefix);
        }
    }

    // ===== PHASE 2: USE REMAINING VEHICLES (INCLUDING SKIPPED MIXED ONES) =====
    if (!unserved.empty()) {
        // Create a new shuffled list of all vehicles for phase 2
        vector<Vehicle*> phase2_vehicles;
        for (auto& vehicle : vehicles) {
            phase2_vehicles.push_back(&vehicle);
        }
        shuffle(phase2_vehicles.begin(), phase2_vehicles.end(), gen);

        // Try to insert remaining customers into any vehicle
        for (auto vehicle_ptr : phase2_vehicles) {
            if (unserved.empty()) break;

            // In phase 2, we can use mixed vehicles from already-used groups
            // since the restriction only applies to the first phase

            // Try to insert each remaining customer
            for (auto it = unserved.begin(); it != unserved.end(); ) {
                Customer* customer = *it;

                if (customer->served) {
                    ++it;
                    continue;
                }

                if (!check_capacity(*vehicle_ptr, *customer, config, remaining_frigo)) {
                    ++it;
                    continue;
                }

                InsertionInfo best_insertion = find_best_insertion(*vehicle_ptr, *customer, config);

                if (best_insertion.feasible) {
                    insert_customer(*vehicle_ptr, *customer, best_insertion.position,
                        best_insertion.arrival_time, config, remaining_frigo);
                    it = unserved.erase(it);
                }
                else {
                    ++it;
                }
            }
        }
    }

    // Final cleanup
    unserved.erase(
        remove_if(unserved.begin(), unserved.end(), [](Customer* c) { return c->served; }),
        unserved.end()
    );
}