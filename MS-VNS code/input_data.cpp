#include "input_data.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <stdexcept>
#include <iostream>  // Add this if missing

using namespace std;

void read_input(const string& filename, Config& config, vector<Customer>& customers, vector<Vehicle>& vehicles) {
    ifstream file(filename);
    string line;
    vector<string> vehicle_lines;

    // Read vehicle section
    while (getline(file, line)) {
        if (line.find("VEHICLE_ID") != string::npos) {
            while (getline(file, line)) {
                if (line.empty()) break;
                vehicle_lines.push_back(line);
            }
            break;
        }
    }


    // Parse vehicles
    for (const auto& v_line : vehicle_lines) {
        stringstream ss(v_line);
        int  max_frigo;
        string id_str, type, fc_str, frc_str;
        ss >> id_str >> type >> fc_str >> frc_str >> max_frigo;

        int fc = 0, frc = 0;
        if (fc_str != "NONE") fc = stoi(fc_str);
        if (frc_str != "NONE") frc = stoi(frc_str);

        vehicles.emplace_back(id_str, type, fc, frc, max_frigo);
    }

    //std::cout << "=== VEHICLE DATA ===\n";
    //for (const auto& v : vehicles) {
    //    std::cout << "Vehicle ID: " << v.id
    //        << " | Type: " << v.type
    //        << " | Fresh Cap: " << v.fresh_cap
    //        << " | Frozen Cap: " << v.frozen_cap
    //        << " | Max Frigoboxes: " << v.max_frigo << "\n";
    //}
    //std::cout << "\n";


    // Read frigobox data
    while (getline(file, line)) {
        if (line.find("FRIGOBOX DATA") != string::npos) break;
    }
    while (getline(file, line)) {
        if (line.empty()) break;
        size_t colon = line.find(':');
        if (colon == string::npos) continue;

        string key = line.substr(0, colon);
        string value = line.substr(colon + 1);
        value.erase(remove(value.begin(), value.end(), ' '), value.end());

        if (key == "TOTAL_AVAILABLE_FRIGOBOXES") {
            config.total_frigo = stoi(value);
        }
        else if (key == "CAPACITY_PER_FRIGOBOX") {
            config.frigobox_cap = stoi(value);
        }
        else if (key == "WEIGHT_PER_FRIGOBOX") {
            config.frigobox_weight = stoi(value);
        }
    }
    //std::cout << "=== FRIGOBOX DATA ===\n";
    //std::cout << "Total Available Frigoboxes: " << config.total_frigo << "\n";
    //std::cout << "Capacity per Frigobox: " << config.frigobox_cap << "\n";
    //std::cout << "Weight per Frigobox: " << config.frigobox_weight << "\n\n";

    // Read customers
    while (getline(file, line)) {
        if (line.find("CUST NO.") != string::npos) break;
    }


    bool depot_found = false;
    while (getline(file, line)) {
        if (line.empty()) continue;
        Customer c;
        stringstream ss(line);
        ss >> c.id >> c.x >> c.y >> c.ready >> c.due >> c.service >> c.frozen >> c.fresh;

        if (c.id == 0) {
            config.depot_x = c.x;
            config.depot_y = c.y;
            config.max_route_duration = c.due;
            depot_found = true;
        }
        customers.push_back(c);
    }
    //std::cout << "=== CUSTOMER DATA ===\n";
    //for (const auto& c : customers) {
    //    std::cout << "Customer ID: " << c.id
    //        << " | Fresh: " << c.fresh
    //        << " | Frozen: " << c.frozen
    //        << " | Time Window: [" << c.ready << ", " << c.due << "]"
    //        << " | Service Time: " << c.service << "\n";
    //}
    //std::cout << "\n";

    if (!depot_found) throw runtime_error("Depot not found in input");
}


void print_solution(const Config& config, const vector<Customer>& customers, const vector<Vehicle>& vehicles) {
    int total_served = 0;
    double total_distance = 0.0;

    auto calculate_distance = [&config](const Vehicle& v) {
        if (v.route.empty()) return 0.0;
        double distance = 0.0;
        distance += sqrt(pow(v.route[0].customer.x - config.depot_x, 2) +
            pow(v.route[0].customer.y - config.depot_y, 2));
        for (size_t i = 1; i < v.route.size(); ++i) {
            distance += sqrt(pow(v.route[i].customer.x - v.route[i - 1].customer.x, 2) +
                pow(v.route[i].customer.y - v.route[i - 1].customer.y, 2));
        }
        distance += sqrt(pow(config.depot_x - v.route.back().customer.x, 2) +
            pow(config.depot_y - v.route.back().customer.y, 2));
        return distance;
        };

    for (const auto& c : customers) {
        if (c.id != 0 && c.served) total_served++;
    }

    cout << "=== Solution Summary ===\n";
    cout << "Customers served: " << total_served << "/" << (customers.size() - 1) << "\n";
    //cout << "Remaining frigoboxes: " << config.total_frigo << "\n\n";

    for (const auto& v : vehicles) {
        if (v.route.empty()) continue;
        double dist = calculate_distance(v);
        total_distance += dist;

        cout << "Vehicle " << v.id << " (" << v.type << ")\n";
        cout << "Route: 0 -> ";
        for (const auto& rp : v.route) cout << rp.customer.id << " -> ";
        cout << "0\nDistance: " << dist << "\n";
        cout << "Fresh: " << v.current_fresh << "/" << v.fresh_cap << " | ";
        cout << "Frozen: " << v.current_frozen << "/" << v.frozen_cap << "\n";
        cout << "Frigoboxes used: " << v.frigoboxes << "\n\n";
    }

    cout << "Total distance: " << total_distance << "\n";
    cout << "Unserved customers: ";
    for (const auto& c : customers) {
        if (!c.served && c.id != 0) cout << c.id << " ";
    }
    cout << "\n";
}