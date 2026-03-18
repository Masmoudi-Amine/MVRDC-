#pragma once
#pragma once
#include <vector>
#include <string>
#include <string>
#include <iostream>

using namespace std;

struct Config {
    int depot_x;
    int depot_y;
    int max_route_duration;
    int total_frigo;
    int frigobox_cap;
    int frigobox_weight;
};

struct Customer {
    int id;
    int x, y;
    int ready, due, service;
    int frozen, fresh;
    bool served = false;
};

struct RoutePoint {
    Customer customer;
    double arrival_time;
    double departure_time;
};

struct Vehicle {
    std::string id; // Changed from int to string
    std::string type;
    int fresh_cap;
    int frozen_cap;
    int max_frigo;
    std::vector<RoutePoint> route;
    int current_fresh = 0;
    int current_frozen = 0;
    int frigoboxes = 0;

    std::vector<double> start_times; // start-of-service times for each customer in route
    std::vector<double> waiting_times; // waiting times at each customer


    // Constructor updated to use string for id
    Vehicle(std::string i, std::string t, int fc, int frc, int mf)
        : id(i), type(t), fresh_cap(fc), frozen_cap(frc), max_frigo(mf) {}
};

void read_input(const string& filename, Config& config, vector<Customer>& customers, vector<Vehicle>& vehicles);
void print_solution(const Config& config, const vector<Customer>& customers, const vector<Vehicle>& vehicles);
void printRouteDetails(const Config& config, const std::vector<Vehicle>& vehicles);
