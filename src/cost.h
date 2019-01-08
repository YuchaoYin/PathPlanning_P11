#ifndef COST_H
#define COST_H
#include "vehicle.h"
#include <vector>
using namespace std;

float calculateCost(const Vehicle &vehicle, const vector<Vehicle> &sensorFusion, const vector<Vehicle> &trajectory);
float collisionCost(const Vehicle &vehicle, const vector<Vehicle> &sensorFusion, const vector<Vehicle> &trajectory);
float speedCost(const Vehicle &vehicle, const vector<Vehicle> &sensorFusion, const vector<Vehicle> &trajectory);
float laneSpeed(const Vehicle &vehicle, const vector<Vehicle> &sensorFusion, int l);

#endif // COST_H
