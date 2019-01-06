#ifndef COST_H
#define COST_H
#include "vehicle.h"

float calculateCost(const Vehicle &vehicle, const vector<Vehicle> &sensorFusion, const vector<Vehicle> &trajectory);
float collisionCost(const Vehicle &vehicle, const Vector<Vehicle> &sensorFusion, const vector<Vehicle> &trajectory);
float collisionSpeed(const Vehicle &vehicle, const vector<Vehicle> &sensorFusion, const vector<Vehicle> &trajectory);
float laneSpeed(const Vehicle &Vehicle, const vector<Vehicle> &sensorFusion, int l);

#endif // COST_H
