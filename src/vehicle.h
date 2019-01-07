#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <string>

using namespace std;

class Vehicle {

public:
    float s;
    float v;
    float a;
    int l;
    std::string state;

    Vehicle();
    Vehicle(float s, float v, float a, int l, string state = "laneDriving");
    virtual ~Vehicle();

    vector<string> successorStates();
    vector<Vehicle> getBestTrajectory(vector<Vehicle> sensorFusion);
    vector<Vehicle> generateTrajectory(string state, vector<Vehicle> sensorFusion);
    bool selectTargetObject(vector<Vehicle> sensorFusion, int l, Vehicle &targetObject);
};
#endif // VEHICLE_H
