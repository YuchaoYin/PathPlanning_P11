#ifndef VEHICLE_H
#define VEHICLE_H

#include

class Vehicle {

public:
    float s;
    float v;
    float a;
    int l;
    sdt::string state;

    Vehicle();
    Vehicle(float s, float v, float a, int l, sdt::string state="KL");
    virtual ~Vehicle;

    vector<string> successorStates();
    vector<Vehicle> getBestTrajectory(vector<Vehicle> sensorFusion);
    vector<Vehicle> generateTrajectory(std::string nextState, vector<Vehicle> sensorFusion);
    vector<Vehicle> selectTargetObject(vector<Vehicle> sensorFusion, int l, Vehicle targetObject);
}
#endif // VEHICLE_H
