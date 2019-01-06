#include "cost.h"
#include <math.h>
#include <iterator>

float calculateCost(const Vehicle &vehicle, const vector<Vehicle> &sensorFusion, const vector<Vehicle> &trajectory){
    float cost = 0.0;

    //consider speed cost and collision cost
    //define weights
    float weightsSpeed = 10.0;
    float weightsCollision = 10000.0;

    cost = weightsCollision * collisionCost(vehicle, sensorFusion, trajectory) + weightsSpeed * collisionSpeed(vehicle, sensorFusion, trajectory);

    return cost;
}

float collisionCost(const Vehicle &vehicle, const Vector<Vehicle> &sensorFusion, const vector<Vehicle> &trajectory){
    //consider only lane change
    //define safety area
    float minSafety = vehicle.s - 10.0;
    float maxSafety = vehicle.s + 10.0;

    if (trajectory[1].state == "laneChangeRight" || trajectory[1].state == "laneChangeLeft" ){
        int targetLane = trajectory[1].l;
        for (vector<Vehicle>::const_iterator it = sensorFusion.begin(); it != sensorFuson.end(); ++it){
            object = *it;
            if (object.l == targetLane && object.s > minSafety && object.s < maxSafety){
                return 1.;
            }
        }
    }
    else {
        return 0.;
    }
    return 0.;
}

float collisionSpeed(const Vehicle &vehicle, const vector<Vehicle> &sensorFusion, const vector<Vehicle> &trajectory){
    int targetLane = trajectory[1].l;
    float speedLane = laneSpeed(vehicle, sensorFusion, targetLane);

    int intendedLane;
    if (trajectory[1].state == "laneDriving"){
        intendedLane = trajectory[1].l;
    }
    else if (trajectory[1].state == "prepareLaneChangeRight"){
        intendedLane = trajectory[1].l + 1;
    }
    else if (trajectory[1].state == "prepareLaneChangeLeft"){
        intendedLane = trajectory[1].l - 1;
    }

    float speedIntended = laneSpeed(vehicle, sensorFusion, intendedLane);

    float cost = (2.0 * 50.0 - speedIntended - speedLane)/50.0;

    return cost;

}

float laneSpeed(const Vehicle &Vehicle, const vector<Vehicle> &sensorFusion, int l){
    // assume we only consider the vehicles in front with s less than 30
    int minSTarget = this->s + 30;
    bool foundTargetObject = false;
    Vehicle object;
    Vehicle targetObject;

    for (vector<Vehicle>::iterator it = sensorFusion.begin(); it != sensorFuson.end(); ++it){
        object = *it;
        // choose the nearest one as target object
        if (object.s <= minSTarget && object.s > this->s && object.l == l){
            foundTargetObject = true;
            minSTarget = object.s;
            targetObject = object;
        }
    }

    if (foundTargetObject){
        return targetObject.v;
    }
    else {
        return 50.0 //target speed
    }
}
