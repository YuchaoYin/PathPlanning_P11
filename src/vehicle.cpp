#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include <limits>

using namespace std;

Vehicle::Vehicle(){

}

Vehicle::Vehicle(float s, float v, float a, int d, string state){
    this->s = s;
    this->v = v;
    this->a = a;
    this->l = l;
    this->state = state;

}

Vehicle::~Vehicle(){

}

vector<Vehicle> Vehicle::getBestTrajectory(vector<Vehicle> sensorFusion){
    //find all successor states
    vector<string> states = successorStates();

    float minCost = numeric_limits<float>::max();
    float cost;
    vector<Vehicle> bestTrajectory;

    for(vector<string>::iterator it = states.begin(); it != states.end(); ++it){
        //generate trajectory
        vector<Vehicle> trajectory = generateTrajectory(*it, sensorFusion);
        if (trajectory.size() != 0){
            cost = calculateCost(*this, sensorFusion, trajectory);
            if (cost < minCost){
                bestTrajectory = trajectory;
                minCost = cost;
            }
        }
    }

    return bestTrajectory;
}

vector<string> Vehicle::successorStates(){
    vector<string> states;
    if (state == "laneDriving"){
        states.push_back("laneDriving");
        if (l == 0){
            states.push_back("prepareLaneChangeRight");
        }
        else if (l == 1){
            states.push_back("prepareLaneChangeRight");
            states.push_back("prepareLaneChangeLeft");
        }
        else if (l == 2){
            states.push_back("prepareLaneChangeLeft");
        }
    }
    else if (state == "prepareLaneChangeRight"){
        states.push_back("laneDriving");
        if (l != 2){
            states.push_back("prepareLaneChangeRight");
            states.push_back("laneChangeRight");
        }
    }
    else if (state == "prepareLaneChangeLeft"){
        states.push_back("laneDriving");
        if (l != 0){
            states.push_back("prepareLaneChangeLeft");
            states.push_back("laneChangeLeft");
        }
    }
}

vector<Vehicle> Vehicle::generateTrajectory(std::string nextState, vector<Vehicle> sensorFusion){
    vector<Vehicle> trajectory = {Vehicle(this->s, this->v, this->a, this->l, this->state)};
    float nextVel;
    float nextPos;
    float nextAccel;
    int nextLane;
    float setSpeed;
    Vehicle targetObject;

    setSpeed = 50.0;

    if (nextState == "laneDriving"){
        //generate trajectory of lane driving
        //select target object
        if (selectTargetObject(sensorFusion, this->l, targetObject)){
            nextVel = targetObject.v;
        }
        //no object in front, driving with selected speed
        else{
            setSpeed = 50.0;
            nextVel = setSpeed;
        }
        //calculate new acceleration and position
        nextAccel = nextVel - this->v;
        nextPos = this->s + nextVel + nextAccel/2.0;

        //trajectory
        trajectory.push_back(Vehicle(nextPos, nextVel, nextAccel, this->l, "laneDriving"));
    }
    else if (nextState == "prepareLaneChangeRight" || nextState == "prepareLaneChangeLeft"){
        //generate trajectory of prepare lane change
        if (nextState == "prepareLaneChangeRight"){
            nextLane = this->l + 1;
        }
        else{
            nextLane = this->l - 1;
        }
        //get the object in front in the ego lane
        if (selectTargetObject(sensorFusion, this->l, targetObject)){
            nextVel = targetObject.v;
        }
        //no object in front, driving with selected speed
        else{
            nextVel = setSpeed;
        }

        //check if target object exists in the new lane and choose the lowest velocity as target velocity
        if (selectTargetObject(sensorFusion, nextLane, targetObject)){
            if (targetObject.v < nextVel){
                nextVel = targetObject.v;
            }
        }

        //calculate new acceleration and position
        nextAccel = nextVel - this->v;
        nextPos = this->s + nextVel + nextAccel/2.0;

        //trajectory
        trajectory.push_back(Vehicle(nextPos, nextVel, nextAccel, this->l, nextState));

    }
    else{
        //generate trajectory of prepare lane change
        if (nextState == "laneChangeRight"){
            nextLane = this->l + 1;
        }
        else{
            nextLane = this->l - 1;
        }
        //select target object in the target lane
        if (selectTargetObject(sensorFusion, nextLane, targetObject)){
            nextVel = targetObject.v;
        }
        else{
            nextVel = setSpeed;
        }

        //calculate new acceleration and position
        nextAccel = nextVel - this->v;
        nextPos = this->s + nextVel + nextAccel/2.0;

        //trajectory
        trajectory.push_back(Vehicle(nextPos, nextVel, nextAccel, nextLane, nextState));
    }
}

bool Vehicle::selectTargetObject(vector<Vehicle> sensorFusion, int l, Vehicle &targetObject){
    // assume we only consider the vehicles in front with s less than 30
    int minSTarget = this->s + 30;
    bool foundTargetObject = false;
    Vehicle object;

    for (vector<Vehicle>::iterator it = sensorFusion.begin(); it != sensorFusion.end(); ++it){
        object = *it;
        // choose the nearest one as target object
        if (object.s <= minSTarget && object.s > this->s && object.l == l){
            foundTargetObject = true;
            minSTarget = object.s;
            targetObject = object;
        }
    }
    return foundTargetObject;
}
