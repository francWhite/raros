#include "RoboStepper.h"
#include "Arduino.h"

RoboStepper::RoboStepper() {
  // use the default values
  this->wheelRadius = WHEEL_RADIUS;
  this->wheelsDistance = WHEELS_DISTANCE;
}

RoboStepper::~RoboStepper(){
}

double RoboStepper::getWheelsDistance() { return this->wheelsDistance; }

void RoboStepper::setWheelsDistance(double distance) { this->wheelsDistance = distance; }

double RoboStepper::getWheelRadius() { return this->wheelRadius; }

void RoboStepper::setWheelRadius(double radius) { this->wheelRadius = radius; }

int RoboStepper::distanceInMeterToSteps(double distanceInMeter)
{
    int steps = 0;

    // calculate the circumference of the wheel
    double circumference = 2 * wheelRadius * PI;
    // calculate the distance for one step
    double distOneStep = circumference / STEPS_PER_REVOLUTION;
    // calculate steps needed for the specified distance
    steps = distanceInMeter / distOneStep;

    return steps;
}

double RoboStepper::stepsToDistanceInMeter(int steps)
{
    double distance = 0;

    // calculate the circumference of the wheel.
    double circumference = 2 * wheelRadius * PI;
    // calculate the distance for one step
    double distOneStep = circumference / STEPS_PER_REVOLUTION;
    // calculate distance for specified steps
    distance = steps * distOneStep;

    return distance;
}

int RoboStepper::calculateAcceleration(int startSpeed, int maxSpeed, int timePeriod)
{
    int acceleration = 0;

    // calculate delta
    int delta = maxSpeed - startSpeed;

    // calculate the acceleration (the slope)
    acceleration = (timePeriod * STEPS_PER_REVOLUTION) / delta;

    return acceleration;
}

double RoboStepper::calculateCircleArc(double radius, double angle){

  double angleInRad = angle * PI / 180;
  double arc = angleInRad * radius;
  return arc;
}

int RoboStepper::stepsAccelerationRange(float distanceInMeter)
{
    int accelerationRange = 0;

    int steps = distanceInMeterToSteps(distanceInMeter);

    if(steps >= 2 * STEPS_PER_REVOLUTION) {
        accelerationRange = STEPS_PER_REVOLUTION;
    } else {
        accelerationRange = steps / 2;
    }

    return accelerationRange;
}

int RoboStepper::stepsForRotation(double angleInDegree)
{
    double angleInRad = (angleInDegree * PI) / 180;
    double steps = (angleInRad * wheelsDistance * STEPS_PER_REVOLUTION) / (4 * PI * wheelRadius);
    return (int)steps;
}

int RoboStepper::stepsForTurning(double angleInDegree) { 
  return 2 * stepsForRotation(angleInDegree);
}
