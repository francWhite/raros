/*****************************************************************************
 * | File        :   robostepper.cpp
 * | Author      :   Jordan Sucur
 * | Function    :
 * | Info        :
 *                   The RoboStepper provides a few helper functions.
 *----------------
 * |This version:    V1.0
 * | Date        :   2022-08-24
 * | Info        :   Basic version
 *
 ******************************************************************************/
#ifndef ROBOSTEPPER_H
#define ROBOSTEPPER_H

// baudrate values
#define BAUD_RATE_SERIAL 9600
#define BAUD_RATE_SERIAL_USB 9600

// values for communication
#define STOP_MOVING 0

#define FORWARD 100
#define FORWARD_SPEED 101
#define FORWARD_DIST 102
#define FORWARD_DIST_SPEED 103
#define FORWARD_DIST_ACCEL 104

#define BACKWARD 200
#define BACKWARD_SPEED 201
#define BACKWARD_DIST 202
#define BACKWARD_DIST_SPEED 203
#define BACKWARD_DIST_ACCEL 204

#define ROTATE_LEFT_FOR_ANGLE 300
#define ROTATE_RIGHT_FOR_ANGLE 301

#define TURN_LEFT_FOR_ANGLE 400
#define TURN_LEFT_FOR_ANGLE_AND_RADIUS 401
#define TURN_RIGHT_FOR_ANGLE 402
#define TURN_RIGHT_FOR_ANGLE_AND_RADIUS 403

#define SET_SPEED 500
#define SET_SPEED_LEFT 501
#define SET_SPEED_RIGHT 502

// calibration part
#define UPDATE_WHEEL_RADIUS 900
#define MOVE_FORWARD_FOR_CALIBRATION 901
#define UPDATE_WHEELS_DISTANCE 902
#define ROTATE_LEFT_FOR_CALIBRATION 903

// power motor supplay management
#define ACTIVATE_MOTOR_POWER_SUPPLY_MANAGEMENT 1000

// movement speed (min, max and default)
#define MIN_MOVEMENT_SPEED 1
#define MAX_MOVEMENT_SPEED 60
#define DEFAULT_MOVEMENT_SPEED 60
#define DEFAULT_CALIBRATION_MOVEMENT_SPEED 20

// acceleration speed (min and max)
#define ACCELERATION_MIN_SPEED 10
#define ACCELERATION_MAX_SPEED 60

// rotation speed (default, the change of the value is currently not foreseen)
#define DEFAULT_ROTATION_SPEED 20
#define DEFAULT_CALIBRATION_ROTATION_SPEED 10

// acceleration range in revolutions (for example 4: it means, that
// the acceleration / de-accelerations is done during 4 revolutions)
#define ACCELERATION_RANGE_IN_REVOLUTIONS 4

// wheel distance (default: 0.22 m)
#define WHEELS_DISTANCE 0.22

// wheel radius (default: 0.04 m)
#define WHEEL_RADIUS 0.042112

// movement direction
#define FORWARD_DIR 1
#define BACKWARD_DIR 2

// rotation (turning) direction
#define LEFT_DIR 3
#define RIGHT_DIR 4

// microsteps resolution
#define FULL_STEP 1
#define HALF_STEP 0.5
#define QUARTER_STEP 0.25
#define EIGHTH_STEP 0.125

// steps per revolution (adjast it in case of an other microsteps resolution)
#define STEPS_PER_REVOLUTION (1600 / QUARTER_STEP)

// values for establishing of the connection
#define SYN -2
#define ACK "ACK"

// delimiter character
#define DELIMITER '#'

class RoboStepper
{
private:
    double wheelsDistance;
    double wheelRadius;

public:
    RoboStepper();
    ~RoboStepper();

    /**
     * @brief Returns steps for the specified distance in meter.
     * @param distanceInMeter
     * @return the number of steps needed for the specified distance
     */
    int distanceInMeterToSteps(double distanceInMeter);

    /**
     * @brief Returns the distance in meter for the specified steps.
     * @param steps the steps
     * @return the distance in meter
     */
    double stepsToDistanceInMeter(int steps);

    /**
     * @brief Returns the calculated acceleration as number of steps between two changes of speed.
     * @param startSpeed the start speed
     * @param maxSpeed the maximum allowed speed (target speed)
     * @param timePeriod the number of revolutions during witch the maximum is to be reached (e.g. 1, 2 etc.)
     * @return number of steps between two changes of speed
     */
    int calculateAcceleration(int startSpeed, int maxSpeed, int timePeriod);

    /**
     * @brief Returns the length of the circle arc for the specified angle and radius.
     * @param radius the radius in meter
     * @param angle the angle in degrees
     */
    double calculateCircleArc(double radius, double angle);

    /**
     * @brief Returns the number of steps that belong to the acceleration range.
     * @param distanceInMeter the distance in meter to be passed
     * @return the number of steps in the acceleration range
     */
    int stepsAccelerationRange(float distanceInMeter);

    /**
     * @brief Returns the number of steps neded to rotate for the specified angle.
     * @param angleInDegree the angle
     * @return  number of steps
     */
    int stepsForRotation(double angleInDegree);

    /**
     * @brief Returns the number of steps neded to turn for the specified angle.
     * @param angleInDegree the angle
     * @return  number of steps
     */
    int stepsForTurning(double angleInDegree);

    /**
     * @brief Returns the distance between the two wheels.
     * @return the distance
     */
    double getWheelsDistance();

    /**
     * @brief Sets the distance betweend the two wheels.
     * @param distance the distance
     */
    void setWheelsDistance(double distance);

    /**
     * @brief Returns the wheel radius.
     * @return the wheel radius
     */
    double getWheelRadius();

    /**
     * @brief Sets the wheel radius.
     * @param radius the radius
     */
    void setWheelRadius(double radius);
};

#endif // ROBOSTEPPER_H
