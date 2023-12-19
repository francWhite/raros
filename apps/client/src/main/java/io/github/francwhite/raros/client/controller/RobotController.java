package io.github.francwhite.raros.client.controller;

import io.github.francwhite.raros.client.connector.Color;
import io.github.francwhite.raros.client.connector.Status;

public interface RobotController {

  /**
   * Gets the current status of the robot.
   *
   * @return The current status of the robot.
   */
  Status getStatus();

  /**
   * Enables the magnet.
   */
  void enableMagnet();

  /**
   * Disables the magnet.
   */
  void disableMagnet();

  /**
   * Plays a beep sound.
   */
  void beep();

  /**
   * Plays a tone. This method will block until the tone is finished.
   *
   * @param frequency The frequency of the tone in Hz.
   * @param duration  The duration of the tone in ms.
   */
  void playTone(int frequency, int duration);

  /**
   * Plays a tone. This method will return immediately, additional calls to this method will queue the requested tone.
   *
   * @param frequency The frequency of the tone in Hz.
   * @param duration  The duration of the tone in ms.
   */
  void playToneAsync(int frequency, int duration);

  /**
   * Reads the current color of the color sensor.
   *
   * @return The last color read from the color sensor.
   */
  Color getColor();

  /**
   * Reads the current distance to the nearest object in front of the robot.
   *
   * @return Distance in cm. If no object is detected infinity is returned.
   */
  float getDistanceFront();

  /**
   * Reads the current distance to the nearest object in front of the robot.
   *
   * @param angle The angle to rotate the range sensor to. The angle must be between -90 and 90.
   * @return Distance in cm. If no object is detected infinity is returned.
   */
  float getDistanceFront(int angle);

  /**
   * Reads the current distance to the nearest object behind the robot.
   *
   * @return Distance in cm. If no object is detected infinity is returned.
   */
  float getDistanceBack();

  /**
   * Rotates the range sensor (front) to a given angle.
   *
   * @param angle The angle to rotate the range sensor to. The angle must be between -90 and 90.
   */
  void rotateRangeSensor(int angle);

  /**
   * Stops any movement of the robot.
   */
  void stopMovement();

  /**
   * Moves the robot forward by the specified distance.
   * This method will block until the robot has reached the specified distance or an obstacle is detected.
   *
   * @param distance The distance to move the robot forward in m.
   */
  void moveForward(double distance);

  /**
   * Moves the robot forward by the specified distance.
   * This method will block until the robot has reached the specified distance or an obstacle is detected.
   *
   * @param distance The distance to move the robot forward in m.
   * @param speed    The speed at which the robot moves forward in % (0-100%).
   */
  void moveForward(double distance, int speed);

  /**
   * Moves the robot forward by the specified distance using acceleration.
   * This method will block until the robot has reached the specified distance or an obstacle is detected.
   *
   * @param distance   The distance to move the robot forward in m.
   * @param startSpeed The speed at which the robot starts moving forward in % (0-100%).
   * @param endSpeed   The desired end speed % (0-100%).
   */
  void moveForward(double distance, int startSpeed, int endSpeed);

  /**
   * Moves the robot forward indefinitely.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   */
  void moveForwardAsync();

  /**
   * Moves the robot forward by the specified distance.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param distance The distance to move the robot forward in m.
   */
  void moveForwardAsync(double distance);

  /**
   * Moves the robot forward by the specified distance.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param distance The distance to move the robot forward in m.
   * @param speed    The speed to move the robot forward in % (0-100%).
   */
  void moveForwardAsync(double distance, int speed);

  /**
   * Moves the robot forward by the specified distance using acceleration.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param distance   The distance to move the robot forward in m.
   * @param startSpeed The speed at which the robot starts moving forward in % (0-100%).
   * @param endSpeed   The desired end speed % (0-100%).
   */
  void moveForwardAsync(double distance, int startSpeed, int endSpeed);


  /**
   * Moves the robot backward by the specified distance.
   * This method will block until the robot has reached the specified distance or an obstacle is detected.
   *
   * @param distance The distance to move the robot backward in m.
   */
  void moveBackward(double distance);

  /**
   * Moves the robot backward by the specified distance.
   * This method will block until the robot has reached the specified distance or an obstacle is detected.
   *
   * @param distance The distance to move the robot backward in m.
   * @param speed    The speed at which the robot moves forward in % (0-100%).
   */
  void moveBackward(double distance, int speed);

  /**
   * Moves the robot backward by the specified distance using acceleration.
   * This method will block until the robot has reached the specified distance or an obstacle is detected.
   *
   * @param distance   The distance to move the robot backward in m.
   * @param startSpeed The speed at which the robot starts moving backward in % (0-100%).
   * @param endSpeed   The desired end speed % (0-100%).
   */
  void moveBackward(double distance, int startSpeed, int endSpeed);

  /**
   * Moves the robot backward indefinitely.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   */
  void moveBackwardAsync();

  /**
   * Moves the robot backward by the specified distance.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param distance The distance to move the robot backward in m.
   */
  void moveBackwardAsync(double distance);

  /**
   * Moves the robot backward by the specified distance.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param distance The distance to move the robot backward in m.
   * @param speed    The speed at which the robot moves forward in % (0-100%).
   */
  void moveBackwardAsync(double distance, int speed);

  /**
   * Moves the robot backward by the specified distance using acceleration.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param distance   The distance to move the robot backward in m.
   * @param startSpeed The speed at which the robot starts moving backward in % (0-100%).
   * @param endSpeed   The desired end speed % (0-100%).
   */
  void moveBackwardAsync(double distance, int startSpeed, int endSpeed);

  /**
   * Rotates the robot to the left by the specified angle.
   * This method will block until the robot has rotated by the specified angle.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void rotateLeft(double angle);

  /**
   * Rotates the robot to the left by the specified angle.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void rotateLeftAsync(double angle);

  /**
   * Rotates the robot to the right by the specified angle.
   * This method will block until the robot has rotated by the specified angle.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void rotateRight(double angle);

  /**
   * Rotates the robot to the right by the specified angle.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void rotateRightAsync(double angle);

  /**
   * Turns the robot to the left by the specified angle.
   * This method will block until the robot has rotated by the specified angle.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void turnLeft(double angle);

  /**
   * Turns the robot to the left by the specified angle.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void turnLeftAsync(double angle);


  /**
   * Turns the robot to the left by the specified angle and radius.
   * This method will block until the robot has rotated by the specified angle.
   *
   * @param angle  The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   * @param radius The radius of the turn in m.
   */
  void turnLeft(double angle, double radius);

  /**
   * Turns the robot to the left by the specified angle and radius.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param angle  The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   * @param radius The radius of the turn in m.
   */
  void turnLeftAsync(double angle, double radius);

  /**
   * Turns the robot to the right by the specified angle.
   * This method will block until the robot has rotated by the specified angle.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void turnRight(double angle);

  /**
   * Turns the robot to the right by the specified angle.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void turnRightAsync(double angle);

  /**
   * Turns the robot to the right by the specified angle and radius.
   * This method will block until the robot has rotated by the specified angle.
   *
   * @param angle  The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   * @param radius The radius of the turn in m.
   */
  void turnRight(double angle, double radius);

  /**
   * Turns the robot to the right by the specified angle and radius.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param angle  The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   * @param radius The radius of the turn in m.
   */
  void turnRightAsync(double angle, double radius);

  /**
   * Captures an image from the camera.
   *
   * @return String The captured image as a base64 encoded string.
   */
  String captureImage();

  /**
   * Captures an image from the camera.
   *
   * @param angleHorizontal The horizontal angle to rotate the camera to. The angle must be between -90 and 90.
   * @param angleVertical   The vertical angle to rotate the camera to. The angle must be between -90 and 90.
   * @return String The captured image as a base64 encoded string.
   */
  String captureImage(int angleHorizontal, int angleVertical);

  /**
   * Rotates the camera to a given angle.
   *
   * @param angleHorizontal The horizontal angle to rotate the camera to. The angle must be between -90 and 90.
   * @param angleVertical   The vertical angle to rotate the camera to. The angle must be between -90 and 90.
   */
  void rotateCamera(int angleHorizontal, int angleVertical);
}