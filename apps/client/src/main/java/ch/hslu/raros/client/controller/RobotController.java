package ch.hslu.raros.client.controller;

import ch.hslu.raros.client.connector.Color;
import ch.hslu.raros.client.connector.Status;

//TODO (entire project): rename methods to match the camelCase naming convention
public interface RobotController {

  /**
   * Gets the current status of the robot.
   *
   * @return The current status of the robot.
   */
  Status GetStatus();

  /**
   * Enables the magnet.
   */
  void EnableMagnet();

  /**
   * Disables the magnet.
   */
  void DisableMagnet();

  /**
   * Plays a beep sound.
   */
  void Beep();

  /**
   * Plays a tone. This method will block until the tone is finished.
   *
   * @param frequency The frequency of the tone in Hz.
   * @param duration  The duration of the tone in ms.
   */
  void PlayTone(int frequency, int duration);

  /**
   * Plays a tone. This method will return immediately, additional calls to this method will queue the requested tone.
   *
   * @param frequency The frequency of the tone in Hz.
   * @param duration  The duration of the tone in ms.
   */
  void PlayToneAsync(int frequency, int duration);

  /**
   * Reads the current color of the color sensor.
   *
   * @return The last color read from the color sensor.
   */
  Color GetColor();

  /**
   * Reads the current distance to the nearest object in front of the robot.
   *
   * @return Distance in cm. If no object is detected infinity is returned.
   */
  float GetDistanceFront();

  /**
   * Reads the current distance to the nearest object in front of the robot.
   *
   * @param angle The angle to rotate the range sensor to. The angle must be between -90 and 90.
   * @return Distance in cm. If no object is detected infinity is returned.
   */
  float GetDistanceFront(int angle);

  /**
   * Reads the current distance to the nearest object behind the robot.
   *
   * @return Distance in cm. If no object is detected infinity is returned.
   */
  float GetDistanceBack();

  /**
   * Rotates the range sensor (front) to a given angle.
   *
   * @param angle The angle to rotate the range sensor to. The angle must be between -90 and 90.
   */
  void RotateRangeSensor(int angle);

  /**
   * Stops any movement of the robot.
   */
  void StopMovement();

  /**
   * Moves the robot forward by the specified distance.
   * This method will block until the robot has reached the specified distance or an obstacle is detected.
   *
   * @param distance The distance to move the robot forward in m.
   */
  void MoveForward(double distance);

  /**
   * Moves the robot forward by the specified distance.
   * This method will block until the robot has reached the specified distance or an obstacle is detected.
   *
   * @param distance The distance to move the robot forward in m.
   * @param speed    The speed at which the robot moves forward in % (0-100%).
   */
  void MoveForward(double distance, int speed);

  /**
   * Moves the robot forward by the specified distance using acceleration.
   * This method will block until the robot has reached the specified distance or an obstacle is detected.
   *
   * @param distance   The distance to move the robot forward in m.
   * @param startSpeed The speed at which the robot starts moving forward in % (0-100%).
   * @param endSpeed   The desired end speed % (0-100%).
   */
  void MoveForward(double distance, int startSpeed, int endSpeed);

  /**
   * Moves the robot forward indefinitely.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   */
  void MoveForwardAsync();

  /**
   * Moves the robot forward by the specified distance.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param distance The distance to move the robot forward in m.
   */
  void MoveForwardAsync(double distance);

  /**
   * Moves the robot forward by the specified distance.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param distance The distance to move the robot forward in m.
   * @param speed    The speed to move the robot forward in % (0-100%).
   */
  void MoveForwardAsync(double distance, int speed);

  /**
   * Moves the robot forward by the specified distance using acceleration.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param distance   The distance to move the robot forward in m.
   * @param startSpeed The speed at which the robot starts moving forward in % (0-100%).
   * @param endSpeed   The desired end speed % (0-100%).
   */
  void MoveForwardAsync(double distance, int startSpeed, int endSpeed);


  /**
   * Moves the robot backward by the specified distance.
   * This method will block until the robot has reached the specified distance or an obstacle is detected.
   *
   * @param distance The distance to move the robot backward in m.
   */
  void MoveBackward(double distance);

  /**
   * Moves the robot backward by the specified distance.
   * This method will block until the robot has reached the specified distance or an obstacle is detected.
   *
   * @param distance The distance to move the robot backward in m.
   * @param speed    The speed at which the robot moves forward in % (0-100%).
   */
  void MoveBackward(double distance, int speed);

  /**
   * Moves the robot backward by the specified distance using acceleration.
   * This method will block until the robot has reached the specified distance or an obstacle is detected.
   *
   * @param distance   The distance to move the robot backward in m.
   * @param startSpeed The speed at which the robot starts moving backward in % (0-100%).
   * @param endSpeed   The desired end speed % (0-100%).
   */
  void MoveBackward(double distance, int startSpeed, int endSpeed);

  /**
   * Moves the robot backward indefinitely.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   */
  void MoveBackwardAsync();

  /**
   * Moves the robot backward by the specified distance.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param distance The distance to move the robot backward in m.
   */
  void MoveBackwardAsync(double distance);

  /**
   * Moves the robot backward by the specified distance.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param distance The distance to move the robot backward in m.
   * @param speed    The speed at which the robot moves forward in % (0-100%).
   */
  void MoveBackwardAsync(double distance, int speed);

  /**
   * Moves the robot backward by the specified distance using acceleration.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param distance   The distance to move the robot backward in m.
   * @param startSpeed The speed at which the robot starts moving backward in % (0-100%).
   * @param endSpeed   The desired end speed % (0-100%).
   */
  void MoveBackwardAsync(double distance, int startSpeed, int endSpeed);

  /**
   * Rotates the robot to the left by the specified angle.
   * This method will block until the robot has rotated by the specified angle.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void RotateLeft(double angle);

  /**
   * Rotates the robot to the left by the specified angle.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void RotateLeftAsync(double angle);

  /**
   * Rotates the robot to the right by the specified angle.
   * This method will block until the robot has rotated by the specified angle.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void RotateRight(double angle);

  /**
   * Rotates the robot to the right by the specified angle.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void RotateRightAsync(double angle);

  /**
   * Turns the robot to the left by the specified angle.
   * This method will block until the robot has rotated by the specified angle.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void TurnLeft(double angle);

  /**
   * Turns the robot to the left by the specified angle.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void TurnLeftAsync(double angle);


  /**
   * Turns the robot to the left by the specified angle and radius.
   * This method will block until the robot has rotated by the specified angle.
   *
   * @param angle  The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   * @param radius The radius of the turn in m.
   */
  void TurnLeft(double angle, double radius);

  /**
   * Turns the robot to the left by the specified angle and radius.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param angle  The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   * @param radius The radius of the turn in m.
   */
  void TurnLeftAsync(double angle, double radius);

  /**
   * Turns the robot to the right by the specified angle.
   * This method will block until the robot has rotated by the specified angle.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void TurnRight(double angle);

  /**
   * Turns the robot to the right by the specified angle.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void TurnRightAsync(double angle);

  /**
   * Turns the robot to the right by the specified angle and radius.
   * This method will block until the robot has rotated by the specified angle.
   *
   * @param angle  The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   * @param radius The radius of the turn in m.
   */
  void TurnRight(double angle, double radius);

  /**
   * Turns the robot to the right by the specified angle and radius.
   * This method will return immediately, additional calls to this method will cancel the current movement and start the requested movement.
   *
   * @param angle  The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   * @param radius The radius of the turn in m.
   */
  void TurnRightAsync(double angle, double radius);
}