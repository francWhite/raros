package ch.hslu.raros.client.controller;

import ch.hslu.raros.client.connector.Color;

//TODO (entire project): rename methods to match the camelCase naming convention
public interface RobotController {

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
   * @param speed    The speed at which the robot moves forward in m/s.
   */
  void MoveForward(double distance, double speed);

  /**
   * Moves the robot forward indefinitely.
   * This method will return immediately, additional calls to this method will cancel the current movement and queue the requested movement.
   */
  void MoveForwardAsync();

  /**
   * Moves the robot forward by the specified distance.
   * This method will return immediately, additional calls to this method will cancel the current movement and queue the requested movement.
   *
   * @param distance The distance to move the robot forward in m.
   */
  void MoveForwardAsync(double distance);

  /**
   * Moves the robot forward by the specified distance.
   * This method will return immediately, additional calls to this method will cancel the current movement and queue the requested movement.
   *
   * @param distance The distance to move the robot forward in m.
   * @param speed    The speed to move the robot forward in m/s.
   */
  void MoveForwardAsync(double distance, double speed);


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
   * @param speed    The speed at which the robot moves forward in m/s.
   */
  void MoveBackward(double distance, double speed);

  /**
   * Moves the robot backward indefinitely.
   * This method will return immediately, additional calls to this method will cancel the current movement and queue the requested movement.
   */
  void MoveBackwardAsync();

  /**
   * Moves the robot backward by the specified distance.
   * This method will return immediately, additional calls to this method will cancel the current movement and queue the requested movement.
   *
   * @param distance The distance to move the robot backward in m.
   */
  void MoveBackwardAsync(double distance);

  /**
   * Moves the robot backward by the specified distance.
   * This method will return immediately, additional calls to this method will cancel the current movement and queue the requested movement.
   *
   * @param distance The distance to move the robot backward in m.
   * @param speed    The speed at which the robot moves forward in m/s.
   */
  void MoveBackwardAsync(double distance, double speed);

  /**
   * Rotates the robot to the left by the specified angle.
   * This method will block until the robot has rotated by the specified angle.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void RotateLeft(double angle);

  /**
   * Rotates the robot to the left by the specified angle.
   * This method will return immediately, additional calls to this method will cancel the current movement and queue the requested movement.
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
   * This method will return immediately, additional calls to this method will cancel the current movement and queue the requested movement.
   *
   * @param angle The angle to rotate the robot by in degrees. The angle must be between 0° and 180°.
   */
  void RotateRightAsync(double angle);
}

