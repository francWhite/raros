package ch.hslu.raros.client.controller;

import ch.hslu.raros.client.connector.Color;

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
}

