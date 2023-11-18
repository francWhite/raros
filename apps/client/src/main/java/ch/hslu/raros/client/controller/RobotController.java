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
   * @return Color The last color read from the color sensor.
   */
  Color GetColor();
}

