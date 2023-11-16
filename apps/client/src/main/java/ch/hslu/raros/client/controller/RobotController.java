package ch.hslu.raros.client.controller;

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
   * Plays a tone.
   *
   * @param frequency The frequency of the tone in Hz.
   * @param duration  The duration of the tone in ms.
   */
  void PlayTone(int frequency, int duration);
}

