package ch.hslu.raros.client.controller;

import ch.hslu.raros.client.connector.BuzzerService;
import ch.hslu.raros.client.connector.MagnetService;

class RobotControllerImpl implements RobotController {

  private final MagnetService magnetService;
  private final BuzzerService buzzerService;

  public RobotControllerImpl(MagnetService magnetService, BuzzerService buzzerService) {
    this.magnetService = magnetService;
    this.buzzerService = buzzerService;
  }

  @Override
  public void EnableMagnet() {
    this.magnetService.SetMagnetState(true).join();
  }

  @Override
  public void DisableMagnet() {
    this.magnetService.SetMagnetState(false).join();
  }

  @Override
  public void Beep() {
    this.buzzerService.PlayTone(15000).join();
  }

  @Override
  public void PlayTone(int frequency, int duration) {
    this.buzzerService.PlayTone(frequency, duration).join();
  }
}
