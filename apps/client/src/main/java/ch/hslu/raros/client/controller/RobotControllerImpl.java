package ch.hslu.raros.client.controller;

import ch.hslu.raros.client.connector.BuzzerService;
import ch.hslu.raros.client.connector.Color;
import ch.hslu.raros.client.connector.ColorService;
import ch.hslu.raros.client.connector.MagnetService;

class RobotControllerImpl implements RobotController {

  private final ActionAwaiter actionAwaiter;
  private final MagnetService magnetService;
  private final BuzzerService buzzerService;
  private final ColorService colorService;

  public RobotControllerImpl(ActionAwaiter actionAwaiter,
                             MagnetService magnetService,
                             BuzzerService buzzerService,
                             ColorService colorService) {
    this.actionAwaiter = actionAwaiter;
    this.magnetService = magnetService;
    this.buzzerService = buzzerService;
    this.colorService = colorService;
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
    this.buzzerService.PlayTone(15000, 100).join();
  }

  @Override
  public void PlayTone(int frequency, int duration) {
    var action = this.buzzerService.PlayTone(frequency, duration);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void PlayToneAsync(int frequency, int duration) {
    this.buzzerService.PlayTone(frequency, duration);
  }

  @Override
  public Color GetColor() {
    return this.colorService.GetColor().join();
  }
}
