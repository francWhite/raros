package ch.hslu.raros.client.controller;

import ch.hslu.raros.client.connector.MagnetService;

class RobotControllerImpl implements RobotController {

  private final MagnetService magnetService;

  public RobotControllerImpl(MagnetService magnetService) {
    this.magnetService = magnetService;
  }

  @Override
  public void EnableMagnet() {
    this.magnetService.SetMagnetState(true).join();
  }

  @Override
  public void DisableMagnet() {
    this.magnetService.SetMagnetState(false).join();
  }
}
