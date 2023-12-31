package io.github.francwhite.raros.client.controller;

import io.github.francwhite.raros.client.connector.ServiceFactory;

import java.net.URI;

public class RobotControllerFactory {
  /**
   * Creates a new instance of {@code RobotController}.
   *
   * @param apiBaseUri The base URI of the Robot REST-API.
   * @return {@code RobotController}
   */
  public static RobotController create(URI apiBaseUri) {
    return new RobotControllerImpl(
      new ActionAwaiterImpl(ServiceFactory.createActionService(apiBaseUri)),
      ServiceFactory.createMagnetService(apiBaseUri),
      ServiceFactory.createBuzzerService(apiBaseUri),
      ServiceFactory.createColorService(apiBaseUri),
      ServiceFactory.createDistanceService(apiBaseUri),
      ServiceFactory.createNavigationService(apiBaseUri),
      ServiceFactory.createStatusService(apiBaseUri),
      ServiceFactory.createCameraService(apiBaseUri)
    );
  }
}
