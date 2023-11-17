package ch.hslu.raros.client.controller;

import ch.hslu.raros.client.connector.ServiceFactory;

import java.net.URI;

public class RobotControllerFactory {
  public static RobotController create(URI apiBaseUri) {
    return new RobotControllerImpl(
      new ActionAwaiterImpl(ServiceFactory.createActionService(apiBaseUri)),
      ServiceFactory.createMagnetService(apiBaseUri),
      ServiceFactory.createBuzzerService(apiBaseUri)
    );
  }
}
