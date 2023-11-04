package ch.hslu.raros.client.controller;

import ch.hslu.raros.client.connector.ServiceFactory;

import java.net.URI;

public class RobotControllerFactory {
  public static RobotController create(URI apiBaseUri) {
    var magnetService = ServiceFactory.createMagnetService(apiBaseUri);
    return new RobotControllerImpl(magnetService);
  }
}
