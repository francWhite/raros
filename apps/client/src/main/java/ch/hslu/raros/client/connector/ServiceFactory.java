package ch.hslu.raros.client.connector;

import java.net.URI;

public class ServiceFactory {
  public static MagnetService createMagnetService(URI apiBaseUri) {
    return new MagnetApiService(apiBaseUri);
  }

  public static BuzzerService createBuzzerService(URI apiBaseUri) {
    return new BuzzerApiService(apiBaseUri);
  }
}
