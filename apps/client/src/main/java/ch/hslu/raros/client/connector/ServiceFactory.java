package ch.hslu.raros.client.connector;

import java.net.URI;

public class ServiceFactory {

  public static ActionService createActionService(URI apiBaseUri) {
    return new ActionApiService(apiBaseUri);
  }

  public static MagnetService createMagnetService(URI apiBaseUri) {
    return new MagnetApiService(apiBaseUri);
  }

  public static BuzzerService createBuzzerService(URI apiBaseUri) {
    return new BuzzerApiService(apiBaseUri);
  }

  public static ColorService createColorService(URI apiBaseUri) {
    return new ColorApiService(apiBaseUri);
  }

  public static DistanceService createDistanceService(URI apiBaseUri) {
    return new DistanceApiService(apiBaseUri);
  }

  public static NavigationService createNavigationService(URI apiBaseUri) {
    return new NavigationApiService(apiBaseUri);
  }
}
