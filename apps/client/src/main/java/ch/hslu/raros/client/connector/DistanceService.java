package ch.hslu.raros.client.connector;

import java.util.concurrent.CompletableFuture;

public interface DistanceService {
  /**
   * Gets the current distance of the distance sensor.
   *
   * @return CompletableFuture<Distance> The current distance of the distance sensor.
   */
  CompletableFuture<Distance> GetDistance();
}
