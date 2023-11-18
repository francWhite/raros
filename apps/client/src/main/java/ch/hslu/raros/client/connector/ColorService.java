package ch.hslu.raros.client.connector;

import java.util.concurrent.CompletableFuture;

public interface ColorService {
  /**
   * Gets the current color of the color sensor.
   * @return CompletableFuture<Color> The current color of the color sensor.
   */
  CompletableFuture<Color> GetColor();
}
