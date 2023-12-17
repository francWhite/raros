package io.github.francwhite.raros.client.connector;

import java.util.concurrent.CompletableFuture;

public interface ColorService {
  /**
   * Gets the current color of the color sensor.
   * @return {@code CompletableFuture<Color>} The current color of the color sensor.
   */
  CompletableFuture<Color> GetColor();
}

