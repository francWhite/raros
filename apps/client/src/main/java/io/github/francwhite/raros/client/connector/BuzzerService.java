package io.github.francwhite.raros.client.connector;

import java.util.concurrent.CompletableFuture;

public interface BuzzerService {
  /**
   * Play a tone with the given frequency and duration
   *
   * @param frequency in Hz
   * @param duration  in ms
   * @return {@code CompletableFuture<Void>}
   */
  CompletableFuture<ActionInvocationResult> playTone(int frequency, int duration);
}
