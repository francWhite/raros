package ch.hslu.raros.client.connector;

import java.util.concurrent.CompletableFuture;

public interface BuzzerService {
  /**
   * Play a tone with the given frequency and duration
   *
   * @param frequency in Hz
   * @param duration  in ms
   * @return CompletableFuture<Void>
   */
  CompletableFuture<ActionInvocationResult> PlayTone(int frequency, int duration);
}
