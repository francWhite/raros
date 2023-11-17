package ch.hslu.raros.client.connector;

import java.util.UUID;
import java.util.concurrent.CompletableFuture;

public interface ActionService {

  /**
   * Checks if the given action is completed
   * @param goalId The goalId to check
   * @return CompletableFuture<Boolean> True if the action is completed, false otherwise
   */
  CompletableFuture<Boolean> IsActionCompleted(UUID goalId);
}
