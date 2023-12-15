package ch.hslu.raros.client.connector;

import java.util.concurrent.CompletableFuture;

public interface StatusService {
  /**
   * Gets the current status of the robot.
   *
   * @return CompletableFuture<Status> The current status of the robot.
   */
  CompletableFuture<Status> GetStatus();
}

