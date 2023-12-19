package io.github.francwhite.raros.client.connector;

import java.util.concurrent.CompletableFuture;

public interface StatusService {
  /**
   * Gets the current status of the robot.
   *
   * @return {@code CompletableFuture<Status>} The current status of the robot.
   */
  CompletableFuture<Status> getStatus();
}

