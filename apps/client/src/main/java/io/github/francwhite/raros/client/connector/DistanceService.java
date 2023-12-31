package io.github.francwhite.raros.client.connector;

import java.util.concurrent.CompletableFuture;

public interface DistanceService {
  /**
   * Gets the current distance to the nearest object in front and back of the robot.
   *
   * @return {@code CompletableFuture<Distance>} The current distance to the nearest object in front and back of the robot.
   */
  CompletableFuture<Distance> getDistance();

  /**
   * Rotates the range sensor (front) to a given angle.
   *
   * @param angle The angle to rotate the range sensor to. The angle must be between -90 and 90.
   * @return {@code CompletableFuture<Void>}
   */
  CompletableFuture<Void> rotateSensor(int angle);
}
