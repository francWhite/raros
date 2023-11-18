package ch.hslu.raros.client.connector;

import java.util.concurrent.CompletableFuture;

public interface DistanceService {
  /**
   * Gets the current distance to the nearest object in front and back of the robot.
   *
   * @return CompletableFuture<Distance> The current distance to the nearest object in front and back of the robot.
   */
  CompletableFuture<Distance> GetDistance();

  /**
   * Rotates the range sensor (front) to a given angle.
   *
   * @param angle The angle to rotate the range sensor to. The angle must be between -90 and 90.
   * @return CompletableFuture<Void>
   */
  CompletableFuture<Void> RotateSensor(int angle);
}
