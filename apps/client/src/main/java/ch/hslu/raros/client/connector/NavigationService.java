package ch.hslu.raros.client.connector;

import java.util.concurrent.CompletableFuture;

public interface NavigationService {

  /**
   * Stops the robot.
   *
   * @return CompletableFuture<Void>
   */
  CompletableFuture<Void> Stop();

  /**
   * Moves the robot forward indefinitely.
   *
   * @param direction The direction to move in.
   * @return CompletableFuture<ActionInvocationResult> The result of the action invocation.
   */
  CompletableFuture<ActionInvocationResult> Move(Direction direction);

  /**
   * Moves the robot forward for the given distance.
   *
   * @param distance  The distance to move in meters.
   * @param direction The direction to move in.
   * @return CompletableFuture<ActionInvocationResult> The result of the action invocation.
   */
  CompletableFuture<ActionInvocationResult> Move(double distance, Direction direction);

  /**
   * Moves the robot forward for the given distance.
   *
   * @param distance  The distance to move in meters.
   * @param speed     The speed to move in meters per second.
   * @param direction The direction to move in.
   * @return CompletableFuture<ActionInvocationResult> The result of the action invocation.
   */
  CompletableFuture<ActionInvocationResult> Move(double distance, double speed, Direction direction);
}

