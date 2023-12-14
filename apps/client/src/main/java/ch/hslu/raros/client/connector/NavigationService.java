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
   * @param direction The direction to move in (only forward and backward are possible)
   * @return CompletableFuture<ActionInvocationResult> The result of the action invocation.
   */
  CompletableFuture<ActionInvocationResult> Move(Direction direction);

  /**
   * Moves the robot forward for the given distance.
   *
   * @param distance  The distance to move in meters.
   * @param direction The direction to move in (only forward and backward are possible)
   * @return CompletableFuture<ActionInvocationResult> The result of the action invocation.
   */
  CompletableFuture<ActionInvocationResult> Move(double distance, Direction direction);

  /**
   * Moves the robot forward for the given distance.
   *
   * @param distance  The distance to move in meters.
   * @param speed     The speed to move in meters per second.
   * @param direction The direction to move in (only forward and backward are possible)
   * @return CompletableFuture<ActionInvocationResult> The result of the action invocation.
   */
  CompletableFuture<ActionInvocationResult> Move(double distance, double speed, Direction direction);

  /**
   * Rotates the robot by the given angle.
   *
   * @param angle     The angle to rotate in degrees. The angle must be between 0° and 180°.
   * @param direction The direction to rotate in (only left and right are possible)
   * @return CompletableFuture<ActionInvocationResult> The result of the action invocation.
   */
  CompletableFuture<ActionInvocationResult> Rotate(double angle, Direction direction);

  /**
   * Turns the robot by the given angle and radius.
   *
   * @param angle     The angle to turn in degrees. The angle must be between 0° and 180°.
   * @param radius    The radius to turn in meters.
   * @param direction The direction to turn in (only left and right are possible)
   * @return CompletableFuture<ActionInvocationResult> The result of the action invocation.
   */
  CompletableFuture<ActionInvocationResult> Turn(double angle, double radius, Direction direction);
}

