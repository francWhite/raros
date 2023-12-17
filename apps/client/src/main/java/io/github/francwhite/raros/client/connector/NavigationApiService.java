package io.github.francwhite.raros.client.connector;

import io.github.francwhite.raros.client.util.HttpRequestBuilder;
import io.github.francwhite.raros.client.util.JsonSerializer;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.concurrent.CompletableFuture;

class NavigationApiService implements NavigationService {
  private static final int MAX_SPEED = 60;
  private static final int DEFAULT_SPEED = 30;

  private record MoveRequest(double distance, int startSpeed, int endSpeed, Direction direction) {
  }

  private record RotateRequest(double angle, Direction direction) {
  }

  private record TurnRequest(double angle, double radius, Direction direction) {
  }

  private final URI apiUri;

  public NavigationApiService(URI apiBaseUri) {
    this.apiUri = apiBaseUri.resolve("/api/navigation/");
  }

  @Override
  public CompletableFuture<Void> Stop() {
    var request = HttpRequest.newBuilder(apiUri.resolve("./stop"))
      .POST(HttpRequest.BodyPublishers.noBody())
      .build();

    return HttpClient.newHttpClient()
      .sendAsync(request, HttpResponse.BodyHandlers.discarding())
      .thenApply(HttpResponse::body);

  }

  @Override
  public CompletableFuture<ActionInvocationResult> Move(Direction direction) {
    return Move(new MoveRequest(Math.pow(10, 4), DEFAULT_SPEED, DEFAULT_SPEED, direction));
  }

  @Override
  public CompletableFuture<ActionInvocationResult> Move(double distance, Direction direction) {
    return Move(new MoveRequest(distance, DEFAULT_SPEED, DEFAULT_SPEED, direction));
  }

  @Override
  public CompletableFuture<ActionInvocationResult> Move(double distance, int speed, Direction direction) {
    return Move(new MoveRequest(distance, ConvertSpeedPercentageToRPM(speed), ConvertSpeedPercentageToRPM(speed), direction));
  }

  @Override
  public CompletableFuture<ActionInvocationResult> Move(double distance, int startSpeed, int endSpeed, Direction direction) {
    return Move(new MoveRequest(distance, ConvertSpeedPercentageToRPM(startSpeed), ConvertSpeedPercentageToRPM(endSpeed), direction));
  }

  @Override
  public CompletableFuture<ActionInvocationResult> Rotate(double angle, Direction direction) {
    if (angle < 0 || angle > 180)
      throw new IllegalArgumentException("The angle must be between 0° and 180°.");

    if (direction != Direction.Left && direction != Direction.Right)
      throw new IllegalArgumentException("Only left and right are supported as directions.");

    var request = HttpRequestBuilder.buildJsonPOST(apiUri.resolve("./rotate"), new RotateRequest(angle, direction));

    return HttpClient.newHttpClient()
      .sendAsync(request, HttpResponse.BodyHandlers.ofString())
      .thenApply(HttpResponse::body)
      .thenApply(s -> JsonSerializer.deserialize(s, ActionInvocationResult.class));

  }

  @Override
  public CompletableFuture<ActionInvocationResult> Turn(double angle, double radius, Direction direction) {
    if (angle < 0 || angle > 180)
      throw new IllegalArgumentException("The angle must be between 0° and 180°.");

    if (direction != Direction.Left && direction != Direction.Right)
      throw new IllegalArgumentException("Only left and right are supported as directions.");

    var request = HttpRequestBuilder.buildJsonPOST(apiUri.resolve("./turn"), new TurnRequest(angle, radius, direction));

    return HttpClient.newHttpClient()
      .sendAsync(request, HttpResponse.BodyHandlers.ofString())
      .thenApply(HttpResponse::body)
      .thenApply(s -> JsonSerializer.deserialize(s, ActionInvocationResult.class));

  }

  private CompletableFuture<ActionInvocationResult> Move(MoveRequest moveRequest) {
    if (moveRequest.direction != Direction.Forward && moveRequest.direction != Direction.Backward)
      throw new IllegalArgumentException("Only forward and backward are supported as directions.");

    var request = HttpRequestBuilder.buildJsonPOST(apiUri.resolve("./move"), moveRequest);

    return HttpClient.newHttpClient()
      .sendAsync(request, HttpResponse.BodyHandlers.ofString())
      .thenApply(HttpResponse::body)
      .thenApply(s -> JsonSerializer.deserialize(s, ActionInvocationResult.class));
  }

  private int ConvertSpeedPercentageToRPM(int speed) {
    if (speed < 0 || speed > 100)
      throw new IllegalArgumentException("The speed must be between 0% and 100%.");

    return (int) Math.round(speed / 100.0 * MAX_SPEED);
  }
}
