package ch.hslu.raros.client.connector;

import ch.hslu.raros.client.util.HttpRequestBuilder;
import ch.hslu.raros.client.util.JsonSerializer;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.concurrent.CompletableFuture;

class NavigationApiService implements NavigationService {
  private record MoveRequest(double distance, double speed, Direction direction) {
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
    return Move(new MoveRequest(Math.pow(10, 4), -1d, direction));
  }

  @Override
  public CompletableFuture<ActionInvocationResult> Move(double distance, Direction direction) {
    return Move(new MoveRequest(distance, -1d, direction));
  }

  @Override
  public CompletableFuture<ActionInvocationResult> Move(double distance, double speed, Direction direction) {
    return Move(new MoveRequest(distance, speed, direction));
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
}
