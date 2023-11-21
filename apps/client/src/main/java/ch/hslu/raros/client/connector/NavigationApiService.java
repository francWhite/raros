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

  private final URI apiUri;

  public NavigationApiService(URI apiBaseUri) {
    this.apiUri = apiBaseUri.resolve("/api/navigation/");
  }

  @Override
  public CompletableFuture<Void> Stop() {
    var request = HttpRequest.newBuilder(apiUri.resolve("./stop"))
      .POST(HttpRequest.BodyPublishers.noBody())
      .build();

    try (var httpClient = HttpClient.newHttpClient()) {
      return httpClient
        .sendAsync(request, HttpResponse.BodyHandlers.discarding())
        .thenApply(HttpResponse::body);
    }
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

  private CompletableFuture<ActionInvocationResult> Move(MoveRequest moveRequest) {
    var request = HttpRequestBuilder.buildJsonPOST(apiUri.resolve("./move"), moveRequest);

    try (var httpClient = HttpClient.newHttpClient()) {
      return httpClient
        .sendAsync(request, HttpResponse.BodyHandlers.ofString())
        .thenApply(HttpResponse::body)
        .thenApply(s -> JsonSerializer.deserialize(s, ActionInvocationResult.class));
    }
  }
}
