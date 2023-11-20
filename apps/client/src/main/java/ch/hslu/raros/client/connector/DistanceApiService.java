package ch.hslu.raros.client.connector;

import ch.hslu.raros.client.util.HttpRequestBuilder;
import ch.hslu.raros.client.util.JsonSerializer;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.concurrent.CompletableFuture;

class DistanceApiService implements DistanceService {
  private record Rotation(int angle) {
  }

  private final URI apiUri;

  public DistanceApiService(URI apiBaseUri) {
    this.apiUri = apiBaseUri.resolve("/api/distance/");
  }

  @Override
  public CompletableFuture<Distance> GetDistance() {
    var request = HttpRequest.newBuilder(apiUri)
      .GET()
      .build();

    try (var httpClient = HttpClient.newHttpClient()) {
      return httpClient.sendAsync(request, HttpResponse.BodyHandlers.ofString())
        .thenApply(HttpResponse::body)
        .thenApply(s -> JsonSerializer.deserialize(s, Distance.class));
    }
  }

  @Override
  public CompletableFuture<Void> RotateSensor(int angle) {
    if (angle < -90 || angle > 90)
      throw new IllegalArgumentException("The angle must be between -90 and 90.");

    var request = HttpRequestBuilder.buildJsonPOST(apiUri.resolve("./rotate-sensor"), new Rotation(angle));

    try (var httpClient = HttpClient.newHttpClient()) {
      return httpClient.sendAsync(request, HttpResponse.BodyHandlers.discarding())
        .thenApply(HttpResponse::body);
    }
  }
}