package ch.hslu.raros.client.connector;

import ch.hslu.raros.client.util.JsonSerializer;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.concurrent.CompletableFuture;

class ColorApiService implements ColorService {
  private final URI apiUri;

  public ColorApiService(URI apiBaseUri) {
    this.apiUri = apiBaseUri.resolve("/api/color/");
  }

  @Override
  public CompletableFuture<Color> GetColor() {
    var request = HttpRequest.newBuilder(apiUri)
      .GET()
      .build();

    try (var httpClient = HttpClient.newHttpClient()) {
      return httpClient.sendAsync(request, HttpResponse.BodyHandlers.ofString())
        .thenApply(HttpResponse::body)
        .thenApply(s -> JsonSerializer.deserialize(s, Color.class));
    }
  }
}
