package ch.hslu.raros.client.connector;

import ch.hslu.raros.client.util.JsonSerializer;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.concurrent.CompletableFuture;

class MagnetApiService implements MagnetService {

  private final URI apiUri;

  public MagnetApiService(URI apiBaseUri) {
    this.apiUri = apiBaseUri.resolve("/api/magnet");
  }

  @Override
  public CompletableFuture<Void> SetMagnetState(Boolean active) {
    var magnetState = new MagnetState();
    magnetState.active = active;

    HttpRequest request = HttpRequest.newBuilder(apiUri)
      .POST(HttpRequest.BodyPublishers.ofString(JsonSerializer.serialize(magnetState)))
      .header("Content-Type", "application/json")
      .build();

    try (HttpClient httpClient = HttpClient.newHttpClient()) {
      return httpClient
        .sendAsync(request, HttpResponse.BodyHandlers.discarding())
        .thenApply(HttpResponse::body);
    }
  }
}

class MagnetState {
  public Boolean active;
}