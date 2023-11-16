package ch.hslu.raros.client.connector;

import ch.hslu.raros.client.util.HttpRequestBuilder;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpResponse;
import java.util.concurrent.CompletableFuture;

record MagnetState(Boolean active) {
}

class MagnetApiService implements MagnetService {

  private final URI apiUri;

  public MagnetApiService(URI apiBaseUri) {
    this.apiUri = apiBaseUri.resolve("/api/magnet");
  }

  @Override
  public CompletableFuture<Void> SetMagnetState(Boolean active) {
    var magnetState = new MagnetState(active);

    var request = HttpRequestBuilder.buildJsonPOST(apiUri, magnetState);

    try (var httpClient = HttpClient.newHttpClient()) {
      return httpClient
        .sendAsync(request, HttpResponse.BodyHandlers.discarding())
        .thenApply(HttpResponse::body);
    }
  }
}

