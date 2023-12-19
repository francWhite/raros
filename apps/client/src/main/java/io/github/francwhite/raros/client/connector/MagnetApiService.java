package io.github.francwhite.raros.client.connector;

import io.github.francwhite.raros.client.util.HttpRequestBuilder;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpResponse;
import java.util.concurrent.CompletableFuture;

class MagnetApiService implements MagnetService {
  private record MagnetState(Boolean active) {
  }

  private final URI apiUri;

  public MagnetApiService(URI apiBaseUri) {
    this.apiUri = apiBaseUri.resolve("/api/magnet");
  }

  @Override
  public CompletableFuture<Void> setMagnetState(Boolean active) {
    var magnetState = new MagnetState(active);

    var request = HttpRequestBuilder.buildJsonPOST(apiUri, magnetState);

    return HttpClient.newHttpClient()
      .sendAsync(request, HttpResponse.BodyHandlers.discarding())
      .thenApply(HttpResponse::body);

  }
}
