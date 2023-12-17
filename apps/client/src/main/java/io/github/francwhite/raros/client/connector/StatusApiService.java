package io.github.francwhite.raros.client.connector;

import io.github.francwhite.raros.client.util.JsonSerializer;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.concurrent.CompletableFuture;

public class StatusApiService implements StatusService {
  private final URI apiUri;

  public StatusApiService(URI apiBaseUri) {
    this.apiUri = apiBaseUri.resolve("/api/status/");
  }

  @Override
  public CompletableFuture<Status> GetStatus() {
    var request = HttpRequest.newBuilder(apiUri)
      .GET()
      .build();

    return HttpClient.newHttpClient()
      .sendAsync(request, HttpResponse.BodyHandlers.ofString())
      .thenApply(HttpResponse::body)
      .thenApply(s -> JsonSerializer.deserialize(s, Status.class));
  }
}
