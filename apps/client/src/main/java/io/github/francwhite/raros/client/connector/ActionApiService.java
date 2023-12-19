package io.github.francwhite.raros.client.connector;

import io.github.francwhite.raros.client.util.JsonSerializer;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;

public class ActionApiService implements ActionService {
  private record Goal(UUID goalId, Boolean completed) {
  }

  private final URI apiUri;

  public ActionApiService(URI apiBaseUri) {
    this.apiUri = apiBaseUri.resolve("/api/actions/goals/");
  }

  @Override
  public CompletableFuture<Boolean> isActionCompleted(UUID goalId) {
    var uri = apiUri.resolve(goalId.toString());
    var request = HttpRequest.newBuilder(uri)
      .GET()
      .build();

    return HttpClient.newHttpClient()
      .sendAsync(request, HttpResponse.BodyHandlers.ofString())
      .thenApply(HttpResponse::body)
      .thenApply(s -> JsonSerializer.deserialize(s, Goal.class))
      .thenApply(Goal::completed);

  }
}