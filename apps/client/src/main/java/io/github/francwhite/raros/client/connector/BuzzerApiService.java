package io.github.francwhite.raros.client.connector;

import io.github.francwhite.raros.client.util.HttpRequestBuilder;
import io.github.francwhite.raros.client.util.JsonSerializer;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpResponse;
import java.util.concurrent.CompletableFuture;


class BuzzerApiService implements BuzzerService {
  private record Tone(int frequency, int duration) {
  }

  private final URI apiUri;

  public BuzzerApiService(URI apiBaseUri) {
    this.apiUri = apiBaseUri.resolve("/api/buzzer/");
  }

  @Override
  public CompletableFuture<ActionInvocationResult> PlayTone(int frequency, int duration) {
    var tone = new Tone(frequency, duration);
    var request = HttpRequestBuilder.buildJsonPOST(apiUri.resolve("./tone"), tone);

    return HttpClient.newHttpClient()
      .sendAsync(request, HttpResponse.BodyHandlers.ofString())
      .thenApply(HttpResponse::body)
      .thenApply(s -> JsonSerializer.deserialize(s, ActionInvocationResult.class));

  }
}
