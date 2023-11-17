package ch.hslu.raros.client.connector;

import ch.hslu.raros.client.util.HttpRequestBuilder;
import ch.hslu.raros.client.util.JsonSerializer;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpResponse;
import java.util.concurrent.CompletableFuture;

record Tone(int frequency, int duration) {
}

class BuzzerApiService implements BuzzerService {

  private final URI apiUri;

  public BuzzerApiService(URI apiBaseUri) {
    this.apiUri = apiBaseUri.resolve("/api/buzzer/");
  }

  @Override
  public CompletableFuture<ActionInvocationResult> PlayTone(int frequency, int duration) {
    var tone = new Tone(frequency, duration);
    var request = HttpRequestBuilder.buildJsonPOST(apiUri.resolve("./tone"), tone);

    try (var httpClient = HttpClient.newHttpClient()) {
      return httpClient
        .sendAsync(request, HttpResponse.BodyHandlers.ofString())
        .thenApply(HttpResponse::body)
        .thenApply(s -> JsonSerializer.deserialize(s, ActionInvocationResult.class));
    }
  }
}
