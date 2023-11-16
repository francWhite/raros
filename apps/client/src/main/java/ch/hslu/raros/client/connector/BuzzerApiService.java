package ch.hslu.raros.client.connector;

import ch.hslu.raros.client.util.HttpRequestBuilder;

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
  public CompletableFuture<Void> PlayTone(int frequency) {
    return PlayTone(new Tone(frequency, 100));
  }

  @Override
  public CompletableFuture<Void> PlayTone(int frequency, int duration) {
    return PlayTone(new Tone(frequency, duration));
  }

  private CompletableFuture<Void> PlayTone(Tone tone) {
    var request = HttpRequestBuilder.buildJsonPOST(apiUri.resolve("./tone"), tone);

    try (var httpClient = HttpClient.newHttpClient()) {
      return httpClient
        .sendAsync(request, HttpResponse.BodyHandlers.discarding())
        .thenApply(HttpResponse::body);
    }
  }
}
