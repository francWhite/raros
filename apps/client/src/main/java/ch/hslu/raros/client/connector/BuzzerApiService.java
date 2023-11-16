package ch.hslu.raros.client.connector;

import ch.hslu.raros.client.util.JsonSerializer;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
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
    HttpRequest request = HttpRequest.newBuilder(apiUri.resolve("./tone"))
      .POST(HttpRequest.BodyPublishers.ofString(JsonSerializer.serialize(tone)))
      .header("Content-Type", "application/json")
      .build();

    try (HttpClient httpClient = HttpClient.newHttpClient()) {
      return httpClient
        .sendAsync(request, HttpResponse.BodyHandlers.discarding())
        .thenApply(HttpResponse::body);
    }
  }
}
