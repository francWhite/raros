package ch.hslu.raros.client.connector;

import ch.hslu.raros.client.util.HttpRequestBuilder;
import ch.hslu.raros.client.util.JsonSerializer;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.concurrent.CompletableFuture;

class CameraApiService implements CameraService {
  private record Rotation(int angleHorizontal, int angleVertical) {
  }

  private record ImageResponse(String imageBase64) {
  }

  private final URI apiUri;

  public CameraApiService(URI apiBaseUri) {
    this.apiUri = apiBaseUri.resolve("/api/camera/");
  }

  @Override
  public CompletableFuture<String> CaptureImage() {
    var request = HttpRequest.newBuilder(apiUri.resolve("./capture"))
      .POST(HttpRequest.BodyPublishers.noBody())
      .build();

    return HttpClient.newHttpClient()
      .sendAsync(request, HttpResponse.BodyHandlers.ofString())
      .thenApply(HttpResponse::body)
      .thenApply(s -> JsonSerializer.deserialize(s, ImageResponse.class))
      .thenApply(ImageResponse::imageBase64);
  }

  @Override
  public CompletableFuture<Void> RotateCamera(int angleHorizontal, int angleVertical) {
    if (angleHorizontal < -90 || angleHorizontal > 90)
      throw new IllegalArgumentException("The horizontal angle must be between -90 and 90.");

    if (angleVertical < -90 || angleVertical > 90)
      throw new IllegalArgumentException("The vertical angle must be between -90 and 90.");

    var request = HttpRequestBuilder.buildJsonPOST(apiUri.resolve("./rotate"), new Rotation(angleHorizontal, angleVertical));

    return HttpClient.newHttpClient()
      .sendAsync(request, HttpResponse.BodyHandlers.discarding())
      .thenApply(HttpResponse::body);
  }
}