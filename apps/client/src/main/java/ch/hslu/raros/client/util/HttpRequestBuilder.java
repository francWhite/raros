package ch.hslu.raros.client.util;

import java.net.URI;
import java.net.http.HttpRequest;

public class HttpRequestBuilder {
  public static HttpRequest buildJsonPOST(URI uri, Object body) {
    return HttpRequest.newBuilder(uri)
      .POST(HttpRequest.BodyPublishers.ofString(JsonSerializer.serialize(body)))
      .header("Content-Type", "application/json")
      .build();
  }
}
