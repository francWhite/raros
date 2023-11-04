package ch.hslu.raros.client.util;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class JsonSerializer {

  public static String serialize(Object object) {
    try {
      return new ObjectMapper()
        .writerWithDefaultPrettyPrinter()
        .writeValueAsString(object);
    } catch (JsonProcessingException e) {
      throw new RuntimeException(e);
    }
  }
}
