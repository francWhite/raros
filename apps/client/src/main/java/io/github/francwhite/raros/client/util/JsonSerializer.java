package io.github.francwhite.raros.client.util;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;

public class JsonSerializer {

  public static String serialize(Object object) {
    try {
      return new ObjectMapper()
        .writerWithDefaultPrettyPrinter()
        .writeValueAsString(object);
    } catch (JsonProcessingException e) {
      throw new RuntimeException(e.getMessage(), e);
    }
  }
  public static <T> T deserialize(String object, Class<T> classType) {
    try {
      return new ObjectMapper()
        .setPropertyNamingStrategy(new PropertyNamingStrategies.SnakeCaseStrategy())
        .readerFor(classType)
        .readValue(object);
    } catch (JsonProcessingException e) {
      throw new RuntimeException(e.getMessage(), e);
    }
  }
}
