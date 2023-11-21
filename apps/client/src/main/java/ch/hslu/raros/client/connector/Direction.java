package ch.hslu.raros.client.connector;

import com.fasterxml.jackson.annotation.JsonValue;

public enum Direction {
  Forward(1),
  Backward(2);

  private final int value;

  private Direction(int value) {
    this.value = value;
  }

  @JsonValue
  public int getValue() {
    return value;
  }
}
