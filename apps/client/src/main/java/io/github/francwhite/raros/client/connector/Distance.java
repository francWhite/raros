package io.github.francwhite.raros.client.connector;

import java.util.Optional;

public class Distance {
  Float front;
  Float back;

  public float getFront() {
    return Optional.ofNullable(front).orElse(Float.POSITIVE_INFINITY);
  }

  public float getBack() {
    return Optional.ofNullable(back).orElse(Float.POSITIVE_INFINITY);
  }
}