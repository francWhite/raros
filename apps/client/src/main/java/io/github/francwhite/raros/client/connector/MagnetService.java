package io.github.francwhite.raros.client.connector;

import java.util.concurrent.CompletableFuture;

public interface MagnetService {
  CompletableFuture<Void> setMagnetState(Boolean active);
}
