package ch.hslu.raros.client.connector;

import java.util.concurrent.CompletableFuture;

public interface MagnetService {
  CompletableFuture<Void> SetMagnetState(Boolean active);
}