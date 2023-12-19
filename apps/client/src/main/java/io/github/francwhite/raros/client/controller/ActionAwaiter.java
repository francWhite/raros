package io.github.francwhite.raros.client.controller;

import io.github.francwhite.raros.client.connector.ActionInvocationResult;

import java.util.concurrent.CompletableFuture;

interface ActionAwaiter {
  /**
   * Waits for the given action to be completed.
   *
   * @param action The action to wait for.
   */
  void waitForAction(CompletableFuture<ActionInvocationResult> action);
}
