package ch.hslu.raros.client.controller;

import ch.hslu.raros.client.connector.ActionInvocationResult;

import java.util.concurrent.CompletableFuture;

interface ActionAwaiter {
  /**
   * Waits for the given action to be completed.
   *
   * @param action The action to wait for.
   */
  void WaitForAction(CompletableFuture<ActionInvocationResult> action);
}
