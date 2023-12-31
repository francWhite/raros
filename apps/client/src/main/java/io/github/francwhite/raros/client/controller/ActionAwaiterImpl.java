package io.github.francwhite.raros.client.controller;

import io.github.francwhite.raros.client.connector.ActionInvocationResult;
import io.github.francwhite.raros.client.connector.ActionService;

import java.util.concurrent.CompletableFuture;

@SuppressWarnings("BusyWait")
class ActionAwaiterImpl implements ActionAwaiter {
  private final ActionService actionService;

  public ActionAwaiterImpl(ActionService actionService) {
    this.actionService = actionService;
  }

  @Override
  public void waitForAction(CompletableFuture<ActionInvocationResult> action) {
    var goalId = action.join().GoalId();
    while (!actionService.isActionCompleted(goalId).join()) {
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        throw new RuntimeException(e.getMessage(), e);
      }
    }
  }
}
