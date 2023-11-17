package ch.hslu.raros.client.controller;

import ch.hslu.raros.client.connector.ActionInvocationResult;
import ch.hslu.raros.client.connector.ActionService;

import java.util.concurrent.CompletableFuture;

@SuppressWarnings("BusyWait")
public class ActionAwaiterImpl implements ActionAwaiter {
  private final ActionService actionService;

  public ActionAwaiterImpl(ActionService actionService) {
    this.actionService = actionService;
  }

  @Override
  public void WaitForAction(CompletableFuture<ActionInvocationResult> action) {
    var goalId = action.join().GoalId();
    while (!actionService.IsActionCompleted(goalId).join()) {
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        throw new RuntimeException(e);
      }
    }
  }
}
