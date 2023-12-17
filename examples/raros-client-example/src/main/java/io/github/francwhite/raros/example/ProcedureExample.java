package io.github.francwhite.raros.example;

import io.github.francwhite.raros.client.connector.Color;
import io.github.francwhite.raros.client.controller.RobotControllerFactory;

import java.net.URI;

public class ProcedureExample {
  public static void main(String[] args) {
    var controller = RobotControllerFactory.create(URI.create("http://eee-03300:8000"));
    System.out.println("Starting procedure...");

    controller.MoveForwardAsync();
    Color color = controller.GetColor();
    while (color.red() < 70) {
      if (controller.GetDistanceFront() < 30) {
        System.out.println("possible collision recognized, aborting");
        controller.StopMovement();
        controller.PlayToneAsync(1200, 1000);
        System.exit(0);
      }
      color = controller.GetColor();
      System.out.println("Waiting for red color to appear... current color: " + color);
    }

    System.out.println("Target detected! Stop movement.");
    controller.Beep();
    controller.StopMovement();

    System.out.println("Picking up payload...");
    controller.EnableMagnet();

    System.out.println("Move to drop zone...");
    controller.MoveBackward(0.5);
    controller.RotateLeft(90);
    controller.MoveForward(0.1);

    System.out.println("Releasing payload");
    controller.Beep();
    controller.DisableMagnet();

    System.out.println("Leave drop zone...");
    controller.MoveBackward(0.1);
    controller.RotateRight(90);
    controller.MoveBackward(0.5);

    System.out.println("Finished!");
    controller.PlayToneAsync(440, 500);
    controller.PlayToneAsync(880, 500);
    controller.PlayToneAsync(440, 500);
    System.exit(0);
  }
}
