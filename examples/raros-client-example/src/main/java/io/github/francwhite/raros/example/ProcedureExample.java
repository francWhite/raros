package io.github.francwhite.raros.example;

import io.github.francwhite.raros.client.connector.Color;
import io.github.francwhite.raros.client.controller.RobotControllerFactory;

import java.net.URI;

public class ProcedureExample {
  public static void main(String[] args) {
    var controller = RobotControllerFactory.create(URI.create("http://eee-03300:8000"));
    System.out.println("Starting procedure...");

    controller.moveForwardAsync();
    Color color = controller.getColor();
    while (color.red() < 70) {
      if (controller.getDistanceFront() < 30) {
        System.out.println("possible collision recognized, aborting");
        controller.stopMovement();
        controller.playToneAsync(1200, 1000);
        System.exit(0);
      }
      color = controller.getColor();
      System.out.println("Waiting for red color to appear... current color: " + color);
    }

    System.out.println("Target detected! Stop movement.");
    controller.beep();
    controller.stopMovement();

    System.out.println("Picking up payload...");
    controller.enableMagnet();

    System.out.println("Move to drop zone...");
    controller.moveBackward(0.5);
    controller.rotateLeft(90);
    controller.moveForward(0.1);

    System.out.println("Releasing payload");
    controller.beep();
    controller.disableMagnet();

    System.out.println("Leave drop zone...");
    controller.moveBackward(0.1);
    controller.rotateRight(90);
    controller.moveBackward(0.5);

    System.out.println("Finished!");
    controller.playToneAsync(440, 500);
    controller.playToneAsync(880, 500);
    controller.playToneAsync(440, 500);
    System.exit(0);
  }
}
