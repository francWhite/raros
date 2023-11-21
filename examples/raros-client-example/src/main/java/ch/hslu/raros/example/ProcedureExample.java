package ch.hslu.raros.example;

import ch.hslu.raros.client.connector.Color;
import ch.hslu.raros.client.controller.RobotControllerFactory;

import java.net.URI;

public class ProcedureExample {
  public static void main(String[] args) {
    var controller = RobotControllerFactory.create(URI.create("http://eee-03300:8000"));
    System.out.println("Starting procedure...");

    controller.MoveForwardAsync();
    Color color = controller.GetColor();
    while (color.red() < 60) {
      if (controller.GetDistanceFront() < 30) {
        System.out.println("possible collision recognized, aborting");
        controller.StopMovement();
        controller.PlayToneAsync(1200, 1000);
        System.exit(0);
      }
      color = controller.GetColor();
      System.out.println("Waiting for green color to appear... current color: " + color);
    }

    System.out.println("Red color detected! Stop movement.");
    controller.Beep();
    controller.StopMovement();

    System.out.println("Enable magnet...");
    controller.EnableMagnet();

    System.out.println("Move 0.5m backward...");
    controller.MoveBackward(0.5);

    System.out.println("Disable magnet...");
    controller.DisableMagnet();

    System.out.println("Move 0.1m backward...");
    controller.MoveBackward(0.1);

    System.out.println("Finished!");
    controller.PlayTone(440, 200);
    controller.PlayTone(880, 200);
    controller.PlayTone(440, 200);
    System.exit(0);
  }
}
