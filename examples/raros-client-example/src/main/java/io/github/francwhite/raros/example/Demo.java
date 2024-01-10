package io.github.francwhite.raros.example;

import io.github.francwhite.raros.client.connector.Color;
import io.github.francwhite.raros.client.controller.RobotController;
import io.github.francwhite.raros.client.controller.RobotControllerFactory;

import java.net.URI;

public class Demo {

  public static void main(String[] args) {
    var controller = RobotControllerFactory.create(URI.create("http://eee-03300:8000"));
    System.out.println("Starting procedure...");


    controller.moveForwardAsync(5, 40);

    while (true) {
      var color = controller.getColor();
      if (isRed(color)) {
        System.out.println("Red chip detected");
        storeChip(controller, 0.3);
      }

      if (isGreen(color)) {
        System.out.println("Green chip detected");
        storeChip(controller, 0.6);
      }

      if (isBlue(color)) {
        System.out.println("Blue chip detected");
        storeChip(controller, 0.9);
      }

      var distance = getDistanceFront(controller);
      if (distance < 25) {
        System.out.println("no more chips detected, finishing procedure");
        controller.stopMovement();
        controller.playTone(880, 500);
        controller.playTone(440, 500);
        controller.playTone(880, 500);
        var image = controller.captureImage(0, 45);
        System.out.println("final image captured: " + image);
        System.exit(0);
      }
    }
  }

  private static void storeChip(RobotController controller, double dropZoneDistance) {
    pickupChip(controller);
    dropChip(controller, dropZoneDistance);

    System.out.println("Searching next chip...");
    controller.moveForwardAsync(5, 40);
  }

  private static void pickupChip(RobotController controller) {
    System.out.println("Picking up chip...");
    controller.stopMovement();
    controller.enableMagnet();
    controller.moveBackwardAsync(3);

    // get back to the start position
    while (true) {
      var distance = getDistanceBack(controller);
      if (distance < 30) {
        controller.stopMovement();
        break;
      }
    }
  }

  private static void dropChip(RobotController controller, double dropZoneDistance) {
    System.out.println("Move to red drop zone...");
    controller.moveForward(dropZoneDistance);
    controller.rotateLeft(90);
    controller.moveForward(0.5);

    System.out.println("Releasing payload");
    controller.beep();
    controller.disableMagnet();

    System.out.println("Leave drop zone...");
    controller.moveBackward(0.5);
    controller.rotateRight(90);
  }

  private static boolean isRed(Color color) {
    return color.red() > 60;
  }

  private static boolean isGreen(Color color) {
    return color.green() == getMax(color);
  }

  private static boolean isBlue(Color color) {
    return color.blue() == getMax(color);
  }

  private static int getMax(Color color) {
    return Math.max(color.red(), Math.max(color.green(), color.blue()));
  }

  private static double getDistanceFront(RobotController controller) {
    //error tolerant distance measurement -> sometimes the sensor returns an abnormal low value
    var distanceA = controller.getDistanceFront();
    var distanceB = controller.getDistanceFront();
    return Math.max(distanceA, distanceB);
  }

  private static double getDistanceBack(RobotController controller) {
    //error tolerant distance measurement -> sometimes the sensor returns an abnormal low value
    var distanceA = controller.getDistanceBack();
    var distanceB = controller.getDistanceBack();
    return Math.max(distanceA, distanceB);
  }
}
