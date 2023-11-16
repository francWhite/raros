package ch.hslu.raros.example;

import ch.hslu.raros.client.controller.RobotControllerFactory;

import java.net.URI;
import java.util.Scanner;

public class Main {
  public static void main(String[] args) {
    var robotController = RobotControllerFactory.create(URI.create("http://eee-03300:8000"));
    var scanner = new Scanner(System.in);

    System.out.println("Press");
    System.out.println("  e to enable magnet");
    System.out.println("  d to disable magnet");
    System.out.println("  b to beep");
    System.out.println("  t to play tone");

    while (true) {
      var input = scanner.next();
      switch (input) {
        case "e":
          System.out.println("Enabling magnet...");
          robotController.EnableMagnet();
          break;
        case "d":
          System.out.println("Disabling magnet...");
          robotController.DisableMagnet();
          break;
        case "b":
          System.out.println("Beeping...");
          robotController.Beep();
          break;
        case "t":
          System.out.print("Enter frequency: ");
          var frequency = scanner.nextInt();
          System.out.print("Enter duration: ");
          var duration = scanner.nextInt();
          System.out.println("Playing tone...");
          robotController.PlayTone(frequency, duration);
          break;
        case "q":
          return;
        default:
          System.out.println("Invalid input.");
          break;
      }
    }
  }
}