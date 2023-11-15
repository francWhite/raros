package ch.hslu.raros.example;

import ch.hslu.raros.client.controller.RobotControllerFactory;

import java.net.URI;
import java.util.Scanner;

public class Main {
  public static void main(String[] args) {
    var robotController = RobotControllerFactory.create(URI.create("http://eee-03300:8000"));

    System.out.println("Press 'e' to enable the magnet, 'd' to disable it, or 'q' to quit.");
    while (true) {
      var input = new Scanner(System.in).nextLine();
      switch (input) {
        case "e":
          robotController.EnableMagnet();
          break;
        case "d":
          robotController.DisableMagnet();
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