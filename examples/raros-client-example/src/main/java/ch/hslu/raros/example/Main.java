package ch.hslu.raros.example;

import ch.hslu.raros.client.controller.RobotController;
import ch.hslu.raros.client.controller.RobotControllerFactory;

import java.net.URI;
import java.text.MessageFormat;
import java.util.Scanner;

public class Main {
  public static void main(String[] args) {
    var robotController = RobotControllerFactory.create(URI.create("http://eee-03300:8000"));
    var scanner = new Scanner(System.in);

    System.out.println("Press");
    System.out.println("  m to control magnet");
    System.out.println("  b to control buzzer");
    System.out.println("  c to get color");
    System.out.println("  d to get distance");

    var input = scanner.next();
    switch (input) {
      case "m":
        magnetExample(robotController, scanner);
        break;
      case "b":
        buzzerExample(robotController, scanner);
        break;
      case "c":
        colorExample(robotController, scanner);
        break;
      case "d":
        distanceExample(robotController, scanner);
        break;
    }
  }


  private static void magnetExample(RobotController robotController, Scanner scanner) {
    System.out.println("Press");
    System.out.println("  e to enable magnet");
    System.out.println("  d to disable magnet");
    System.out.println("  q to quit");

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
        case "q":
          return;
        default:
          System.out.println("Invalid input.");
          break;
      }
    }
  }

  private static void buzzerExample(RobotController robotController, Scanner scanner) {
    System.out.println("Press");
    System.out.println("  b to beep");
    System.out.println("  t to play tone");
    System.out.println("  ta to play tone async");
    System.out.println("  q to quit");

    while (true) {
      System.out.print("Enter input: ");
      var input = scanner.next();
      switch (input) {
        case "b":
          System.out.println("Beeping...");
          robotController.Beep();
          break;
        case "t": {
          var frequency = getFrequency(scanner);
          var duration = getDuration(scanner);
          System.out.println("Playing tone and waiting for completion...");
          robotController.PlayTone(frequency, duration);
          System.out.println("Finished playing tone.");
          break;
        }
        case "ta": {
          var frequency = getFrequency(scanner);
          var duration = getDuration(scanner);
          System.out.println("Playing tone...");
          robotController.PlayToneAsync(frequency, duration);
          break;
        }
        case "q":
          return;
        default:
          System.out.println("Invalid input.");
          break;
      }
    }
  }

  private static void colorExample(RobotController robotController, Scanner scanner) {
    System.out.println("Press");
    System.out.println("  c to get color");
    System.out.println("  q to quit");

    while (true) {
      System.out.print("Enter input: ");
      var input = scanner.next();
      switch (input) {
        case "c":
          var color = robotController.GetColor();
          System.out.println(MessageFormat.format("Color (R,G,B): {0}, {1}, {2}", color.red(), color.green(), color.blue()));
          break;
        case "q":
          return;
        default:
          System.out.println("Invalid input.");
          break;
      }
    }
  }

  private static void distanceExample(RobotController robotController, Scanner scanner) {
    System.out.println("Press");
    System.out.println("  f to get distance in front");
    System.out.println("  b to get distance in back");
    System.out.println("  q to quit");

    while (true) {
      System.out.print("Enter input: ");
      var input = scanner.next();
      switch (input) {
        case "f": {
          var distance = robotController.GetDistanceFront();
          System.out.println(MessageFormat.format("Distance to nearest obstacle in front of the robot: {0}", distance));
          break;
        }
        case "b": {
          var distance = robotController.GetDistanceBack();
          System.out.println(MessageFormat.format("Distance to nearest obstacle behind the robot: {0}", distance));
          break;
        }
        case "q":
          return;
        default:
          System.out.println("Invalid input.");
          break;
      }
    }
  }

  private static int getFrequency(Scanner scanner) {
    System.out.print("Enter frequency: ");
    return scanner.nextInt();
  }

  private static int getDuration(Scanner scanner) {
    System.out.print("Enter duration: ");
    return scanner.nextInt();
  }
}