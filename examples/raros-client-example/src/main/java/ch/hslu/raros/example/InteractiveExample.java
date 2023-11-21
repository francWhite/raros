package ch.hslu.raros.example;

import ch.hslu.raros.client.controller.RobotController;
import ch.hslu.raros.client.controller.RobotControllerFactory;

import java.net.URI;
import java.text.MessageFormat;
import java.util.Optional;
import java.util.Scanner;

public class InteractiveExample {
  public static void main(String[] args) {
    var robotController = RobotControllerFactory.create(URI.create("http://eee-03300:8000"));
    var scanner = new Scanner(System.in);

    System.out.println("Press");
    System.out.println("  m to control magnet");
    System.out.println("  b to control buzzer");
    System.out.println("  c to get color");
    System.out.println("  d to get distance or rotate distance sensor");
    System.out.println("  n to navigate");

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
      case "n":
        navigationExample(robotController, scanner);
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
    System.out.println("  r to rotate distance sensor");
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
        case "r": {
          System.out.print("Enter angle: ");
          var angle = scanner.nextInt();
          robotController.RotateRangeSensor(angle);
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

  private static void navigationExample(RobotController robotController, Scanner scanner) {
    System.out.println("Press");
    System.out.println("  f to move forward");
    System.out.println("  fa to move forward async");
    System.out.println("  b to move backward");
    System.out.println("  ba to move backward async");
    System.out.println("  s to stop");
    System.out.println("  q to quit");

    while (true) {
      System.out.print("Enter input: ");
      var input = scanner.next();
      switch (input) {
        case "f": {
          var moveRequest = getMoveRequest(scanner);
          if (moveRequest.speed().isPresent()) {
            System.out.println("Moving forward and waiting for completion...");
            robotController.MoveForward(moveRequest.distance(), moveRequest.speed().get());
            System.out.println("Finished moving forward.");
          } else {
            System.out.println("Moving forward and waiting for completion...");
            robotController.MoveForward(moveRequest.distance());
            System.out.println("Finished moving forward.");
          }
          break;
        }
        case "fa": {
          var moveRequest = getMoveRequest(scanner);
          if (moveRequest.speed().isPresent()) {
            System.out.println("Moving forward...");
            robotController.MoveForwardAsync(moveRequest.distance(), moveRequest.speed().get());
          } else {
            System.out.println("Moving forward...");
            robotController.MoveForwardAsync(moveRequest.distance());
          }
          break;
        }
        case "b": {
          var moveRequest = getMoveRequest(scanner);
          if (moveRequest.speed().isPresent()) {
            System.out.println("Moving backward and waiting for completion...");
            robotController.MoveBackward(moveRequest.distance(), moveRequest.speed().get());
            System.out.println("Finished moving backward.");
          } else {
            System.out.println("Moving backward and waiting for completion...");
            robotController.MoveBackward(moveRequest.distance());
            System.out.println("Finished moving backward.");
          }
          break;
        }
        case "ba": {
          var moveRequest = getMoveRequest(scanner);
          if (moveRequest.speed().isPresent()) {
            System.out.println("Moving backward...");
            robotController.MoveBackwardAsync(moveRequest.distance(), moveRequest.speed().get());
          } else {
            System.out.println("Moving backward...");
            robotController.MoveBackwardAsync(moveRequest.distance());
          }
          break;
        }
        case "s":
          System.out.println("Stopping...");
          robotController.StopMovement();
          break;
        case "q":
          return;
        default:
          System.out.println("Invalid input.");
          break;
      }
    }
  }

  private record MoveRequest(double distance, Optional<Double> speed) {
  }

  private static MoveRequest getMoveRequest(Scanner scanner) {
    System.out.print("With speed? (y/n): ");
    var withSpeed = scanner.next();
    if (withSpeed.equals("y")) {
      System.out.print("Enter distance: ");
      var distance = scanner.nextDouble();
      System.out.print("Enter speed: ");
      var speed = scanner.nextDouble();
      return new MoveRequest(distance, Optional.of(speed));
    } else {
      System.out.print("Enter distance: ");
      var distance = scanner.nextDouble();
      return new MoveRequest(distance, Optional.empty());
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