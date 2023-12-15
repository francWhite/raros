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
    System.out.println("  s to get the current status");
    System.out.println("  m to control magnet");
    System.out.println("  b to control buzzer");
    System.out.println("  c to get color");
    System.out.println("  d to get distance or rotate distance sensor");
    System.out.println("  n to navigate");

    var input = scanner.next();
    switch (input) {
      case "s":
        statusExample(robotController, scanner);
        break;
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

  private static void statusExample(RobotController robotController, Scanner scanner) {
    System.out.println("Press");
    System.out.println("  s to get the status");
    System.out.println("  q to quit");

    while (true) {
      var input = scanner.next();
      switch (input) {
        case "s":
          var status = robotController.GetStatus();
          System.out.println(MessageFormat.format("Status: {0}", status));
          break;
        case "q":
          return;
        default:
          System.out.println("Invalid input.");
          break;
      }
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
    System.out.println("  s to stop");
    System.out.println("  f to move forward");
    System.out.println("  fa to move forward async");
    System.out.println("  b to move backward");
    System.out.println("  ba to move backward async");
    System.out.println("  rl to rotate left");
    System.out.println("  rla to rotate left async");
    System.out.println("  rr to rotate right");
    System.out.println("  rra to rotate right async");
    System.out.println("  tl to turn left");
    System.out.println("  tr to turn right");
    System.out.println("  q to quit");

    while (true) {
      System.out.print("Enter input: ");
      var input = scanner.next();
      switch (input) {
        case "s":
          System.out.println("Stopping...");
          robotController.StopMovement();
          break;
        case "f": {
          var moveRequest = getMoveRequest(scanner);
          System.out.println("Moving forward and waiting for completion...");
          if (moveRequest.startSpeed().isEmpty() && moveRequest.endSpeed().isEmpty()) {
            robotController.MoveForward(moveRequest.distance());
          } else if (moveRequest.startSpeed().isPresent() && moveRequest.endSpeed().isEmpty()) {
            robotController.MoveForward(moveRequest.distance(), moveRequest.startSpeed().get());
          } else {
            robotController.MoveForward(moveRequest.distance(), moveRequest.startSpeed().get(), moveRequest.endSpeed().get());
          }
          System.out.println("Finished moving forward.");
          break;
        }
        case "fa": {
          var moveRequest = getMoveRequest(scanner);
          System.out.println("Moving forward...");
          if (moveRequest.startSpeed().isEmpty() && moveRequest.endSpeed().isEmpty()) {
            robotController.MoveForwardAsync(moveRequest.distance());
          } else if (moveRequest.startSpeed().isPresent() && moveRequest.endSpeed().isEmpty()) {
            robotController.MoveForwardAsync(moveRequest.distance(), moveRequest.startSpeed().get());
          } else {
            robotController.MoveForwardAsync(moveRequest.distance(), moveRequest.startSpeed().get(), moveRequest.endSpeed().get());
          }
          break;
        }
        case "b": {
          var moveRequest = getMoveRequest(scanner);
          System.out.println("Moving backward and waiting for completion...");
          if (moveRequest.startSpeed().isEmpty() && moveRequest.endSpeed().isEmpty()) {
            robotController.MoveBackward(moveRequest.distance());
          } else if (moveRequest.startSpeed().isPresent() && moveRequest.endSpeed().isEmpty()) {
            robotController.MoveBackward(moveRequest.distance(), moveRequest.startSpeed().get());
          } else {
            robotController.MoveBackward(moveRequest.distance(), moveRequest.startSpeed().get(), moveRequest.endSpeed().get());
          }
          System.out.println("Finished moving backward.");
          break;
        }
        case "ba": {
          var moveRequest = getMoveRequest(scanner);
          System.out.println("Moving backward...");
          if (moveRequest.startSpeed().isEmpty() && moveRequest.endSpeed().isEmpty()) {
            robotController.MoveBackwardAsync(moveRequest.distance());
          } else if (moveRequest.startSpeed().isPresent() && moveRequest.endSpeed().isEmpty()) {
            robotController.MoveBackwardAsync(moveRequest.distance(), moveRequest.startSpeed().get());
          } else {
            robotController.MoveBackwardAsync(moveRequest.distance(), moveRequest.startSpeed().get(), moveRequest.endSpeed().get());
          }
          break;
        }
        case "rl": {
          var angle = getAngle(scanner);
          System.out.println("Rotating left and waiting for completion...");
          robotController.RotateLeft(angle);
          System.out.println("Finished rotating");
          break;
        }
        case "rla": {
          var angle = getAngle(scanner);
          System.out.println("Rotating left...");
          robotController.RotateLeftAsync(angle);
          break;
        }
        case "rr": {
          var angle = getAngle(scanner);
          System.out.println("Rotating right and waiting for completion...");
          robotController.RotateRight(angle);
          System.out.println("Finished rotating");
          break;
        }
        case "rra": {
          var angle = getAngle(scanner);
          System.out.println("Rotating right...");
          robotController.RotateRightAsync(angle);
          break;
        }
        case "tl": {
          var turnRequest = getTurnRequest(scanner);
          System.out.println("Turning left and waiting for completion...");
          if (turnRequest.radius().isPresent()) {
            robotController.TurnLeft(turnRequest.angle(), turnRequest.radius().get());
          } else {
            robotController.TurnLeft(turnRequest.angle());
          }
          System.out.println("Finished turning");
          break;
        }
        case "tr": {
          var turnRequest = getTurnRequest(scanner);
          System.out.println("Turning right and waiting for completion...");
          if (turnRequest.radius().isPresent()) {
            robotController.TurnRight(turnRequest.angle(), turnRequest.radius().get());
          } else {
            robotController.TurnRight(turnRequest.angle());
          }
          System.out.println("Finished turning");
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

  private record MoveRequest(double distance, Optional<Integer> startSpeed, Optional<Integer> endSpeed) {
  }

  private static MoveRequest getMoveRequest(Scanner scanner) {
    System.out.print("With speed? (y/n): ");
    var withSpeed = scanner.next();
    System.out.print("Enter distance: ");
    var distance = scanner.nextDouble();

    if (withSpeed.equals("y")) {
      System.out.print("With acceleration? (y/n): ");
      var withAcceleration = scanner.next();
      if (withAcceleration.equals("y")) {
        System.out.print("Enter start speed: ");
        var startSpeed = scanner.nextInt();
        System.out.print("Enter end speed: ");
        var endSpeed = scanner.nextInt();
        return new MoveRequest(distance, Optional.of(startSpeed), Optional.of(endSpeed));
      } else {
        System.out.print("Enter speed: ");
        var speed = scanner.nextInt();
        return new MoveRequest(distance, Optional.of(speed), Optional.of(speed));
      }
    } else {
      return new MoveRequest(distance, Optional.empty(), Optional.empty());
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

  private static double getAngle(Scanner scanner) {
    System.out.print("Enter angle: ");
    return scanner.nextDouble();
  }

  private static double getRadius(Scanner scanner) {
    System.out.print("Enter radius: ");
    return scanner.nextDouble();
  }

  private record TurnRequest(double angle, Optional<Double> radius) {
  }

  private static TurnRequest getTurnRequest(Scanner scanner) {
    System.out.print("With radius? (y/n): ");
    var withRadius = scanner.next();
    var angle = getAngle(scanner);
    if (withRadius.equals("y")) {
      var radius = getRadius(scanner);
      return new TurnRequest(angle, Optional.of(radius));
    } else {
      return new TurnRequest(angle, Optional.empty());
    }
  }
}