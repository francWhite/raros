package io.github.francwhite.raros.client.controller;

import io.github.francwhite.raros.client.connector.*;

import java.io.FileOutputStream;
import java.io.IOException;

class RobotControllerImpl implements RobotController {
  private final ActionAwaiter actionAwaiter;
  private final MagnetService magnetService;
  private final BuzzerService buzzerService;
  private final ColorService colorService;
  private final DistanceService distanceService;
  private final NavigationService navigationService;
  private final StatusService statusService;
  private final CameraService cameraService;

  public RobotControllerImpl(ActionAwaiter actionAwaiter,
                             MagnetService magnetService,
                             BuzzerService buzzerService,
                             ColorService colorService,
                             DistanceService distanceService,
                             NavigationService navigationService,
                             StatusService statusService,
                             CameraService cameraService) {
    this.actionAwaiter = actionAwaiter;
    this.magnetService = magnetService;
    this.buzzerService = buzzerService;
    this.colorService = colorService;
    this.distanceService = distanceService;
    this.navigationService = navigationService;
    this.statusService = statusService;
    this.cameraService = cameraService;
  }

  @Override
  public Status getStatus() {
    try {
      return this.statusService.getStatus().join();
    } catch (RuntimeException e) {
      return new Status(false, false, false, false, false);
    }
  }

  @Override
  public void enableMagnet() {
    this.magnetService.setMagnetState(true).join();
  }

  @Override
  public void disableMagnet() {
    this.magnetService.setMagnetState(false).join();
  }

  @Override
  public void beep() {
    this.buzzerService.playTone(15000, 100).join();
  }

  @Override
  public void playTone(int frequency, int duration) {
    var action = this.buzzerService.playTone(frequency, duration);
    this.actionAwaiter.waitForAction(action);
  }

  @Override
  public void playToneAsync(int frequency, int duration) {
    this.buzzerService.playTone(frequency, duration);
  }

  @Override
  public Color getColor() {
    return this.colorService.getColor().join();
  }

  @Override
  public float getDistanceFront() {
    var distance = this.distanceService.getDistance().join();
    return distance.getFront();
  }

  @Override
  public float getDistanceFront(int angle) {
    this.distanceService.rotateSensor(angle).join();
    var distance = this.distanceService.getDistance().join();
    return distance.getFront();
  }

  @Override
  public float getDistanceBack() {
    var distance = this.distanceService.getDistance().join();
    return distance.getBack();
  }

  @Override
  public void rotateRangeSensor(int angle) {
    this.distanceService.rotateSensor(angle).join();
  }

  @Override
  public void stopMovement() {
    this.navigationService.stop().join();
  }

  @Override
  public void moveForward(double distance) {
    var action = this.navigationService.move(distance, Direction.Forward);
    this.actionAwaiter.waitForAction(action);
  }

  @Override
  public void moveForward(double distance, int speed) {
    var action = this.navigationService.move(distance, speed, Direction.Forward);
    this.actionAwaiter.waitForAction(action);
  }

  @Override
  public void moveForward(double distance, int startSpeed, int endSpeed) {
    var action = this.navigationService.move(distance, startSpeed, endSpeed, Direction.Forward);
    this.actionAwaiter.waitForAction(action);
  }

  @Override
  public void moveForwardAsync() {
    this.navigationService.move(Direction.Forward);
  }

  @Override
  public void moveForwardAsync(double distance) {
    this.navigationService.move(distance, Direction.Forward);
  }

  @Override
  public void moveForwardAsync(double distance, int speed) {
    this.navigationService.move(distance, speed, Direction.Forward);
  }

  @Override
  public void moveForwardAsync(double distance, int startSpeed, int endSpeed) {
    this.navigationService.move(distance, startSpeed, endSpeed, Direction.Forward);
  }

  @Override
  public void moveBackward(double distance) {
    var action = this.navigationService.move(distance, Direction.Backward);
    this.actionAwaiter.waitForAction(action);
  }

  @Override
  public void moveBackward(double distance, int speed) {
    var action = this.navigationService.move(distance, speed, Direction.Backward);
    this.actionAwaiter.waitForAction(action);
  }

  @Override
  public void moveBackward(double distance, int startSpeed, int endSpeed) {
    var action = this.navigationService.move(distance, startSpeed, endSpeed, Direction.Backward);
    this.actionAwaiter.waitForAction(action);
  }

  @Override
  public void moveBackwardAsync() {
    this.navigationService.move(Direction.Backward);
  }

  @Override
  public void moveBackwardAsync(double distance) {
    this.navigationService.move(distance, Direction.Backward);
  }

  @Override
  public void moveBackwardAsync(double distance, int speed) {
    this.navigationService.move(distance, speed, Direction.Backward);
  }

  @Override
  public void moveBackwardAsync(double distance, int startSpeed, int endSpeed) {
    this.navigationService.move(distance, startSpeed, endSpeed, Direction.Backward);
  }

  @Override
  public void rotateLeft(double angle) {
    var action = this.navigationService.rotate(angle, Direction.Left);
    this.actionAwaiter.waitForAction(action);
  }

  @Override
  public void rotateLeftAsync(double angle) {
    this.navigationService.rotate(angle, Direction.Left);
  }

  @Override
  public void rotateRight(double angle) {
    var action = this.navigationService.rotate(angle, Direction.Right);
    this.actionAwaiter.waitForAction(action);
  }

  @Override
  public void rotateRightAsync(double angle) {
    this.navigationService.rotate(angle, Direction.Right);
  }

  @Override
  public void turnLeft(double angle) {
    var action = this.navigationService.turn(angle, 0, Direction.Left);
    this.actionAwaiter.waitForAction(action);
  }

  @Override
  public void turnLeftAsync(double angle) {
    this.navigationService.turn(angle, 0, Direction.Left);
  }

  @Override
  public void turnLeft(double angle, double radius) {
    var action = this.navigationService.turn(angle, radius, Direction.Left);
    this.actionAwaiter.waitForAction(action);
  }

  @Override
  public void turnLeftAsync(double angle, double radius) {
    this.navigationService.turn(angle, radius, Direction.Left);
  }

  @Override
  public void turnRight(double angle) {
    var action = this.navigationService.turn(angle, 0, Direction.Right);
    this.actionAwaiter.waitForAction(action);
  }

  @Override
  public void turnRightAsync(double angle) {
    this.navigationService.turn(angle, 0, Direction.Right);
  }

  @Override
  public void turnRight(double angle, double radius) {
    var action = this.navigationService.turn(angle, radius, Direction.Right);
    this.actionAwaiter.waitForAction(action);
  }

  @Override
  public void turnRightAsync(double angle, double radius) {
    this.navigationService.turn(angle, radius, Direction.Right);
  }

  @Override
  public String captureImage() {
    var image = this.cameraService.captureImage().join();
    return this.saveImage(image);
  }

  @Override
  public String captureImage(int angleHorizontal, int angleVertical) {
    this.cameraService.rotateCamera(angleHorizontal, angleVertical).join();
    var image = this.cameraService.captureImage().join();
    return this.saveImage(image);
  }

  @Override
  public void rotateCamera(int angleHorizontal, int angleVertical) {
    this.cameraService.rotateCamera(angleHorizontal, angleVertical).join();
  }

  private String saveImage(String imageBase64) {
    var imageBytes = java.util.Base64.getDecoder().decode(imageBase64);
    var outputDirectory = new java.io.File("images");
    //noinspection ResultOfMethodCallIgnored
    outputDirectory.mkdir();

    var filePath = outputDirectory.getAbsolutePath()
      + "/image-"
      + java.time.LocalDateTime.now().toString().replace(":", "-")
      + ".jpg";

    try (FileOutputStream fos = new FileOutputStream(filePath)) {
      fos.write(imageBytes);
      return filePath;
    } catch (IOException e) {
      throw new RuntimeException(e.getMessage(), e);
    }
  }
}
