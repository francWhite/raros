package io.github.francwhite.raros.client.controller;

import io.github.francwhite.raros.client.connector.*;

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
  public Status GetStatus() {
    try {
      return this.statusService.GetStatus().join();
    } catch (RuntimeException e) {
      return new Status(false, false, false, false, false);
    }
  }

  @Override
  public void EnableMagnet() {
    this.magnetService.SetMagnetState(true).join();
  }

  @Override
  public void DisableMagnet() {
    this.magnetService.SetMagnetState(false).join();
  }

  @Override
  public void Beep() {
    this.buzzerService.PlayTone(15000, 100).join();
  }

  @Override
  public void PlayTone(int frequency, int duration) {
    var action = this.buzzerService.PlayTone(frequency, duration);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void PlayToneAsync(int frequency, int duration) {
    this.buzzerService.PlayTone(frequency, duration);
  }

  @Override
  public Color GetColor() {
    return this.colorService.GetColor().join();
  }

  @Override
  public float GetDistanceFront() {
    var distance = this.distanceService.GetDistance().join();
    return distance.getFront();
  }

  @Override
  public float GetDistanceFront(int angle) {
    this.distanceService.RotateSensor(angle).join();
    var distance = this.distanceService.GetDistance().join();
    return distance.getFront();
  }

  @Override
  public float GetDistanceBack() {
    var distance = this.distanceService.GetDistance().join();
    return distance.getBack();
  }

  @Override
  public void RotateRangeSensor(int angle) {
    this.distanceService.RotateSensor(angle).join();
  }

  @Override
  public void StopMovement() {
    this.navigationService.Stop().join();
  }

  @Override
  public void MoveForward(double distance) {
    var action = this.navigationService.Move(distance, Direction.Forward);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void MoveForward(double distance, int speed) {
    var action = this.navigationService.Move(distance, speed, Direction.Forward);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void MoveForward(double distance, int startSpeed, int endSpeed) {
    var action = this.navigationService.Move(distance, startSpeed, endSpeed, Direction.Forward);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void MoveForwardAsync() {
    this.navigationService.Move(Direction.Forward);
  }

  @Override
  public void MoveForwardAsync(double distance) {
    this.navigationService.Move(distance, Direction.Forward);
  }

  @Override
  public void MoveForwardAsync(double distance, int speed) {
    this.navigationService.Move(distance, speed, Direction.Forward);
  }

  @Override
  public void MoveForwardAsync(double distance, int startSpeed, int endSpeed) {
    this.navigationService.Move(distance, startSpeed, endSpeed, Direction.Forward);
  }

  @Override
  public void MoveBackward(double distance) {
    var action = this.navigationService.Move(distance, Direction.Backward);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void MoveBackward(double distance, int speed) {
    var action = this.navigationService.Move(distance, speed, Direction.Backward);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void MoveBackward(double distance, int startSpeed, int endSpeed) {
    var action = this.navigationService.Move(distance, startSpeed, endSpeed, Direction.Backward);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void MoveBackwardAsync() {
    this.navigationService.Move(Direction.Backward);
  }

  @Override
  public void MoveBackwardAsync(double distance) {
    this.navigationService.Move(distance, Direction.Backward);
  }

  @Override
  public void MoveBackwardAsync(double distance, int speed) {
    this.navigationService.Move(distance, speed, Direction.Backward);
  }

  @Override
  public void MoveBackwardAsync(double distance, int startSpeed, int endSpeed) {
    this.navigationService.Move(distance, startSpeed, endSpeed, Direction.Backward);
  }

  @Override
  public void RotateLeft(double angle) {
    var action = this.navigationService.Rotate(angle, Direction.Left);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void RotateLeftAsync(double angle) {
    this.navigationService.Rotate(angle, Direction.Left);
  }

  @Override
  public void RotateRight(double angle) {
    var action = this.navigationService.Rotate(angle, Direction.Right);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void RotateRightAsync(double angle) {
    this.navigationService.Rotate(angle, Direction.Right);
  }

  @Override
  public void TurnLeft(double angle) {
    var action = this.navigationService.Turn(angle, 0, Direction.Left);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void TurnLeftAsync(double angle) {
    this.navigationService.Turn(angle, 0, Direction.Left);
  }

  @Override
  public void TurnLeft(double angle, double radius) {
    var action = this.navigationService.Turn(angle, radius, Direction.Left);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void TurnLeftAsync(double angle, double radius) {
    this.navigationService.Turn(angle, radius, Direction.Left);
  }

  @Override
  public void TurnRight(double angle) {
    var action = this.navigationService.Turn(angle, 0, Direction.Right);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void TurnRightAsync(double angle) {
    this.navigationService.Turn(angle, 0, Direction.Right);
  }

  @Override
  public void TurnRight(double angle, double radius) {
    var action = this.navigationService.Turn(angle, radius, Direction.Right);
    this.actionAwaiter.WaitForAction(action);
  }

  @Override
  public void TurnRightAsync(double angle, double radius) {
    this.navigationService.Turn(angle, radius, Direction.Right);
  }

  @Override
  public String CaptureImage() {
    return this.cameraService.CaptureImage().join();
  }

  @Override
  public String CaptureImage(int angleHorizontal, int angleVertical) {
    this.cameraService.RotateCamera(angleHorizontal, angleVertical).join();
    return this.cameraService.CaptureImage().join();
  }

  @Override
  public void RotateCamera(int angleHorizontal, int angleVertical) {
    this.cameraService.RotateCamera(angleHorizontal, angleVertical).join();
  }
}
