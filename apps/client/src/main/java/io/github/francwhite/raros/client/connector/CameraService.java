package io.github.francwhite.raros.client.connector;

import java.util.concurrent.CompletableFuture;

public interface CameraService {
  /**
   * Captures an image from the camera.
   *
   * @return {@code CompletableFuture<String>} The captured image as a base64 encoded string.
   */
  CompletableFuture<String> captureImage();

  /**
   * Rotates the camera to a given angle.
   *
   * @param angleHorizontal The horizontal angle to rotate the camera to. The angle must be between -90 and 90.
   * @param angleVertical   The vertical angle to rotate the camera to. The angle must be between -90 and 90.
   * @return {@code CompletableFuture<Void>}
   */
  CompletableFuture<Void> rotateCamera(int angleHorizontal, int angleVertical);
}
