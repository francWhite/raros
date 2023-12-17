package io.github.francwhite.raros.client.connector;

public record Status(boolean IsAvailable,
                     boolean IsMoving,
                     boolean IsPlayingTone,
                     boolean IsMagnetActive,
                     boolean IsCollisionDetectionActive) {
}
