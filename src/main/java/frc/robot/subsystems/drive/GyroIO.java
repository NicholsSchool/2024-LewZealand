package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

/** The interface for any GYRO connected to the robot */
public interface GyroIO {
  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs the inputs to update
   */
  default void updateInputs(GyroIOInputs inputs) {}

  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double pitchRad = 0.0;
    public double velocityPitchRadPerSec = 0.0;
    public double yawRad = 0.0;
    public double velocityYawRadPerSec = 0.0;
    public double rollRad = 0.0;
    public double velocityRollRadPerSec = 0.0;
  }
}
