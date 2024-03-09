package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double angleRads = 0.0;
    public double angleDegs = 0.0;
    public double velocityRadsPerSec = 0.0;
    public boolean isExtended = false;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Set voltage command */
  public default void setVoltage(double voltage) {}

  /** Retracts Pistons */
  public default void retract() {}

  /** Extends Pistons */
  public default void extend() {}
}
