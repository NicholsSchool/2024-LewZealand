package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

  @AutoLog
  public static class OuttakeIOInputs {
    public double velocityRPMs = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(OuttakeIOInputs inputs) {}

  /** Set the motor voltage */
  public void setVoltage(double volts);

  /** Enable or disable brake mode on the motors. */
  public void setBrakeMode(boolean brake);

  /** Set the direction on the morors. */
  public void setDirection(boolean forward);
}
