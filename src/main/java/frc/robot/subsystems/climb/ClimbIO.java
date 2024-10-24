package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double leftPositionRads = 0.0;
    public double rightPositionRads = 0.0;
    public double leftVelocityRadsPerSec = 0.0;
    public double rightVelocityRadsPerSec = 0.0;
    public double leftClimbCurrent = 0.0;
    public double rightClimbCurrent = 0.0;
    public boolean isLocked = false;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void lock() {}
  ;

  public default void unlock() {}
  ;

  public default void setVoltageLeft(double voltage) {}
  ;

  public default void setVoltageRight(double voltage) {}
  ;

  /** Stops the motors */
  public void stop();
}
