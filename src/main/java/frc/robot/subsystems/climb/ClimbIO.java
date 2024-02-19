package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double leftPosition = 0.0;
    public double rightPosition = 0.0;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Stops the motors */
  public void stop();
}
