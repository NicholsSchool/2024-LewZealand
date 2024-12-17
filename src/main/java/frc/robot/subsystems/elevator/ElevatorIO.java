package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double leftPositionRads = 0.0;
    public double rightPositionRads = 0.0;
    public double leftVelocityRadsPerSec = 0.0;
    public double rightVelocityRadsPerSec = 0.0;
    public double leftClimbCurrent = 0.0;
    public double rightClimbCurrent = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltageLeft(double power) {}

  public default void setVoltageRight(double power) {}

  public default void stopMotors() {}
}
