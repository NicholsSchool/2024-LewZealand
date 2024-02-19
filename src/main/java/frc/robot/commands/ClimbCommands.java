package frc.robot.commands;

import frc.robot.subsystems.arm.Arm;

public class ClimbCommands {
  public static class LeftClimb {
    public LeftClimb(Arm arm, double power) {
      arm.setPower(power);
    }
  }

  public static class RightClimb {
    public RightClimb(Arm arm, double power) {
      arm.setPower(power);
    }
  }
}
