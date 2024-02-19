package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmPneumatics {
  public static class ArmExtend extends Command {
    public ArmExtend(Arm arm) {
      arm.extend();
    }
  }

  public static class ArmRetract extends Command {
    public ArmRetract(Arm arm) {
      arm.retract();
    }
  }
}
