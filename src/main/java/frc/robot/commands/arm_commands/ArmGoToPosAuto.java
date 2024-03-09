package frc.robot.commands.arm_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmGoToPosAuto extends Command {
  private Arm arm;

  public ArmGoToPosAuto(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setGoToPos();
  }

  @Override
  public boolean isFinished() {
    return arm.hasReachedTarget();
  }
}
