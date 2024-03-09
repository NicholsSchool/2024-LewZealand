package frc.robot.commands.arm_commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm;

public class ArmGoToPosTeleop extends InstantCommand {
  private Arm arm;

  public ArmGoToPosTeleop(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setGoToPos();
  }
}
