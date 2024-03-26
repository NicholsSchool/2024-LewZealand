package frc.robot.commands.arm_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmGoToPosAuto extends Command {
  private Arm arm;
  private double targetPos;

  public ArmGoToPosAuto(Arm arm, double targetPos) {
    this.arm = arm;
    this.targetPos = targetPos;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setTargetPos(targetPos);
    arm.setGoToPos();
  }

  @Override
  public boolean isFinished() {
    return arm.hasReachedTarget();
  }
}
