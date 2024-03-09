package frc.robot.commands.arm_commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm;

public class ArmSetTargetPos extends InstantCommand {
  private Arm arm;
  private double targetPos;

  public ArmSetTargetPos(Arm arm, double targetPos) {
    this.arm = arm;
    this.targetPos = targetPos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setTargetPos(targetPos);
  }
}
