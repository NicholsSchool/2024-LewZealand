// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import java.util.function.DoubleSupplier;

public class ArmManuel extends Command {
  private Arm arm;
  private DoubleSupplier manuelInput;
  /** Creates a new ArmManuel. */
  public ArmManuel(Arm arm, DoubleSupplier manuelInput) {
    this.arm = arm;
    addRequirements(arm);
    this.manuelInput = manuelInput;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setManuel(manuelInput.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Manual Cancelled");
  }
}
