// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;
import java.util.function.DoubleSupplier;

public class ClimbManual extends Command {
  private Climb climb;
  private DoubleSupplier manuelInput;
  /** Creates a new ArmManuel. */
  public ClimbManual(Climb climb, DoubleSupplier manuelInput) {
    this.climb = climb;
    addRequirements(climb);
    this.manuelInput = manuelInput;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setVoltage(manuelInput.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Manual Climb Cancelled");
  }
}
