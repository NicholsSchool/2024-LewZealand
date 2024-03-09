package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class VomitCommand extends Command {

  public VomitCommand(Intake intake) {
    intake.runVomitCommand();
  }
}
