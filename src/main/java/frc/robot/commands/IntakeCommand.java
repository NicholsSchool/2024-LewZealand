package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends Command {

  public IntakeCommand(Arm arm, Intake intake) {
    arm.goToPos(Constants.ArmConstants.armIntakePos);
    intake.runEatCommand();
  }
}
