package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

public class ArmToPos extends Command {
  public static enum ArmPos {
    ampPos,
    trapPos,
    drivePos,
    intakePos
  }

  public ArmToPos(Arm arm, ArmPos armPos) {

    switch (armPos) {
      case ampPos:
        arm.goToPos(Constants.ArmConstants.armAmpPos);
        break;
      case drivePos:
        arm.goToPos(Constants.ArmConstants.armDrivePos);
        break;
      case intakePos:
        arm.goToPos(Constants.ArmConstants.armIntakePos);
        break;
      case trapPos:
        arm.goToPos(Constants.ArmConstants.armTrapPos);
    }
  }

  public ArmToPos(Arm arm, double power) {
    arm.setPower(power);
  }
}
