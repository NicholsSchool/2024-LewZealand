package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class ResetFieldOrientation extends Command {
  public ResetFieldOrientation(Drive drive) {
    drive.setPose(new Pose2d(drive.getPose().getX(), drive.getPose().getY(), new Rotation2d(0.0)));
  }
}
