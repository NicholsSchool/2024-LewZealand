package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;

public class DriveToNote extends SequentialCommandGroup {

  public DriveToNote(Drive drive) {
    System.out.println(" inside drive to ntoe ");
    addCommands(
        new DriveToPose(
            drive,
            new Pose2d(
                FieldConstants.NoteLocations.centerNote1.getX(),
                FieldConstants.NoteLocations.centerNote1.getY(),
                new Rotation2d(0.0))),
        new DriveToPose(
            drive,
            new Pose2d(
                FieldConstants.NoteLocations.centerNote5.getX(),
                FieldConstants.NoteLocations.centerNote5.getY(),
                new Rotation2d(90.0))));
  }
}
