package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;

public class DriveNotePickup extends DriveToPose {

  public DriveNotePickup(Drive drive) {
    super(
        drive,
        new Pose2d(
            drive.getPose().getX()
                + Units.inchesToMeters(14.0) * drive.getPose().getRotation().getSin(),
            drive.getPose().getY()
                + Units.inchesToMeters(14.0) * drive.getPose().getRotation().getCos(),
            drive.getRotation()));
  }
}
