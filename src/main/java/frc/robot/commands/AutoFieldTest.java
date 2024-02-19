package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public class AutoFieldTest extends SequentialCommandGroup {
  /** Automatically drives to the amplifier. */
  public AutoFieldTest(Drive drive) {
    addCommands(
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(1).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(2).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(3).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(4).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(5).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(6).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(7).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(8).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(9).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(10).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(11).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(12).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(13).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(14).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(15).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return AllianceFlipUtil.apply(
                  FieldConstants.aprilTags.getTagPose(16).get().toPose2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return new Pose2d(
                  AllianceFlipUtil.apply(FieldConstants.NoteLocations.ampNote), new Rotation2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return new Pose2d(
                  AllianceFlipUtil.apply(FieldConstants.NoteLocations.speakerNote),
                  new Rotation2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return new Pose2d(
                  AllianceFlipUtil.apply(FieldConstants.NoteLocations.stageNote), new Rotation2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return new Pose2d(
                  AllianceFlipUtil.apply(FieldConstants.NoteLocations.centerNote1),
                  new Rotation2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return new Pose2d(
                  AllianceFlipUtil.apply(FieldConstants.NoteLocations.centerNote2),
                  new Rotation2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return new Pose2d(
                  AllianceFlipUtil.apply(FieldConstants.NoteLocations.centerNote3),
                  new Rotation2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return new Pose2d(
                  AllianceFlipUtil.apply(FieldConstants.NoteLocations.centerNote4),
                  new Rotation2d());
            }),
        new DriveToPose(
            drive,
            () -> {
              return new Pose2d(
                  AllianceFlipUtil.apply(FieldConstants.NoteLocations.centerNote5),
                  new Rotation2d());
            }));
  }
}
