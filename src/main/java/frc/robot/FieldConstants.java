package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise.
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall. Use the {@link #allianceFlip(Translation2d)} and {@link #allianceFlip(Pose2d)}
 * methods to flip these values based on the current alliance color.
 */
public final class FieldConstants {
  public static final boolean isWPIField = false; // Red alliance

  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth =
      Units.inchesToMeters(315.5) + (isWPIField ? Units.inchesToMeters(3.0) : 0.0);
  public static final double tapeWidth = Units.inchesToMeters(2.0);

  public static final Translation2d amplifierTranslation = new Translation2d(1.828, 7.704);

  // Locations of all the game pieces on the field in auto
  public static final class NoteLocations {
    public static final Translation2d ampNote =
        new Translation2d(Units.inchesToMeters(114.0), Units.inchesToMeters(277.74));

    public static final Translation2d speakerNote =
        new Translation2d(Units.inchesToMeters(114.0), Units.inchesToMeters(220.74));

    public static final Translation2d stageNote =
        new Translation2d(Units.inchesToMeters(114.0), Units.inchesToMeters(163.74));

    // the 5 center notes are ordered starting with the amp side
    // <link>https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
    // page 6

    public static final Translation2d centerNote1 =
        new Translation2d(Units.inchesToMeters(335.3), Units.inchesToMeters(293.64));

    public static final Translation2d centerNote2 =
        new Translation2d(Units.inchesToMeters(335.3), Units.inchesToMeters(227.64));

    public static final Translation2d centerNote3 =
        new Translation2d(Units.inchesToMeters(335.3), Units.inchesToMeters(161.64));

    public static final Translation2d centerNote4 =
        new Translation2d(Units.inchesToMeters(335.3), Units.inchesToMeters(95.64));

    public static final Translation2d centerNote5 =
        new Translation2d(Units.inchesToMeters(335.3), Units.inchesToMeters(29.64));
  }

  public static final AprilTagFieldLayout aprilTags =
      new AprilTagFieldLayout(
          List.of(
              new AprilTag(
                  1,
                  new Pose3d(
                      Units.inchesToMeters(593.68),
                      Units.inchesToMeters(9.68),
                      Units.inchesToMeters(53.38),
                      new Rotation3d(120.0, 0.0, Math.PI))),
              new AprilTag(
                  2,
                  new Pose3d(
                      Units.inchesToMeters(637.21),
                      Units.inchesToMeters(34.79),
                      Units.inchesToMeters(53.38),
                      new Rotation3d(120.0, 0.0, Math.PI))),
              new AprilTag(
                  3,
                  new Pose3d(
                      Units.inchesToMeters(652.73),
                      Units.inchesToMeters(196.17),
                      Units.inchesToMeters(57.13),
                      new Rotation3d(180.0, 0.0, Math.PI))),
              new AprilTag(
                  4,
                  new Pose3d(
                      Units.inchesToMeters(652.73),
                      Units.inchesToMeters(218.42),
                      Units.inchesToMeters(57.13),
                      new Rotation3d(180.0, 0.0, Math.PI))),
              new AprilTag(
                  5,
                  new Pose3d(
                      Units.inchesToMeters(578.77),
                      Units.inchesToMeters(323.00),
                      Units.inchesToMeters(53.38),
                      new Rotation3d(270.0, 0.0, Math.PI))),
              new AprilTag(
                  6,
                  new Pose3d(
                      Units.inchesToMeters(72.5),
                      Units.inchesToMeters(323.00),
                      Units.inchesToMeters(53.38),
                      new Rotation3d(270.0, 0.0, Math.PI))),
              new AprilTag(
                  7,
                  new Pose3d(
                      Units.inchesToMeters(-1.50),
                      Units.inchesToMeters(218.42),
                      Units.inchesToMeters(57.13),
                      new Rotation3d(0.0, 0.0, Math.PI))),
              new AprilTag(
                  8,
                  new Pose3d(
                      Units.inchesToMeters(-1.50),
                      Units.inchesToMeters(196.17),
                      Units.inchesToMeters(57.13),
                      new Rotation3d(0.0, 0.0, Math.PI))),
              new AprilTag(
                  9,
                  new Pose3d(
                      Units.inchesToMeters(14.02),
                      Units.inchesToMeters(34.79),
                      Units.inchesToMeters(53.38),
                      new Rotation3d(60.0, 0.0, Math.PI))),
              new AprilTag(
                  10,
                  new Pose3d(
                      Units.inchesToMeters(57.54),
                      Units.inchesToMeters(9.68),
                      Units.inchesToMeters(53.38),
                      new Rotation3d(60.0, 0.0, Math.PI))),
              new AprilTag(
                  11,
                  new Pose3d(
                      Units.inchesToMeters(468.69),
                      Units.inchesToMeters(146.19),
                      Units.inchesToMeters(52.00),
                      new Rotation3d(300.0, 0.0, Math.PI))),
              new AprilTag(
                  12,
                  new Pose3d(
                      Units.inchesToMeters(468.69),
                      Units.inchesToMeters(177.10),
                      Units.inchesToMeters(52.00),
                      new Rotation3d(60.0, 0.0, Math.PI))),
              new AprilTag(
                  13,
                  new Pose3d(
                      Units.inchesToMeters(441.74),
                      Units.inchesToMeters(161.62),
                      Units.inchesToMeters(52.00),
                      new Rotation3d(180.0, 0.0, Math.PI))),
              new AprilTag(
                  14,
                  new Pose3d(
                      Units.inchesToMeters(209.48),
                      Units.inchesToMeters(161.62),
                      Units.inchesToMeters(52.00),
                      new Rotation3d(0.0, 0.0, Math.PI))),
              new AprilTag(
                  15,
                  new Pose3d(
                      Units.inchesToMeters(182.73),
                      Units.inchesToMeters(177.10),
                      Units.inchesToMeters(52.00),
                      new Rotation3d(120.0, 0.0, Math.PI))),
              new AprilTag(
                  16,
                  new Pose3d(
                      Units.inchesToMeters(182.73),
                      Units.inchesToMeters(146.19),
                      Units.inchesToMeters(52.00),
                      new Rotation3d(240.0, 0.0, Math.PI)))),
          fieldLength,
          fieldWidth);
}
