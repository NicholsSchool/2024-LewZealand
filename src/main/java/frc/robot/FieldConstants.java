// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// TJG

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

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

  // this is a list of note positions.
  public static final Translation2d posNote1 = new Translation2d(1.0, 5.0);

  /* 2023 sample
  // Dimensions for loading zone and substations, including the tape
  public static final class LoadingZone {
    // Region dimensions
    public static final double width = Units.inchesToMeters(99.0);
    public static final double innerX = FieldConstants.fieldLength;
    public static final double midX = fieldLength - Units.inchesToMeters(132.25);
    public static final double outerX = fieldLength - Units.inchesToMeters(264.25);
    public static final double leftY = FieldConstants.fieldWidth;
    public static final double midY = leftY - Units.inchesToMeters(50.5);
    public static final double rightY = leftY - width;
    public static final Translation2d[] regionCorners =
        new Translation2d[] {
          new Translation2d(
              midX, rightY), // Start at lower left next to border with opponent community
          new Translation2d(midX, midY),
          new Translation2d(outerX, midY),
          new Translation2d(outerX, leftY),
          new Translation2d(innerX, leftY),
          new Translation2d(innerX, rightY),
        };

    // Double substation dimensions
    public static final double doubleSubstationLength = Units.inchesToMeters(14.0);
    public static final double doubleSubstationX = innerX - doubleSubstationLength;
    public static final double doubleSubstationShelfZ = Units.inchesToMeters(37.375);
    public static final double doubleSubstationCenterY = fieldWidth - Units.inchesToMeters(49.76);

    // Single substation dimensions
    public static final double singleSubstationWidth = Units.inchesToMeters(22.75);
    public static final double singleSubstationLeftX =
        FieldConstants.fieldLength - doubleSubstationLength - Units.inchesToMeters(88.77);
    public static final double singleSubstationCenterX =
        singleSubstationLeftX + (singleSubstationWidth / 2.0);
    public static final double singleSubstationRightX =
        singleSubstationLeftX + singleSubstationWidth;
    public static final Translation2d singleSubstationTranslation =
        new Translation2d(singleSubstationCenterX, leftY);

    public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
    public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
    public static final double singleSubstationCenterZ =
        singleSubstationLowZ + (singleSubstationHeight / 2.0);
    public static final double singleSubstationHighZ =
        singleSubstationLowZ + singleSubstationHeight;
  }

  // Locations of staged game pieces
  public static final class StagingLocations {
    public static final double centerOffsetX = Units.inchesToMeters(47.36);
    public static final double positionX = fieldLength / 2.0 - Units.inchesToMeters(47.36);
    public static final double firstY = Units.inchesToMeters(36.19);
    public static final double separationY = Units.inchesToMeters(48.0);
    public static final Translation2d[] translations = new Translation2d[4];

    static {
      for (int i = 0; i < translations.length; i++) {
        translations[i] = new Translation2d(positionX, firstY + (i * separationY));
      }
    }
  }

    public static final AprilTagFieldLayout aprilTags =
        isWPIField
            ? new AprilTagFieldLayout(
                List.of(
                    new AprilTag(
                        1,
                        new Pose3d(
                            Units.inchesToMeters(610.125),
                            Units.inchesToMeters(43.5),
                            Units.inchesToMeters(19.25),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        2,
                        new Pose3d(
                            Units.inchesToMeters(610.375),
                            Units.inchesToMeters(109.5),
                            Units.inchesToMeters(19.25),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        3,
                        new Pose3d(
                            Units.inchesToMeters(610.0),
                            Units.inchesToMeters(176.0),
                            Units.inchesToMeters(19.25),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        4,
                        new Pose3d(
                            Units.inchesToMeters(635.375),
                            Units.inchesToMeters(272.0),
                            Units.inchesToMeters(27.25),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        5,
                        new Pose3d(
                            Units.inchesToMeters(14.25),
                            LoadingZone.doubleSubstationCenterY,
                            Units.inchesToMeters(27.38),
                            new Rotation3d())),
                    new AprilTag(
                        6,
                        new Pose3d(
                            Units.inchesToMeters(40.45),
                            Grids.nodeY[7],
                            Units.inchesToMeters(18.22),
                            new Rotation3d())),
                    new AprilTag(
                        7,
                        new Pose3d(
                            Units.inchesToMeters(40.45),
                            Grids.nodeY[4],
                            Units.inchesToMeters(18.22),
                            new Rotation3d())),
                    new AprilTag(
                        8,
                        new Pose3d(
                            Units.inchesToMeters(40.45),
                            Grids.nodeY[1],
                            Units.inchesToMeters(18.22),
                            new Rotation3d()))),
                fieldLength,
                fieldWidth)
            : new AprilTagFieldLayout(
                List.of(
                    new AprilTag(
                        1,
                        new Pose3d(
                            Units.inchesToMeters(610.77),
                            Grids.nodeY[1],
                            Units.inchesToMeters(18.22),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        2,
                        new Pose3d(
                            Units.inchesToMeters(610.77),
                            Grids.nodeY[4],
                            Units.inchesToMeters(18.22),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        3,
                        new Pose3d(
                            Units.inchesToMeters(610.77),
                            Grids.nodeY[7],
                            Units.inchesToMeters(18.22),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        4,
                        new Pose3d(
                            Units.inchesToMeters(636.96),
                            LoadingZone.doubleSubstationCenterY,
                            Units.inchesToMeters(27.38),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        5,
                        new Pose3d(
                            Units.inchesToMeters(14.25),
                            LoadingZone.doubleSubstationCenterY,
                            Units.inchesToMeters(27.38),
                            new Rotation3d())),
                    new AprilTag(
                        6,
                        new Pose3d(
                            Units.inchesToMeters(40.45),
                            Grids.nodeY[7],
                            Units.inchesToMeters(18.22),
                            new Rotation3d())),
                    new AprilTag(
                        7,
                        new Pose3d(
                            Units.inchesToMeters(40.45),
                            Grids.nodeY[4],
                            Units.inchesToMeters(18.22),
                            new Rotation3d())),
                    new AprilTag(
                        8,
                        new Pose3d(
                            Units.inchesToMeters(40.45),
                            Grids.nodeY[1],
                            Units.inchesToMeters(18.22),
                            new Rotation3d()))),
                fieldLength,
                fieldWidth);
  */
}
