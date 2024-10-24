package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private static final RobotType robot = RobotType.ROBOT_REAL;
  public static final boolean driveRobotRelative =
      false; // set to true to override all field relative and instead command in robot-relative.

  // true to place tunable numbers in dashboard for setting, false otherwise
  public static final boolean tuningMode = true;
  public static final double loopPeriodSecs = 0.02;
  public static final double MeterPerInch = 0.0254;
  public static final double KgPerLb = 0.453592;

  public static final double JOYSTICK_DEADBAND = 0.05;
  public static final String ClimbConstants = null;

  public static RobotType getRobot() {
    return RobotBase.isReal() ? robot : RobotType.ROBOT_SIM;
  }

  public static enum RobotType {
    ROBOT_REAL, // a real robot
    ROBOT_REPLAY, // data file replay (could be on real bot or simulation)
    ROBOT_SIM, // simulation
    ROBOT_FOOTBALL // Football for simulating
  }

  // CAN IDs (Controller Area Network)
  public static final class CAN {

    public static final int kIntakeCanId = 41;
    public static final int kOuttakeCanId = 42;

    public static final int kLeftClimberId = 34;
    public static final int kRightClimberId = 33;

    public static final int kArmLeaderCanId = 31;
    public static final int kArmFollowerCanId = 32;

    public static final int kRearRightDrivingCanId = 28;
    public static final int kRearRightTurningCanId = 27;
    public static final int kFrontRightDrivingCanId = 26;
    public static final int kFrontRightTurningCanId = 25;
    public static final int kFrontLeftDrivingCanId = 24;
    public static final int kFrontLeftTurningCanId = 23;
    public static final int kRearLeftDrivingCanId = 22;
    public static final int kRearLeftTurningCanId = 21;

    public static final int kPowerDistributionHub = 50;
  }

  public static final class RobotConstants {
    public static final double robotSideLengthInches =
        // 34.0; // robot was measured bumper to bumper to be 33in, +1 in for buffer.
        33.0; // why do we need a +1 buffer? J-Burnett
  }

  public static final class DriveConstants {
    public static final double kMAX_LINEAR_SPEED = 4.8;
    public static final double kTRACK_WIDTH_X = 0.5969; // 23.5in
    public static final double kTRACK_WIDTH_Y = 0.5969;

    public static final double lowGearScaler = 0.6;
  }

  // REV MAXSwerve Modules
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = 0.0762; // 3 inch wheels
    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2.0;
    public static final double kDrivingMotorFreeSpinRPM = 5676; // NEO 550s max RPM
    public static final double odometryCoefficient = 1.0; // TODO: tune odometry and use this

    public static final double kDrivingP = 0.02; // TODO: tune these
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;
    public static final double kDrivingFF = 0.0;
    public static final double kDrivingStaticFF = 0.1;
    public static final double kDrivingVelocityFF = 0.13;

    public static final double kTurningP = 1.78; // TODO: tune these
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;
    public static final double kTurningFF = 0.0;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 24; // amps
    public static final int kTurningMotorCurrentLimit = 24; // amps

    public static final double kDRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double kTURN_GEAR_RATIO = 150.0 / 7.0;
  }

  public static final class IntakeConstants {
    public static final int kBeamBreakChannel = 0;
    public static final double kVomitDelay = 0.5;
    public static final double kP = 6.0;
    public static final double kI = 0.0;
    public static final double kEatRPM = 800.0;
    public static final double kVomitRPM = -1000.0;
    public static final double kPoopRPM = 1000.0;
    public static final double kDigestRPM = 600.0;
  }

  public static final class ArmConstants {
    // NO GO TO POSITION ANGLES SHOULD BE OVER 108 DEGREES
    public static final double armIntakePosDeg = 0.0;
    public static final double armDrivePosDeg = 45.0;
    public static final double armAmpPosDeg = 108.0;
    public static final double armTrapPosDeg = 75.0;
    public static final double armStartPosDeg = 90.0; // pulls arm off of brake

    public static final int ARM_LOCK_SOLENOID_CHANNEL = 6;
    public static final int ARM_SOLENOID_CHANNEL = 7;

    public static final int ARM_CURRENT_LIMIT = 35;

    public static final double MIN_ANGLE_RADS = 0.0;
    public static final double MAX_ANGLE_RADS = Math.PI;

    public static final double ARM_MASS_LBS = 25.0;
    public static final double ARM_COM_DISTANCE_INCHES = 20.0;
    public static final double ARM_LENGTH_METERS = ARM_COM_DISTANCE_INCHES * MeterPerInch;

    // 1:98 planetary gear ratio
    public static final double ARM_GEAR_REDUCTION = 98.0;
    public static final double ARM_GEAR_RATIO = 1.0 / ARM_GEAR_REDUCTION;

    public static final double ARM_FF_KS = 0.0; // TODO: tune these
    public static final double ARM_FF_KG = 0.4;
    public static final double ARM_FF_KV = 1.91;
    public static final double ARM_FF_KA = 0.05;

    public static final double ARM_P = 4.0;
    public static final double ARM_I = 3.0;
    public static final double ARM_D = 0.0;
    public static final double ARM_ACCEL_LIMIT = 1.1;
    public static final double ARM_VEL_LIMIT = 0.85167;
  }

  public static final class OuttakeConstants {
    public static final double kP = 1.0; // TODO: tune these
    public static final double kD = 0.0;
    public static final double GEAR_RATIO_REDUCTION = 3.0;
    ;
    public static final double CURRENT_LIMIT = 35.0;
  }

  public static final class AutoConstants {
    public static final double driveFinishThreshold = 0.075; // TODO: tune these
    public static final double angleFinishThreshold = Math.PI / 12.0;
  }

  public static final class FiddleSongs {
    public static final String ALL_STAR = "all-star.chrp"; // TODO: add fnaf
    public static final String IMPERIAL_MARCH = "Imperial-March.chrp";
    public static final String WII_SONG = "Wii-Song.chrp";
  }
}
