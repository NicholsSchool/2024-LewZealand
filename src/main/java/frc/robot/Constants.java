// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

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
  public static final boolean tuningMode = false;
  public static final double loopPeriodSecs = 0.02;

  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      return robot;
    } else {
      return RobotType.ROBOT_SIM;
    }
  }

  public static enum RobotType {
    ROBOT_REAL, // a real robot
    ROBOT_REPLAY, // data file replay (could be on real bot or simulation)
    ROBOT_SIM // simulation
  }

  // CAN IDs (Controller Area Network)
  public static final class CAN {
    public static final int kRearRightDrivingCanId = 28;
    public static final int kRearRightTurningCanId = 27;
    public static final int kFrontRightDrivingCanId = 26;
    public static final int kFrontRightTurningCanId = 25;
    public static final int kFrontLeftDrivingCanId = 24;
    public static final int kFrontLeftTurningCanId = 23;
    public static final int kRearLeftDrivingCanId = 22;
    public static final int kRearLeftTurningCanId = 21;
  }

  // REV MAXSwerve Modules
  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = 0.0762; // 3 inch wheels
    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2.0;
    public static final double kDrivingMotorFreeSpinRPM = 5676; // NEO 550s max RPM

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;
    public static final double kDrivingFF = 0.0;
    public static final double kDrivingStaticFF = 0.1;
    public static final double kDrivingVelocityFF = 0.13;

    public static final double kTurningP = 1.0;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;
    public static final double kTurningFF = 0.0;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 24; // amps
    public static final int kTurningMotorCurrentLimit = 12; // amps
  }
}
