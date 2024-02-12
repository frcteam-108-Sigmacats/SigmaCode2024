// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ChassisConstants{
    //Max Drive and Turn Speed Restrictions
      public static final double kMaxSpeedMPS = 12;
      public static final double kMaxAngularSpeed = 2 * Math.PI;

    //Distance between the centers of the left wheels and right wheels
      public static final double kTrackWidth = Units.inchesToMeters(22.4375);
    //Distance between the centers of the front wheels and back wheels
      public static final double kWheelBase = Units.inchesToMeters(22.4375);

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), 
      new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
      new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
      new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

    //Is Gyro inverted
      public static final boolean gyroReversed = false;

    //Used to prevent stick drift
      public static double deadband = 0.15;
  }
  public static class ModuleConstants{
    //Driving Pinion Teeth
      public static final int driveMotorPinionTeeth = 14;

    //Is Through Bore Encoder Reversed
      public static final boolean turningEncoderInverted = true;

    //Calculations for Drive Motor Conversion factors and feed forward
      public static final double kFreeSpeedRPM = 5676;
      public static final double kDrivingMotorFreeSpeedRPS = kFreeSpeedRPM / 60;

      public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
      public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

      //45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (driveMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRPS * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

      public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
      public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

      public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
      public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
      public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
      public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians
    
    //PID Set Up for Drive and Turn Motors
      public static final double kDrivingP = 0.04;//Change when robot is with software
      public static final double kDrivingI = 0;//Change when robot is with software
      public static final double kDrivingD = 0;//Change when robot is with software
      public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
      public static final double kDrivingMinOutput = -1;
      public static final double kDrivingMaxOutput = 1;

      public static final double kTurningP = 1;
      public static final double kTurningI = 0;
      public static final double kTurningD = 0;
      public static final double kTurningFF = 0;
      public static final double kTurningMinOutput = -1;
      public static final double kTurningMaxOutput = 1;

    //Drive and Turn Motor Current Limit Configuration
      public static final int kDrivingMotorCurrentLimit = 50; // amps
      public static final int kTurningMotorCurrentLimit = 20; // amps

    //Front Left Module
      public static final int fLeftDriveMotorID = 1;
      public static final int fLeftTurnMotorID = 2;
      public static final double fLeftAngOffset = -Math.PI / 2;

    //Front Right Module
      public static final int fRightDriveMotorID = 3;
      public static final int fRightTurnMotorID = 4;
      public static final double fRightAngOffset = 0;

    //Back Left Module
      public static final int bLeftDriveMotorID = 5;
      public static final int bLeftTurnMotorID = 6;
      public static final double bLeftAngOffset = Math.PI;

    //Back Right Module
      public static final int bRightDriveMotorID = 7;
      public static final int bRightTurnMotorID = 8;
      public static final double bRightAngOffset = Math.PI / 2;

  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
