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
      public static final double kTurningP = 1.0;
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
      public static final boolean fLDriveInvert = false;
      public static final boolean fLTurnInvert = false;

    //Front Right Module
      public static final int fRightDriveMotorID = 3;
      public static final int fRightTurnMotorID = 4;
      public static final double fRightAngOffset = 0;
      public static final boolean fRDriveInvert = false;
      public static final boolean fRTurnInvert = false;

    //Back Left Module
      public static final int bLeftDriveMotorID = 5;
      public static final int bLeftTurnMotorID = 6;
      public static final double bLeftAngOffset = 0;
      public static final boolean bLDriveInvert = true;
      public static final boolean bLTurnInvert = false;

    //Back Right Module
      public static final int bRightDriveMotorID = 7;
      public static final int bRightTurnMotorID = 8;
      public static final double bRightAngOffset = -Math.PI / 2;
      public static final boolean bRDriveInvert = true;
      public static final boolean bRTurnInvert = false;

  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
<<<<<<< HEAD
  public static class IntakeConstants{
    //IDs for the motors on the intake mechanism
    public static final int pivotMotorID = 9;

    public static final int intakeMotorID = 10;

    public static final int transferMotorID = 17;

    //Current Limits for the motors on the intake/transfer mechanism
    public static final int pivotMotorCurrentLimit = 30; //Units in amps
    public static final int intakeMotorCurrentLimit = 20; //Units in amps
    public static final int transferMotorCurrentLimit = 20; //Units in amps

    //PID gains for pivot control
    public static final double kP = 0.01;//change when given the robot
    public static final double kI = 0.0;//change when given the robot
    public static final double kD = 0.0;//change when given the robot

    //DIO port number for laser sensor that will be mounted to the intake
    public static final int irSensorDIOPort = 0;//might have to change

    //Values for the intake angle's
    public static final double restPos = 160;//The rest position which will also be the handoff position
    public static final double groundIntakePos = 290;//The position where we will intake from the ground (Find value first)
    public static final double sourceZonePos = 0;//The position where we will intake a note coming from the source zone that is not the ground (Find value first)


    //The speeds for intaking, outtaking
    public static final double intakeSpeed = -0.75; //Change speed when robot is done
    public static final double transferSpeed = -0.5; //Change speed when robot is done
    public static final double outtakeSpeed = 0.75; //Change speed when robot is done
    public static final double reverseTransferSpeed = 0.5; //Change speed when robot is done

=======
  public static class ShooterMechConstants{
    //ID for index motor
    public static final int indexMotorID = 11;

    //IDs for the flywheel motors
    public static final int shooterLeftMotorID = 12;
    public static final int shooterRightMotorID = 13;

    //ID for shooter pivot motor
    public static final int shooterPivotMotorID = 14;

    //Limits for the motors on the shooter mechanism Units:Amps
    public static final int indexCurrentLimit = 30;
    public static final int pivotCurrentLimit = 30;
    public static final int flywheelCurrentLimit = 40;

    //Pivot angles
    public static final double restPos = 1; //Change later when the robot is in software

    //PID gains for Pivot Controller
    public static final double kP = 0.01;//Change later when robot is with software
    public static final double kI = 0.0;//Change later when robot is with software
    public static final double kD = 0.0;//Change later when robot is with software

    //Port number for Shooter Sensor
    public static final int shootSensorID = 2;

    //Should the right side of the flywheel motors need to be inverted to spin the same direction as the left side of the flywheel motor
    public static boolean rightFlywheelInverted = true;

    //Speeds for the motors
    public static final double indexTransferSpeed = -1.0;
    public static final double indexShootSpeed = -1.0;

    public static final double flywheelTransferSpeed = 0.1;
>>>>>>> Shooter
  }
}
