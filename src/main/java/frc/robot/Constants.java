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
      public static final double kMaxAngularSpeed = 4 * Math.PI;

    //Distance between the centers of the left wheels and right wheels
      public static final double kTrackWidth = Units.inchesToMeters(22.4375);
    //Distance between the centers of the front wheels and back wheels
      public static final double kWheelBase = Units.inchesToMeters(22.4375);

    public static Translation2d frontL = new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0);
    public static double flModuleNorm = frontL.getNorm();

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
      public static final double kFreeSpeedRPM = 6784;
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
      public static final double kDrivingP = 0.02;
      public static final double kDrivingI = 0;
      public static final double kDrivingD = 0;
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

    //Front Right Module
      public static final int fRightDriveMotorID = 3;
      public static final int fRightTurnMotorID = 4;
      public static final double fRightAngOffset = 0;

    //Back Left Module
      public static final int bLeftDriveMotorID = 5;
      public static final int bLeftTurnMotorID = 6;
      public static final double bLeftAngOffset = 0;

    //Back Right Module
      public static final int bRightDriveMotorID = 7;
      public static final int bRightTurnMotorID = 8;
      public static final double bRightAngOffset = -Math.PI / 2;

  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  public static class IntakeConstants{
    //IDs for the motors on the intake mechanism
    public static final int pivotMotorID = 9;

    public static final int intakeMotorID = 10;

    public static final int transferMotorID = 17;

    //Current Limits for the motors on the intake/transfer mechanism
    public static final int pivotMotorCurrentLimit = 10; //Units in amps
    public static final int intakeMotorCurrentLimit = 20; //Units in amps
    public static final int transferMotorCurrentLimit = 20; //Units in amps

    //PID gains for pivot control
    public static final double kP = 0.02;//Try to make the intake pivot faster
    public static final double kI = 0.0;//Try to make the intake pivot faster
    public static final double kD = 0.0;

    //DIO port number for laser sensor that will be mounted to the intake
    public static final int irSensorDIOPort = 0;

    //Values for the intake angle's
    public static final double restPos = 0;//Switch to 25 later//The rest position which will also be the handoff position
    public static final double groundIntakePos = 132;//The position where we will intake from the ground (Find value first)
    public static final double shootIntakePos = 15;

    //The speeds for intaking, outtaking
    public static final double intakeSpeed = -0.8; //Keep at -100% unless needed to change (negative is to suck in the note and positive is to spit out if motor is not inverted)
    public static final double transferSpeed = -0.35; //Keep at -25% unless needed to change (negative is to suck in the note and positive is to spit out if motor is not inverted)
    public static final double outtakeSpeed = 1.0; //(negative is to suck in the note and positive is to spit out if motor is not inverted)
    public static final double reverseTransferSpeed = 0.5; //(negative is to suck in the note and positive is to spit out if motor is not inverted)
  }
  public static class ShooterMechConstants{
    //ID for index motor
    public static final int indexMotorID = 11;

    //IDs for the flywheel motors
    public static final int shooterLeftMotorID = 12;
    public static final int shooterRightMotorID = 13;

    //ID for shooter pivot motor
    public static final int shooterPivotMotorID = 14;

    //Limits for the motors on the shooter mechanism Units:Amps
    public static final int indexCurrentLimit = 40;//Needs to be at 40 to be able to run the indexer pipes properly
    public static final int pivotCurrentLimit = 30;
    public static final int flywheelCurrentLimit = 40;

    //Pivot angles
    public static final double restPos = 0.75; //Switch to 1 later
    public static final double ampPos = 100;
    public static final double climbPos = 110;

    //PID gains for Pivot Controller
    public static final double pivotP = 0.07;//Try to make the pivot faster
    public static final double pivotI = 0;
    public static final double pivotD = 0.001;

    //PID gains for Speed Control
    public static final double speedP = 0.01;
    public static final double speedI = 0.0;
    public static final double speedD = 0.0;

    //Should the right side of the flywheel motors need to be inverted to spin the same direction as the left side of the flywheel motor
    public static boolean rightFlywheelInverted = true;

    //Speeds for the motors
    public static final double indexTransferSpeed = -0.25;//Keep at -33% for automatic intake (negative to suck in the note and positive to spit out unless the motor is inverted)
    public static final double indexShootSpeed = -1.0;//(negative to suck in the note and positive to spit out unless the motor is inverted)
    public static final double indexOuttakeSpeed = 0.25;

    public static final double flywheelTransferSpeed = 0.1;
    public static final double flywheelShootSpeed = -1.0;
    public static final double flywheelAmpShootSpeed = -0.4;
    public static final double flywheelOuttakeSpeed = 0.5;

    public static final double minDistInches = 28;
    public static final double maxDistInches = 131.0;
    public static final double[] distSetPoints = {32  , 50, 66  , 71, 76, 81, 91, 101, 110};
    public static final double[] angleSetPoints = {9.8, 19, 22, 25, 27, 28, 30, 31, 33};
    public static final double maxDegrees = 30;
  }
  public static class ElevatorConstants{
    //IDs for elevator motors
    public static final int leftElevatorMotorID = 15;
    public static final int rightElevatorMotorID = 16;

    //PID gains for position control of elevator
    public static final double kP = 0.7;//Change when robot is with software
    public static final double kI = 0;//Change when robot is with software
    public static final double kD = 0.0;//Change when robot is with software

    //Conversion for position of elevator

    //Gear ratio of the gearbox connected to the NEO motors
    public static double elevatorGearRatio = 100.0 / 1.0;

    public static double positionConversion;

    //Positions for Elevator
    public static final double ampPos = 1;
    public static final double climbUpPos = 1.99;
    public static final double climbDownPos = 0.05;

    //Speeds for Elevator
    public static final double climbUpSpeed = 0.3;
    public static final double climbDownSpeed = -0.3;

    //Do we need to invert the motor direction to have the right motor follow the left motor
    public static boolean invertRightMotor = true;
  }

  //Class for limelight using AI Object Detection
  public static class LimelightNoteConstants{
    //How many degrees back is your limelight rotated from perfectly vertical
    public static final double limelightMountAngleDegrees = 0; //Change later when it is on the robot

    //Distance between the center of the limelight's lens to the floor
    public static final double limelightLensHeightInches = 0;

    //Distance form the target to the floor
    public static final double goalHeightInches = 0; 
  }

  //Class for limelight using AprilTag
  public static class LimelightSpeakerConstants{
    //How many degrees back is your limelight rotated from perfectly vertical
    public static final double limelightMountAngleDegrees = 45; //Change later when it is on the robot

    //Distance between the center of the limelight's lens to the floor
    public static final double limelightLensHeightInches = 6.5;

    //Distance form the target to the floor
    public static final double goalHeightInches = 77.625; 
  }
}
