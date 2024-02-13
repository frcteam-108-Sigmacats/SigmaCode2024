// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ShooterMechConstants{
    //ID for index motor
      public static final int indexMotorID = 11;

    //Speed to run index motor
      public static final double indexMotorSpeed = 1.0; //Change when robot is with software

    //IDs for the flywheel motors
      public static final int shooterLeftMotorID = 12;
      public static final int shooterRightMotorID = 13;

    //ID for shooter pivot motor
      public static final int shooterPivotMotorID = 14;

    //Current Limits for all motors in the shooter mechanism
      public static final int flywheelCurrentLimit = 40;//Units in amps
      public static final int pivotCurrentLimit = 30;//Units in amps
      public static final int indexCurrentLimit = 20;//Units in amps

    //Pivot angles
      public static final double restPos = 1; //Change later when the robot is in software

    //PID gains for Pivot Controller
      public static final double kP = 0.07;//Change later when robot is with software
      public static final double kI = 0.0;//Change later when robot is with software
      public static final double kD = 0.0;//Change later when robot is with software

    //Port number for Shooter Sensor
      public static final int transferSensorID = 2;

    //Should the right side of the flywheel motors need to be inverted to spin the same direction as the left side of the flywheel motor
      public static boolean rightFlywheelInverted = true;
  }
}