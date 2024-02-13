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
    public static final double restPos = 23;//The rest position which will also be the handoff position
    public static final double groundIntakePos = 210;//The position where we will intake from the ground (Find value first)
    public static final double sourceZonePos = 0;//The position where we will intake a note coming from the source zone that is not the ground (Find value first)


    //The speeds for intaking, outtaking
    public static final double intakeSpeed = 0.75; //Change speed when robot is done
    public static final double transferSpeed = -0.5; //Change speed when robot is done
    public static final double outtakeSpeed = -0.75; //Change speed when robot is done
    public static final double reverseTransferSpeed = 0.5; //Change speed when robot is done

  }
}