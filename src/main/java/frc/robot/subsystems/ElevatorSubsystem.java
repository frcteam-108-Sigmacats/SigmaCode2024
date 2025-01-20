// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  //Instatiating elevator motors
  private CANSparkMax leftElevatorMotor;
  private CANSparkMax rightElevatorMotor;

  private PWM leftStaticHook, rightStaticHook;

  //Instantiating PID Controller for position control on elevator
  private SparkPIDController elevatePositionControl;

  //Instantiate Relative Encoder for position control feedback
  private RelativeEncoder elevateEnc;
  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
    //Assign the IDs for the elevator motors
    leftElevatorMotor = new CANSparkMax(ElevatorConstants.leftElevatorMotorID, MotorType.kBrushless);
    rightElevatorMotor = new CANSparkMax(ElevatorConstants.rightElevatorMotorID, MotorType.kBrushless);

    leftStaticHook = new PWM(1);
    rightStaticHook = new PWM(2);

    leftElevatorMotor.restoreFactoryDefaults();
    rightElevatorMotor.restoreFactoryDefaults();

    //Assign PID controller to left motor
    elevatePositionControl = leftElevatorMotor.getPIDController();

    //Assign encoder to left motor's built in encoder
    elevateEnc = leftElevatorMotor.getAlternateEncoder(8192);

    // //Set PID gains to the PID Controller
    // elevatePositionControl.setP(ElevatorConstants.kP);
    // elevatePositionControl.setI(ElevatorConstants.kI);
    // elevatePositionControl.setD(ElevatorConstants.kD);
    // elevatePositionControl.setOutputRange(-0.98, 1.0);
    // elevatePositionControl.setFeedbackDevice(elevateEnc);
    // //Configure motors
    // leftElevatorMotor.setIdleMode(IdleMode.kBrake);
    // leftElevatorMotor.setSmartCurrentLimit(40);//Units in amps
    // leftElevatorMotor.setInverted(true);

    // rightElevatorMotor.setIdleMode(IdleMode.kBrake);
    // rightElevatorMotor.setSmartCurrentLimit(40);//Units in amps
    // rightElevatorMotor.follow(leftElevatorMotor, ElevatorConstants.invertRightMotor);
    configureElevatorMotors();
    configureElevatorMotors();
    configureElevatorMotors();
    leftElevatorMotor.setSoftLimit(SoftLimitDirection.kForward, 2.0f);
    leftElevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftElevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, 0f);
    leftElevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    //Burn flash to save configuration of motors
    leftElevatorMotor.burnFlash();
    rightElevatorMotor.burnFlash();

    elevateEnc.setPosition(0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", getElevatorPosition());//Getting the encoder count for setting position
    SmartDashboard.putNumber("Get Left Hook Angle", leftStaticHook.getPosition());
    SmartDashboard.putNumber("Get Right Hook Angle", rightStaticHook.getPosition());  
    SmartDashboard.putNumber("Left Elevator Motor:", leftElevatorMotor.get());
    SmartDashboard.putNumber("Right Motor Speed:", rightElevatorMotor.get()); 
    SmartDashboard.putNumber("Elevator P ", elevatePositionControl.getP());
    SmartDashboard.putNumber("Elevator I ", elevatePositionControl.getI());
    SmartDashboard.putNumber("Elevator D ", elevatePositionControl.getD());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //Getting the position of the elevator based on the motor's built in encoder
  public double getElevatorPosition(){
    return elevateEnc.getPosition();
  }

  //Setting the position of the elevator based on the motor's built in encoder
  public void setElevatorPosition(double position){
    // if(getElevatorPosition() < position && position >= 5){
    //   setElevatorSpeed(0.3);
    // }
    // else if(getElevatorPosition() > position && position <=5){
    //   setElevatorSpeed(-0.3);
    // }
    // else{
    //   setElevatorSpeed(0);
    // }
    elevatePositionControl.setReference(position, ControlType.kPosition);
    // Smart motion code to test
    // elevatePositionControl.setReference(position, ControlType.kSmartMotion);
  }
  
  //Moving elevator based on speeds
  public void setElevatorSpeed(double speed){
    leftElevatorMotor.set(speed);
    rightElevatorMotor.set(speed);
  }
  
  public void setServoSpeed(double speed){
    leftStaticHook.setSpeed(speed);
    rightStaticHook.setSpeed(-speed);
  }

  public void configureElevatorMotors(){
    //Set PID gains to the PID Controller
    elevatePositionControl.setP(ElevatorConstants.kP);
    elevatePositionControl.setI(ElevatorConstants.kI);
    elevatePositionControl.setD(ElevatorConstants.kD);
    elevatePositionControl.setFF(1/5676); //Test this out on Saturday 
    elevatePositionControl.setIZone(ElevatorConstants.kIZone);
    elevatePositionControl.setOutputRange(-1.0, 1.0);
    elevatePositionControl.setFeedbackDevice(elevateEnc);

    //Smart Motion code to test out
    // elevatePositionControl.setSmartMotionMaxAccel(2000, 0); //Test it out on Saturday
    // elevatePositionControl.setSmartMotionMaxVelocity(2500, 0); //Test it out on Saturday
    // elevatePositionControl.setSmartMotionMinOutputVelocity(2000, 0); //Test it out on Saturday

    //Configure motors
    leftElevatorMotor.setIdleMode(IdleMode.kBrake);
    leftElevatorMotor.setSmartCurrentLimit(40);//Units in amps change amps when testing smart motion
    leftElevatorMotor.setInverted(true);

    rightElevatorMotor.setIdleMode(IdleMode.kBrake);
    rightElevatorMotor.setSmartCurrentLimit(40);//Units in amps change amps when testing smart motion
    rightElevatorMotor.follow(leftElevatorMotor, ElevatorConstants.invertRightMotor);
  }
}
