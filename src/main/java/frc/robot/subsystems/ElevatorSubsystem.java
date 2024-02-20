// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  //Instatiating elevator motors
  private CANSparkMax leftElevatorMotor;
  private CANSparkMax rightElevatorMotor;

  //Instantiating PID Controller for position control on elevator
  private SparkPIDController elevatePositionControl;

  //Instantiate Relative Encoder for position control feedback
  private RelativeEncoder elevateEnc;
  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
    //Assign the IDs for the elevator motors
    leftElevatorMotor = new CANSparkMax(ElevatorConstants.leftElevatorMotorID, MotorType.kBrushless);
    rightElevatorMotor = new CANSparkMax(ElevatorConstants.rightElevatorMotorID, MotorType.kBrushless);

    //Assign PID controller to left motor
    elevatePositionControl = leftElevatorMotor.getPIDController();

    //Assign encoder to left motor's built in encoder
    elevateEnc = leftElevatorMotor.getEncoder();

    //Set PID gains to the PID Controller
    elevatePositionControl.setP(ElevatorConstants.kP);
    elevatePositionControl.setI(ElevatorConstants.kI);
    elevatePositionControl.setD(ElevatorConstants.kD);
    elevatePositionControl.setFeedbackDevice(elevateEnc);

    elevateEnc.setPositionConversionFactor(ElevatorConstants.positionConversion);

    //Configure motors
    leftElevatorMotor.setIdleMode(IdleMode.kBrake);
    leftElevatorMotor.setSmartCurrentLimit(40);//Units in amps

    rightElevatorMotor.setIdleMode(IdleMode.kBrake);
    rightElevatorMotor.setSmartCurrentLimit(40);//Units in amps

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
    elevatePositionControl.setReference(position, ControlType.kPosition);
    rightElevatorMotor.follow(leftElevatorMotor, ElevatorConstants.invertRightMotor);
  }
}
