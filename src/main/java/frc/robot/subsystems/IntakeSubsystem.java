// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  //Instantiating pivot motor, intake motor, and transfer motor
  private CANSparkMax pivotIntakeMotor;
  private CANSparkMax intakeMotor;

  private CANSparkFlex transferMotor;

  //Instantiating PID Controller for controlling intake pivot
  private SparkPIDController pivotControl;

  //Instantiating Absolute Encoder for Pivot reading
  private AbsoluteEncoder intakeAbsEnc;

  //Instantiating intake sensor reading
  private DigitalInput intakeSensor;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    //Assigning the motors their IDs
    pivotIntakeMotor = new CANSparkMax(IntakeConstants.pivotMotorID, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);

    //Assigning the PID Controller to the pivot motor built in speed controller
    pivotControl = pivotIntakeMotor.getPIDController();

    //Assigning the Absolute Encoder to the pivot motor
    intakeAbsEnc = pivotIntakeMotor.getAbsoluteEncoder(Type.kDutyCycle);

    //Assigning the DIO port to our infrared sensor
    intakeSensor = new DigitalInput(IntakeConstants.irSensorDIOPort);

    //Resetting the motors to factory default and clears sticky faults
    pivotIntakeMotor.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();

    pivotIntakeMotor.clearFaults();
    intakeMotor.clearFaults();

    //Configuring the pivotIntakeMotor and Intake Motor
    pivotIntakeMotor.setIdleMode(IdleMode.kCoast);//Don't let the motor move much even when robot is disabled
    pivotIntakeMotor.setSmartCurrentLimit(20);//Sets the amount of amps the motor should at max take (Units:amps)

    intakeMotor.setIdleMode(IdleMode.kCoast);//Don't let the motor move much even when robot is disabled
    intakeMotor.setSmartCurrentLimit(40);//Sets the amount of amps the motor should at max take (Units:amps)

    //Configuring the PID Controller
    pivotControl.setP(IntakeConstants.kP);//Sets the P gain (Starting power to reach set position closely)
    pivotControl.setI(IntakeConstants.kI);//Sets the I gain (Error compensation power to reach the exact position)
    pivotControl.setD(IntakeConstants.kD);//Sets the D gain (Stop power (Basically helps with smoother stop of mechanism))
    pivotControl.setFeedbackDevice(intakeAbsEnc);//Sets the encoder we want the PID to follow 

    //Configuring Absolute Encoder
    intakeAbsEnc.setPositionConversionFactor(360);
    intakeAbsEnc.setInverted(false);

    //Burns flash to finish configuring the motors (Basically saying to set the configurations)
    pivotIntakeMotor.burnFlash();
    intakeMotor.burnFlash();
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
    SmartDashboard.putNumber("Intake Angle", getPivotAngle());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //Gets the angle of the intake
  public double getPivotAngle(){
    return intakeAbsEnc.getPosition();
  }

  //Gets the sensors feedback as to if it detects something or not
  public boolean getIRSensor(){
    return intakeSensor.get();
  }

  public void setIntakeAngle(double angle){
    pivotControl.setReference(angle, ControlType.kPosition);
  }

  //Sets the intake speed for intaking and outtaking
  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);
  }
}
