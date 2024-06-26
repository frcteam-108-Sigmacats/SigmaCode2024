// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterMechConstants;

public class IntakeSubsystem extends SubsystemBase {
  //Instantiating pivot motor, intake motor, and transfer motor
    private CANSparkMax pivotIntakeMotor;
    private CANSparkMax intakeMotor;

    private CANSparkMax transferMotor;

  //Instantiating PID Controller for controlling intake pivot
    private SparkPIDController pivotControl;

  //Instantiating Absolute Encoder for Pivot reading
    private AbsoluteEncoder intakeAbsEnc;

  //Instantiating intake sensor reading
    private DigitalInput intakeSensor, transferSensor;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    //Assigning the motors their IDs
      pivotIntakeMotor = new CANSparkMax(IntakeConstants.pivotMotorID, MotorType.kBrushless);
      intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);
      transferMotor = new CANSparkMax(IntakeConstants.transferMotorID, MotorType.kBrushless);

    //Assigning the PID Controller to the pivot motor built in speed controller
      pivotControl = pivotIntakeMotor.getPIDController();

    //Assigning the Absolute Encoder to the pivot motor
      intakeAbsEnc = pivotIntakeMotor.getAbsoluteEncoder(Type.kDutyCycle);

    //Assigning the DIO port to our infrared sensor
      intakeSensor = new DigitalInput(IntakeConstants.irSensorDIOPort);
      transferSensor = new DigitalInput(IntakeConstants.irSensorTransferDIOPort);

    //Resetting the motors to factory default and clears sticky faults
      pivotIntakeMotor.restoreFactoryDefaults();
      intakeMotor.restoreFactoryDefaults();
      transferMotor.restoreFactoryDefaults();

      pivotIntakeMotor.clearFaults();
      intakeMotor.clearFaults();
      transferMotor.clearFaults();

    // //Configuring the pivotIntakeMotor, Intake Motor, Transfer Motor
    //   pivotIntakeMotor.setIdleMode(IdleMode.kCoast);//Don't let the motor move much even when robot is disabled
    //   pivotIntakeMotor.setSmartCurrentLimit(IntakeConstants.pivotMotorCurrentLimit);//Sets the amount of amps the motor should at max take (Units:amps)

    //   intakeMotor.setIdleMode(IdleMode.kCoast);//Don't let the motor move much even when robot is disabled
    //   intakeMotor.setSmartCurrentLimit(IntakeConstants.intakeMotorCurrentLimit);//Sets the amount of amps the motor should at max take (Units:amps)

    //   transferMotor.setIdleMode(IdleMode.kCoast);
    //   transferMotor.setSmartCurrentLimit(IntakeConstants.transferMotorCurrentLimit);


    // // Enable PID wrap around for the turning motor. This will allow the PID
    // // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // // to 10 degrees will go through 0 rather than the other direction which is a
    // // longer route.
    //   pivotControl.setPositionPIDWrappingEnabled(true);
    //   pivotControl.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    //   pivotControl.setPositionPIDWrappingMaxInput(360);
    // //Configuring the PID Controller
    //   pivotControl.setP(IntakeConstants.kP);//Sets the P gain (Starting power to reach set position closely)
    //   pivotControl.setI(IntakeConstants.kI);//Sets the I gain (Error compensation power to reach the exact position)
    //   pivotControl.setD(IntakeConstants.kD);//Sets the D gain (Stop power (Basically helps with smoother stop of mechanism))
    //   pivotControl.setFF(ShooterMechConstants.pivotFF);
    //   pivotControl.setFeedbackDevice(intakeAbsEnc);//Sets the encoder we want the PID to follow 
    //   pivotControl.setOutputRange(-0.5,
    //   ModuleConstants.kTurningMaxOutput);

    // //Configuring Absolute Encoder
    //   intakeAbsEnc.setPositionConversionFactor(360);
    //   intakeAbsEnc.setInverted(false);
    configureIntakeMotor();
    configureIntakeMotor();
    configureIntakeMotor();

    configurePivotIntakeMotor();
    configurePivotIntakeMotor();
    configurePivotIntakeMotor();

    configureTransferMotor();
    configureTransferMotor();
    configureTransferMotor();

    //Burns flash to finish configuring the motors (Basically saying to set the configurations)
      pivotIntakeMotor.burnFlash();
      intakeMotor.burnFlash();
      transferMotor.burnFlash();
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
    SmartDashboard.putBoolean("Infrared Sensor reading:", getIRSensor());
    SmartDashboard.putBoolean("Transfer IR Sensor: ", getTransferSensor());
    SmartDashboard.putString("Intake Abs Reading", intakeAbsEnc.toString());
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
      //The sensor was reading the note as false so it had to be inverted
      return !intakeSensor.get();
    }
  
  public boolean isAbsEncConnected(){
      return pivotIntakeMotor.getFault(FaultID.kSensorFault);
    }

  public boolean getTransferSensor(){
      return !transferSensor.get();
    }

  //Sets the angle of the intake mechanism
    public void setIntakeAngle(double angle){
      pivotControl.setReference(angle, ControlType.kPosition);
    }

    public void testIntakePivot(double speed){
      pivotIntakeMotor.set(speed);
    }

  //Sets the intake speed for intaking and outtaking
    public void setIntakeSpeed(double speed){
      intakeMotor.set(speed);
    }

  //Sets the speed for the transfer mechanism connected to the intake to do the transfer process and reverse transfer process
    public void setTransferSpeed(double speed){
      transferMotor.set(speed);
    }

  public void configurePivotIntakeMotor(){
    //Configuring the pivotIntakeMotor
      pivotIntakeMotor.setIdleMode(IdleMode.kCoast);//Don't let the motor move much even when robot is disabled
      pivotIntakeMotor.setSmartCurrentLimit(IntakeConstants.pivotMotorCurrentLimit);//Sets the amount of amps the motor should at max take (Units:amps)

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
      pivotControl.setPositionPIDWrappingEnabled(true);
      pivotControl.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
      pivotControl.setPositionPIDWrappingMaxInput(360);
    //Configuring the PID Controller
      pivotControl.setP(IntakeConstants.kP);//Sets the P gain (Starting power to reach set position closely)
      pivotControl.setI(IntakeConstants.kI);//Sets the I gain (Error compensation power to reach the exact position)
      pivotControl.setD(IntakeConstants.kD);//Sets the D gain (Stop power (Basically helps with smoother stop of mechanism))
      pivotControl.setFF(ShooterMechConstants.pivotFF);
      pivotControl.setFeedbackDevice(intakeAbsEnc);//Sets the encoder we want the PID to follow 
      pivotControl.setOutputRange(-0.5,
      0.8);

    //Configuring Absolute Encoder
      intakeAbsEnc.setPositionConversionFactor(360);
      intakeAbsEnc.setInverted(false);
  }

  public void configureIntakeMotor(){
    //Configuring the Intake Motor
      intakeMotor.setIdleMode(IdleMode.kCoast);//Don't let the motor move much even when robot is disabled
      intakeMotor.setSmartCurrentLimit(IntakeConstants.intakeMotorCurrentLimit);//Sets the amount of amps
  }

  public void configureTransferMotor(){
    //Configuring the Transfer Motor
      transferMotor.setIdleMode(IdleMode.kCoast);//Idle Mode is either Brake(Can never rotate motor when robot is on) or Coast (Can rotate while robot is disabled)
      transferMotor.setSmartCurrentLimit(IntakeConstants.transferMotorCurrentLimit);
  }
}
