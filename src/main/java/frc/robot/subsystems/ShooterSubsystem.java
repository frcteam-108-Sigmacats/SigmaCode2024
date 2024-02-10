// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
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
import frc.robot.Constants.ShooterMechConstants;

public class ShooterSubsystem extends SubsystemBase {
  //Instantiating the motors in the intake mechanism
  private CANSparkMax shooterPivotMotor;
  //Motors for running the flywheels
  private CANSparkFlex shooterLeftMotor;
  private CANSparkFlex shooterRightMotor;

  //Motor for running index rollers
  private TalonFX indexMotor;

  private CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();
  private MotorOutputConfigs motorOutput = new MotorOutputConfigs();

  //Instantiate the shooter pivot absolute encoder
  private AbsoluteEncoder shooterPivotAbsEnc;

  //Instantiate PID Controller for shooter pivot
  private SparkPIDController pivotControl;

  //Instantiate Infrared Sensor
  private DigitalInput shooterSensor;


  private double indexSpeed = 0;
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    //Assigning ID to shooter pivot motor
    shooterPivotMotor = new CANSparkMax(ShooterMechConstants.shooterPivotMotorID, MotorType.kBrushless);

    //Assigning ID to flywheel motors
    shooterLeftMotor = new CANSparkFlex(ShooterMechConstants.shooterLeftMotorID, MotorType.kBrushless);
    shooterRightMotor = new CANSparkFlex(ShooterMechConstants.shooterRightMotorID, MotorType.kBrushless);

    //Assinging ID to Index Motor
    indexMotor = new TalonFX(ShooterMechConstants.indexMotorID);

    //Assinging the port number for the infrared sensor
    shooterSensor = new DigitalInput(ShooterMechConstants.shootSensorID);

    //Resetting the motors
    shooterPivotMotor.restoreFactoryDefaults();
    
    shooterLeftMotor.restoreFactoryDefaults();
    shooterRightMotor.restoreFactoryDefaults();

    indexMotor.getConfigurator().apply(new TalonFXConfiguration());

    currentLimit.SupplyCurrentLimit = 40.0;
    motorOutput.NeutralMode = NeutralModeValue.Coast;

    //Assinging the absolute encoder to the pivot motor
    shooterPivotAbsEnc = shooterPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

    //Assinging pivot controller to the pivot motor built in PID controller
    pivotControl = shooterPivotMotor.getPIDController();

    //Configuration of everything

    //Configuring index motor
      indexMotor.getConfigurator().apply(currentLimit);
      indexMotor.getConfigurator().apply(motorOutput);
    //Configuring flywheel motors
    shooterLeftMotor.setSmartCurrentLimit(40);//Units are in amps
    shooterLeftMotor.setIdleMode(IdleMode.kCoast);

    shooterRightMotor.setSmartCurrentLimit(40);//Units are in amps
    shooterRightMotor.setIdleMode(IdleMode.kCoast);

    //Configuring pivot motor
    shooterPivotMotor.setSmartCurrentLimit(20);//Units are in amps
    shooterPivotMotor.setIdleMode(IdleMode.kCoast);

    //Configure Pivot PID Controller
    pivotControl.setP(ShooterMechConstants.kP);
    pivotControl.setI(ShooterMechConstants.kI);
    pivotControl.setD(ShooterMechConstants.kD);
    pivotControl.setFeedbackDevice(shooterPivotAbsEnc);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    pivotControl.setPositionPIDWrappingEnabled(true);
    pivotControl.setPositionPIDWrappingMinInput(0);
    pivotControl.setPositionPIDWrappingMaxInput(360);

    pivotControl.setOutputRange(-1, 1);

    //Setting Absolute Encoder to 360 degree position and inversion
    shooterPivotAbsEnc.setPositionConversionFactor(360);
    shooterPivotAbsEnc.setInverted(true);

    //Burn flash on motors

    shooterPivotMotor.burnFlash();

    shooterLeftMotor.burnFlash();
    shooterRightMotor.burnFlash();
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
    SmartDashboard.putNumber("Shooter Angle", getPivotAngle());
    // SmartDashboard.putNumber("Set Kraken Speed", indexSpeed);
    indexSpeed = SmartDashboard.getNumber("Set Kraken Speed", 0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getPivotAngle(){
    return shooterPivotAbsEnc.getPosition();
  }

  public boolean getIRSensor(){
    return shooterSensor.get();
  }

  public void setPivotAngle(double angle){
    pivotControl.setReference(angle, ControlType.kPosition);
  }
  public void setFlyWheelSpeeds(double speed){
    shooterLeftMotor.set(speed);
    shooterRightMotor.follow(shooterLeftMotor, ShooterMechConstants.rightFlywheelInverted);
  }
  public void setIndexRollerSpeed(){
    indexMotor.set(indexSpeed);
  }
  public void setIndexRollerSpeed(double speed){
    indexMotor.set(speed);
  }
}
