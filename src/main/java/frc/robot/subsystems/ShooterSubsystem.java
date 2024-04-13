// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterMechConstants;

public class ShooterSubsystem extends SubsystemBase {
  //Instantiating the motors in the intake mechanism
  private CANSparkMax shooterPivotMotor;
  //Motors for running the flywheels
  private CANSparkFlex shooterLeftMotor;
  private CANSparkFlex shooterRightMotor;

  //Motor for running index rollers
  private CANSparkFlex indexMotor;

  //Instantiate the shooter pivot absolute encoder and relative encoder for speed reading
  private AbsoluteEncoder shooterPivotAbsEnc;
  private RelativeEncoder leftShooterSpeedRelEnc, rightShooterSpeedRelEnc;

  //Instantiate PID Controller for shooter pivot and speed control
  private SparkPIDController pivotControl, leftSpeedControl, rightSpeedControl;

  private double angle = 0;


  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    //Assigning ID to shooter pivot motor
    shooterPivotMotor = new CANSparkMax(ShooterMechConstants.shooterPivotMotorID, MotorType.kBrushless);

    //Assigning ID to flywheel motors
    shooterLeftMotor = new CANSparkFlex(ShooterMechConstants.shooterLeftMotorID, MotorType.kBrushless);
    shooterRightMotor = new CANSparkFlex(ShooterMechConstants.shooterRightMotorID, MotorType.kBrushless);

    //Assinging ID to Index Motor
    indexMotor = new CANSparkFlex(ShooterMechConstants.indexMotorID, MotorType.kBrushless);

    //Resetting the motors
    shooterPivotMotor.restoreFactoryDefaults();
    
    shooterLeftMotor.restoreFactoryDefaults();
    shooterRightMotor.restoreFactoryDefaults();

    indexMotor.restoreFactoryDefaults();

    //Assinging the absolute encoder to the pivot motor and relative encoder to the one of the flywheel motors
    shooterPivotAbsEnc = shooterPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // shooterPivotAbsEnc.setPositionConversionFactor(360);
    // shooterPivotAbsEnc.setInverted(true);

    leftShooterSpeedRelEnc = shooterLeftMotor.getEncoder();
    rightShooterSpeedRelEnc = shooterRightMotor.getEncoder();

    //Assinging pivot controller to the pivot motor built in PID controller and flywheel motor built in PID Controller
    pivotControl = shooterPivotMotor.getPIDController();

    leftSpeedControl = shooterLeftMotor.getPIDController();
    rightSpeedControl = shooterRightMotor.getPIDController();

    //Configuration of everything

    // //Configuring index motor
    // indexMotor.setSmartCurrentLimit(ShooterMechConstants.indexCurrentLimit);//Unit is in amps
    // indexMotor.setIdleMode(IdleMode.kCoast);

    // //Configuring flywheel motors
    // shooterLeftMotor.setSmartCurrentLimit(ShooterMechConstants.flywheelCurrentLimit);//Units are in amps
    // shooterLeftMotor.setIdleMode(IdleMode.kBrake);

    // shooterRightMotor.setSmartCurrentLimit(ShooterMechConstants.flywheelCurrentLimit);//Units are in amps
    // shooterRightMotor.setIdleMode(IdleMode.kBrake);

    // //Configuring pivot motor
    // shooterPivotMotor.setSmartCurrentLimit(ShooterMechConstants.pivotCurrentLimit);//Units are in amps
    // shooterPivotMotor.setIdleMode(IdleMode.kBrake);

    // //Configure Pivot PID Controller
    // // Enable PID wrap around for the turning motor. This will allow the PID
    // // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // // to 10 degrees will go through 0 rather than the other direction which is a
    // // longer route.
    //   pivotControl.setPositionPIDWrappingEnabled(true);
    //   pivotControl.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    //   pivotControl.setPositionPIDWrappingMaxInput(360);
    // pivotControl.setP(ShooterMechConstants.pivotP);
    // pivotControl.setI(ShooterMechConstants.pivotI);
    // pivotControl.setD(ShooterMechConstants.pivotD);
    // pivotControl.setFF(ShooterMechConstants.pivotFF);
    // pivotControl.setFeedbackDevice(shooterPivotAbsEnc);
    // pivotControl.setOutputRange(-0.25,
    // 0.3);

    // leftSpeedControl.setP(ShooterMechConstants.speedP);
    // leftSpeedControl.setI(ShooterMechConstants.speedI);
    // leftSpeedControl.setD(ShooterMechConstants.speedD);
    // leftSpeedControl.setFF(ShooterMechConstants.speedFF);
    // leftSpeedControl.setFeedbackDevice(leftShooterSpeedRelEnc);

    // rightSpeedControl.setP(ShooterMechConstants.speedP);
    // rightSpeedControl.setI(ShooterMechConstants.speedI);
    // rightSpeedControl.setD(ShooterMechConstants.speedD);
    // rightSpeedControl.setFF(ShooterMechConstants.speedFF);
    // rightSpeedControl.setFeedbackDevice(rightShooterSpeedRelEnc);
    configureIndexMotor();
    configureIndexMotor();
    configureIndexMotor();

    configureShooterMotors();
    configureShooterMotors();
    configureShooterMotors();

    configureShooterPivotMotor();
    configureShooterPivotMotor();
    configureShooterPivotMotor();

    //Burn flash on motors
    indexMotor.burnFlash();

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
    angle = SmartDashboard.getNumber("Shooter Angle", 0);
    SmartDashboard.putNumber("Get Shooter Angle", getPivotAngle());
    SmartDashboard.putNumber("Flywheel Velocity", getFlywheelVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }

  public double getPivotAngle(){
    return shooterPivotAbsEnc.getPosition();
  }

  public double getSetAngle(){
    return angle;
  }

  public double getFlywheelVelocity(){
    return leftShooterSpeedRelEnc.getVelocity();
  }

  public boolean isAbsEncConnected(){
    return shooterPivotMotor.getFault(FaultID.kSensorFault);
  }

  public void setPivotAngle(double angle){
    pivotControl.setReference(angle, ControlType.kPosition);
  }
  
  public void setPivotAngle(){
    pivotControl.setReference(this.angle, ControlType.kPosition);
  }

  public void setPivotSpeed(double speed){
    shooterPivotMotor.set(speed);
  }

  public void setFlyWheelSpeeds(double speed){
    shooterLeftMotor.set(speed);
    shooterRightMotor.set((-speed) - 0.15);
  }

  public void setFlyWheelVelocity(double rpm){
    double rightShooterRPM = rpm + (ModuleConstants.kFreeSpeedRPM * 0.1);
    leftSpeedControl.setReference(rpm, ControlType.kVelocity);
    rightSpeedControl.setReference(-rightShooterRPM, ControlType.kVelocity);
    //shooterRightMotor.follow(shooterLeftMotor, ShooterMechConstants.rightFlywheelInverted);
  }

  public void setIndexRollerSpeed(double speed){
    indexMotor.set(speed);
  }

  public void configureShooterMotors(){
    //Configuring flywheel motors
      shooterLeftMotor.setSmartCurrentLimit(ShooterMechConstants.flywheelCurrentLimit);//Units are in amps
      shooterLeftMotor.setIdleMode(IdleMode.kBrake);

      shooterRightMotor.setSmartCurrentLimit(ShooterMechConstants.flywheelCurrentLimit);//Units are in amps
      shooterRightMotor.setIdleMode(IdleMode.kBrake);
    
    //Setting Up Velocity Control
      leftSpeedControl.setP(ShooterMechConstants.speedP);
      leftSpeedControl.setI(ShooterMechConstants.speedI);
      leftSpeedControl.setD(ShooterMechConstants.speedD);
      leftSpeedControl.setFF(ShooterMechConstants.speedFF);
      leftSpeedControl.setFeedbackDevice(leftShooterSpeedRelEnc);

      rightSpeedControl.setP(ShooterMechConstants.speedP);
      rightSpeedControl.setI(ShooterMechConstants.speedI);
      rightSpeedControl.setD(ShooterMechConstants.speedD);
      rightSpeedControl.setFF(ShooterMechConstants.speedFF);
      rightSpeedControl.setFeedbackDevice(rightShooterSpeedRelEnc);
  }
  
  public void configureShooterPivotMotor(){
    //Converting Absolute Encoder to degrees
      shooterPivotAbsEnc.setPositionConversionFactor(360);

    //Inverting the absolute encoder to get the correct readings
      shooterPivotAbsEnc.setInverted(true);

    //Configuring pivot motor
      shooterPivotMotor.setSmartCurrentLimit(ShooterMechConstants.pivotCurrentLimit);//Units are in amps
      shooterPivotMotor.setIdleMode(IdleMode.kBrake);

    //Configure Pivot PID Controller
      // Enable PID wrap around for the turning motor. This will allow the PID
      // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
      // to 10 degrees will go through 0 rather than the other direction which is a
      // longer route.
        pivotControl.setPositionPIDWrappingEnabled(true);
        pivotControl.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        pivotControl.setPositionPIDWrappingMaxInput(360);
      pivotControl.setP(ShooterMechConstants.pivotP);
      pivotControl.setI(ShooterMechConstants.pivotI);
      pivotControl.setD(ShooterMechConstants.pivotD);
      pivotControl.setFF(ShooterMechConstants.pivotFF);
      pivotControl.setFeedbackDevice(shooterPivotAbsEnc);
      pivotControl.setOutputRange(-0.25,
      0.3);
  }

  public void configureIndexMotor(){
    //Configuring index motor
      indexMotor.setSmartCurrentLimit(ShooterMechConstants.indexCurrentLimit);//Unit is in amps
      indexMotor.setIdleMode(IdleMode.kCoast);
  }
}
