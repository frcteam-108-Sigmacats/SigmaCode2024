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
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule extends SubsystemBase {
  //Motor Initialization
  private final CANSparkFlex driveMotor;
  private final CANSparkMax turnMotor;

  //Encoder Initialization
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  //PID Controller Initialization
  private final SparkPIDController drivePIDControl;
  private final SparkPIDController turnPIDControl;

  //Angle Offset
  private double angleOffset = 0;

  //Desired SwerveModule State
  private SwerveModuleState desiredState = new SwerveModuleState();
  /** Creates a new ExampleSubsystem. */
  public MAXSwerveModule(int driveMotorID, int turnMotorID, double moduleAngleOffset) {
    //Setting up motors with IDs
    driveMotor = new CANSparkFlex(driveMotorID, MotorType.kBrushless);
    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

    //Resetting to factory default to configure
    driveMotor.restoreFactoryDefaults();
    turnMotor.restoreFactoryDefaults();

    //Setting up encoders and PID Controllers
    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    drivePIDControl = driveMotor.getPIDController();
    turnPIDControl = turnMotor.getPIDController();
    drivePIDControl.setFeedbackDevice(driveEncoder);
    turnPIDControl.setFeedbackDevice(turnEncoder);

    //Applying Motor Conversions

    //Setting inversion for Absolute Encoder
    turnEncoder.setInverted(ModuleConstants.turningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turnPIDControl.setPositionPIDWrappingEnabled(true);
    turnPIDControl.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    turnPIDControl.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    //Setting PID gains for Drive Motors and Turn Motors for accurate drive and turning
    drivePIDControl.setP(ModuleConstants.kDrivingP);
    drivePIDControl.setI(ModuleConstants.kDrivingI);
    drivePIDControl.setD(ModuleConstants.kDrivingD);
    drivePIDControl.setFF(ModuleConstants.kDrivingFF);
    drivePIDControl.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);
    
    turnPIDControl.setP(ModuleConstants.kTurningP);
    turnPIDControl.setI(ModuleConstants.kTurningI);
    turnPIDControl.setD(ModuleConstants.kTurningD);
    turnPIDControl.setFF(ModuleConstants.kTurningFF);
    turnPIDControl.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    //Setting up Idle mode and Current Limit. Needed to not overpower the motors and burn it
    driveMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setIdleMode(IdleMode.kCoast);
    driveMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    turnMotor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    //Burn flash to save configurations of the Speed Controllers
    driveMotor.burnFlash();
    turnMotor.burnFlash();

    angleOffset = moduleAngleOffset;
    desiredState.angle = Rotation2d.fromRadians(turnEncoder.getPosition());
    driveEncoder.setPosition(0);
  }


  //Gets the speed and angle the wheel is at the current moment
  public SwerveModuleState getState(){
    return new SwerveModuleState(driveEncoder.getVelocity(), 
    new Rotation2d(turnEncoder.getPosition() - angleOffset));
  }

  //Gets the position of the module
  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(), 
    new Rotation2d(turnEncoder.getPosition() - angleOffset));
  }

  //Sets the desired speed and angle of the wheel we want it to go
  public void setDesiredState(SwerveModuleState m_desiredState){
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = m_desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = m_desiredState.angle.plus(Rotation2d.fromRadians(angleOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(turnEncoder.getPosition()));

    //Sets the motor to the desired state
    drivePIDControl.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    turnPIDControl.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

    desiredState = m_desiredState;
  }
  //Resets drive encoder position to 0
  public void resetEncoders(){
    driveEncoder.setPosition(0);
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
