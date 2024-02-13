// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ModuleConstants;

public class DriveSubsystem extends SubsystemBase {
  //Make the 4 modules
    private final MAXSwerveModule frontLeft = new MAXSwerveModule(ModuleConstants.fLeftDriveMotorID,
    ModuleConstants.fLeftTurnMotorID, ModuleConstants.fLeftAngOffset);

    private final MAXSwerveModule frontRight = new MAXSwerveModule(ModuleConstants.fRightDriveMotorID, 
      ModuleConstants.fRightTurnMotorID, ModuleConstants.fRightAngOffset);

    private final MAXSwerveModule backLeft = new MAXSwerveModule(ModuleConstants.bLeftDriveMotorID, 
      ModuleConstants.bLeftTurnMotorID, ModuleConstants.bLeftAngOffset);

    private final MAXSwerveModule backRight = new MAXSwerveModule(ModuleConstants.bRightDriveMotorID,
    ModuleConstants.bRightTurnMotorID, ModuleConstants.bRightAngOffset);

  //Creating the gyroscope that tracks the robots angles in all axis's
    private final Pigeon2 gyro = new Pigeon2(1);

  private SlewRateLimiter driveLimit = new SlewRateLimiter(5);
  private SlewRateLimiter turnLimit = new SlewRateLimiter(5);
  private SlewRateLimiter rotLimit = new SlewRateLimiter(0.5);

  //Creating an array of the modules made
    private MAXSwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};

  //Creating an odometry to measure robots position on the field
    private SwerveDriveOdometry odometry;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //Setting the gyro to factory default
      gyro.getConfigurator().apply(new Pigeon2Configuration());
      gyro.clearStickyFaults();

    //Resets the gyro
      zeroHeading();

    //Setting the odometry to current position of the robot on the field
      odometry = new SwerveDriveOdometry(ChassisConstants.swerveKinematics, getHeading(), getModulePosition());
  }

  //Gets the angle of the robots direction
    public Rotation2d getHeading(){
      return Rotation2d.fromDegrees(-Math.IEEEremainder(gyro.getAngle(), 360));
    }

  //Gets the position of the robot
    public Pose2d getPose(){
      return odometry.getPoseMeters();
    }

  //Resets the gyro's angle to 0
    public void zeroHeading(){
      gyro.reset();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Updates the robots position
      odometry.update(getHeading(), getModulePosition());

    //Logging the Modules states and the robots state
      SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
      SmartDashboard.putNumber("Robot Degree: ", gyro.getAngle());
      SmartDashboard.putNumber("Robot Pose X: ", getPose().getX());
      SmartDashboard.putNumber("Robot Pose Y: ", getPose().getY());
      SmartDashboard.putNumber("FrontL Angle: ", frontLeft.getState().angle.getDegrees());
      SmartDashboard.putNumber("FrontR Angle: ", frontRight.getState().angle.getDegrees());
      SmartDashboard.putNumber("BackL Angle: ", backLeft.getState().angle.getDegrees());
      SmartDashboard.putNumber("BackR Angle: ", backRight.getState().angle.getDegrees());
  }

  //Sets the speed and direction of each module and adjusts based on if we want to drive field relative or not
    public void drive(Translation2d translation, double rotation, boolean fieldRelative){
      SwerveModuleState[] swerveModuleStates =
          ChassisConstants.swerveKinematics.toSwerveModuleStates(
              fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                  driveLimit.calculate(translation.getX()), 
                                  turnLimit.calculate(translation.getY()), 
                                  rotation, 
                                  getYaw()
                              )
                              : new ChassisSpeeds(
                                  driveLimit.calculate(translation.getX()), 
                                  turnLimit.calculate(translation.getY()), 
                                  rotLimit.calculate(rotation))
                              );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ChassisConstants.kMaxSpeedMPS);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

  //Sets all 4 modules desired state (Used mainly for auto)
    public void setModuleStates(SwerveModuleState[] desiredStates){
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ChassisConstants.kMaxSpeedMPS);//Switch to auto speeds later
      frontLeft.setDesiredState(desiredStates[0]);
      frontRight.setDesiredState(desiredStates[1]);
      backLeft.setDesiredState(desiredStates[2]);
      backRight.setDesiredState(desiredStates[3]);
    }

  //Gets the 4 modules current state
    public SwerveModuleState[] getModuleStates(){
      SwerveModuleState[] states = new SwerveModuleState[4];
      for(int i = 0; i < states.length; i++){
        states[i] = modules[i].getState();
      }
      return states;
    }

  //Gets the modules position (How much did the robot drive forward or backwards and left or right in meters and the direction the wheels are facing)
    public SwerveModulePosition[] getModulePosition(){
      SwerveModulePosition[] positions = new SwerveModulePosition[4];
      for(int i = 0; i < positions.length; i++){
        positions[i] = modules[i].getPosition();
      }
      return positions;
    }

  //Resets the robots position (Used for auto)
    public void resetOdometry(Pose2d pose){
      odometry.resetPosition(getHeading(), getModulePosition(), pose);
    }

  //Sets the drive encoders to 0
    public void resetEncoders(){
      frontLeft.resetEncoders();
      frontRight.resetEncoders();
      backLeft.resetEncoders();
      backRight.resetEncoders();
    }


  //Same thing as getting the heading but is not restricted to 360 degrees
    public Rotation2d getYaw(){
      double yaw = gyro.getYaw().getValueAsDouble();
      return (ChassisConstants.gyroReversed) ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
    }

  //Gets the speeds of the modules rather than the state (Speeds for driving, turning, and rotating | State: Position of the module) (Used for Auto)
    public ChassisSpeeds getSpeeds(){
      SwerveModuleState[] states = getModuleStates();
      return ChassisConstants.swerveKinematics.toChassisSpeeds(states);
    }

  //Gets the Field Relative speeds of the modules (Could be used for auto and if not it is not used anywhere else)
    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds){
      driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

  //Gets the robot relative chassis speeds 
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
      ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

      SwerveModuleState[] targetStates = ChassisConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
      setModuleStates(targetStates);
    }
}
