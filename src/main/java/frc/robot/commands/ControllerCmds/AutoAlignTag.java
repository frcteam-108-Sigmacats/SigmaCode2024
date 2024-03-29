// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControllerCmds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Vision;

public class AutoAlignTag extends Command {
  private DriveSubsystem driveSub;

  private Vision visionSub;

  private IntakeSubsystem intakeSub;

  private CommandXboxController driveController;

  private PIDController alignPID = new PIDController(0.02, 0, 0);

  private double xAxis, yAxis, rotation, offset;

  private Translation2d translation = new Translation2d();

  private boolean fieldRelative, shooterAlign;
  /** Creates a new AutoAlignTag. */
  public AutoAlignTag(DriveSubsystem driveSub, Vision visionSub, IntakeSubsystem intakeSub, CommandXboxController driveController, boolean fieldRelative, boolean shooterAlign) {
    this.driveSub = driveSub;
    this.visionSub = visionSub;
    this.intakeSub = intakeSub;

    this.driveController = driveController;

    this.fieldRelative = fieldRelative;
    this.shooterAlign = shooterAlign;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xAxis = -driveController.getLeftX();
    yAxis = -driveController.getLeftY();
    double currentDist = visionSub.getSpeakerDistanceFromRobot();

    if(currentDist <= 90){
      offset = 0.5;
    }
    else{
      offset = 1.0;
    }
    if(visionSub.isThereTag() && shooterAlign){
      if(visionSub.doesItSeeSpeaker()){
        rotation = alignPID.calculate(visionSub.getAprilTagXOffset(), offset);
        System.out.println("PID:" + rotation);
      }
      else{
        rotation = (Math.abs(-driveController.getRightX()) < ChassisConstants.deadband ? 0 : (-driveController.getRightX() * 10));
      }
    }
    else{
      rotation = (Math.abs(-driveController.getRightX()) < ChassisConstants.deadband ? 0 : (-driveController.getRightX() * 10));
    }
    yAxis = (Math.abs(yAxis) < ChassisConstants.deadband ? 0 : yAxis * 0.3);
    xAxis = (Math.abs(xAxis) < ChassisConstants.deadband ? 0 : xAxis * 0.3);

    translation = new Translation2d(yAxis, xAxis).times(ChassisConstants.kMaxSpeedMPS);
    driveSub.drive(translation, rotation * 2, fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intakeSub.getIRSensor();
  }
}
