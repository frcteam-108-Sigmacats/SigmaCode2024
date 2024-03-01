// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterMechConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

public class AutoShooter extends Command {
  private ShooterSubsystem shooterSub;

  private Vision visionSub;

  private IntakeSubsystem intakeSub;

  private int counter;

  private double pivotAngle = 0;

  private double speed = 0;

  private boolean finish, runIndex;
  /** Creates a new AutoPivotShooter. */
  public AutoShooter(ShooterSubsystem shooterSub, Vision visionSub, IntakeSubsystem intakeSub, boolean runIndex) {
    this.shooterSub = shooterSub;
    this.visionSub = visionSub;
    this.intakeSub = intakeSub;
    this.runIndex = runIndex;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentDist = visionSub.getSpeakerDistanceFromRobot();
    for(int i = 0; i < ShooterMechConstants.distSetPoints.length; i++){
      if(i != ShooterMechConstants.distSetPoints.length - 1){
        if(currentDist >= ShooterMechConstants.distSetPoints[i] && currentDist <= ShooterMechConstants.distSetPoints[i + 1]){
          double secondPoint = ShooterMechConstants.distSetPoints[i + 1];
          double firstPoint = ShooterMechConstants.distSetPoints[i];
          double firstPointAngle = ShooterMechConstants.angleSetPoints[i];
          double secondPointAngle = ShooterMechConstants.angleSetPoints[i+1];
          pivotAngle = (((currentDist - firstPoint) / (secondPoint - firstPoint)) * (secondPointAngle - firstPointAngle)) + firstPointAngle;
          break;
        }
      }
      else{
        pivotAngle = ShooterMechConstants.angleSetPoints[ShooterMechConstants.angleSetPoints.length - 1];
        break;
      }
    }
    if(currentDist >= 0 && currentDist <= 71){
      speed = -0.55;
    }
    else if(currentDist > 71 && currentDist <= 97){
      speed = -0.8;
    }
    else if(currentDist > 97 && currentDist <= 109){
      speed = -0.8;
    }
    else if(currentDist > 110 && currentDist <= 116){
      speed = -1.0;
    }
    else{
      speed = -1.0;
    }
    System.out.println("Auto Shooter is running");
    counter++;
    shooterSub.setPivotAngle(pivotAngle);
    shooterSub.setFlyWheelSpeeds(speed);
    // if(counter >= 50 && Math.abs(pivotAngle - shooterSub.getPivotAngle()) <= 0.5 /*&& pivotAngle - shooterSub.getPivotAngle() >= 0*/){
    //   shooterSub.setIndexRollerSpeed(ShooterMechConstants.indexShootSpeed);
    // }
    if(runIndex){
      shooterSub.setIndexRollerSpeed(ShooterMechConstants.indexShootSpeed);
    }
    else{
      shooterSub.setIndexRollerSpeed(0);
    }
    if(!intakeSub.getIRSensor()){
      System.out.println("Auto Shooter is ending");
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
