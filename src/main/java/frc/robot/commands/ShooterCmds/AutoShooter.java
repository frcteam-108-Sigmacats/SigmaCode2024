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

  private boolean finish;
  /** Creates a new AutoPivotShooter. */
  public AutoShooter(ShooterSubsystem shooterSub, Vision visionSub, IntakeSubsystem intakeSub) {
    this.shooterSub = shooterSub;
    this.visionSub = visionSub;
    this.intakeSub = intakeSub;
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
          pivotAngle = ShooterMechConstants.angleSetPoints[i];
          break;
        }
      }
      else{
        pivotAngle = ShooterMechConstants.angleSetPoints[ShooterMechConstants.angleSetPoints.length - 1];
        break;
      }
    }
    if(currentDist >= 0 && currentDist <= 76){
      speed = -0.55;
    }
    else if(currentDist >= 76 && currentDist <= 97){
      speed = -0.75;
    }
    else if(currentDist >= 97 && currentDist <= 113){
      speed = -0.8;
    }
    else if(currentDist >= 113 && currentDist <= 116){
      speed = -1.0;
    }
    else{
      speed = -1.0;
    }
    counter++;
    shooterSub.setPivotAngle(pivotAngle);
    shooterSub.setFlyWheelSpeeds(speed);
    if(counter >= 50){
      shooterSub.setIndexRollerSpeed(ShooterMechConstants.indexShootSpeed);
    }
    if(!intakeSub.getIRSensor()){
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
