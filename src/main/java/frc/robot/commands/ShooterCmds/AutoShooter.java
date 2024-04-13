// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.IntakeConstants;
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
    // for(int i = 0; i < ShooterMechConstants.distSetPoints.length; i++){
    //   if(i != ShooterMechConstants.distSetPoints.length - 1){
    //     if(currentDist >= ShooterMechConstants.distSetPoints[i] && currentDist <= ShooterMechConstants.distSetPoints[i + 1]){
    //       double secondPoint = ShooterMechConstants.distSetPoints[i + 1];
    //       double firstPoint = ShooterMechConstants.distSetPoints[i];
    //       double firstPointAngle = ShooterMechConstants.angleSetPoints[i];
    //       double secondPointAngle = ShooterMechConstants.angleSetPoints[i+1];
    //       pivotAngle = (((currentDist - firstPoint) / (secondPoint - firstPoint)) * (secondPointAngle - firstPointAngle)) + firstPointAngle;
    //       break;
    //     }
    //   }
    //   else{
    //     pivotAngle = ShooterMechConstants.angleSetPoints[ShooterMechConstants.angleSetPoints.length - 1];
    //     break;
    //   }
    // }
    //pivotAngle = (-0.0030042 * Math.pow(currentDist, 2)) + (0.8324295 * currentDist) - 15.1666232;
    //pivotAngle = (-12.764943846962566) + (0.79771905891679129 * currentDist) + (-0.0031590772834475325 * Math.pow(currentDist, 2));
    pivotAngle = (0.00000000000016370943436754830 * Math.pow(currentDist, 8)) - (0.00000000013485474610480835 * Math.pow(currentDist, 7)) +
    (0.000000047319450547384044 * Math.pow(currentDist, 6)) - (0.0000092286980457404328 * Math.pow(currentDist, 5)) +
    (0.0010925631267597823 * Math.pow(currentDist, 4)) - (0.080238408863234717 * Math.pow(currentDist, 3)) +
    (3.5581453006266690 * Math.pow(currentDist, 2)) - (86.320078674990356 * currentDist) + 884.72326745926716;
    // pivotAngle = (-0.0024512509473472699 * Math.pow(currentDist, 2)) + (0.70558030260894622 * currentDist) - 9.6701102318001979;
     
    if(currentDist >= 0 && currentDist <= 71){
      speed = -0.65;
    }
    else if(currentDist > 71 && currentDist <= 97){
      speed = -0.8;
    }
    else if(currentDist > 97 && currentDist <= 109){
      speed = -0.8;
    }
    else if(currentDist > 110 && currentDist <= 116){
      speed = -0.8;
    }
    else{
      speed = -0.8;
    }
    // System.out.println("Auto Shooter is running");
    counter++;
    //shooterSub.setPivotAngle(pivotAngle);
    shooterSub.setPivotAngle(pivotAngle);
    // System.out.println("Shooter Angle is " + pivotAngle);
    //shooterSub.setFlyWheelSpeeds(speed);
    shooterSub.setFlyWheelVelocity(-7000);
    // if(counter >= 50 && Math.abs(pivotAngle - shooterSub.getPivotAngle()) <= 0.5 /*&& pivotAngle - shooterSub.getPivotAngle() >= 0*/){
    //   shooterSub.setIndexRollerSpeed(ShooterMechConstants.indexShootSpeed);
    // }
    if(runIndex){
      shooterSub.setIndexRollerSpeed(ShooterMechConstants.indexShootSpeed);
    }
    else{
      shooterSub.setIndexRollerSpeed(0);
      // System.out.println("Indexers at 0");
    }
    if(!intakeSub.getIRSensor()){
      //System.out.println("Auto Shooter is ending");
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
