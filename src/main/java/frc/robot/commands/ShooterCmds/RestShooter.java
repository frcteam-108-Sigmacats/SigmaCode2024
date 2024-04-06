// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterMechConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RestShooter extends Command {
  private ShooterSubsystem shooterSub;
  private IntakeSubsystem intakeSub;
  /** Creates a new RestShooter. */
  public RestShooter(ShooterSubsystem shooterSub, IntakeSubsystem intakeSub) {
    this.shooterSub = shooterSub;
    this.intakeSub = intakeSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooterSub.setFlyWheelSpeeds(-1.0);//switch to -30%
    double rpm = -6784 * 0.2;
    System.out.println("RPM is " + rpm);
    shooterSub.setFlyWheelVelocity(rpm);
    if(intakeSub.getTransferSensor() == true){
      shooterSub.setIndexRollerSpeed(ShooterMechConstants.indexTransferSpeed / 2.5);
    }
    else{
      shooterSub.setIndexRollerSpeed(0);
    }
    shooterSub.setPivotAngle(ShooterMechConstants.restPos);
    // System.out.println("Shooter at rest position");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
