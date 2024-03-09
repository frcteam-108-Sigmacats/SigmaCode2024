// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterMechConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class RestShooter extends Command {
  private ShooterSubsystem shooterSub;
  /** Creates a new RestShooter. */
  public RestShooter(ShooterSubsystem shooterSub) {
    this.shooterSub = shooterSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSub.setFlyWheelSpeeds(-0.3);//switch to -30%
    shooterSub.setIndexRollerSpeed(0);
    shooterSub.setPivotAngle(ShooterMechConstants.restPos);
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
