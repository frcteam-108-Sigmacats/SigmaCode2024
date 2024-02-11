// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SetFlyWheelSpeeds extends Command {
  private ShooterSubsystem shooterSub;

  private double speed;

  private int counter;
  /** Creates a new SetFlyWheelSpeeds. */
  public SetFlyWheelSpeeds(ShooterSubsystem shooterSub, double speed) {
    this.shooterSub = shooterSub;

    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    shooterSub.setFlyWheelSpeeds(speed);
    if(counter >= 150){
      shooterSub.setIndexRollerSpeed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSub.setFlyWheelSpeeds(0);
    shooterSub.setIndexRollerSpeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(counter >= 150){
    //   return true;
    // }
    return false;
  }
}
