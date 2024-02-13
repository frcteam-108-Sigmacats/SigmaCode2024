// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterMechConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTransfer extends Command {
  private ShooterSubsystem shooterSub;
  private double indexSpeed, flywheelSpeed;
  /** Creates a new ShooterTransfer. */
  public ShooterTransfer(ShooterSubsystem shooterSub, double indexSpeed, double flywheelSpeed) {
    this.shooterSub = shooterSub;
    this.indexSpeed = indexSpeed;
    this.flywheelSpeed = flywheelSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSub.setPivotAngle(ShooterMechConstants.restPos);
    shooterSub.setIndexRollerSpeed(indexSpeed);
    shooterSub.setFlyWheelSpeeds(flywheelSpeed);
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
