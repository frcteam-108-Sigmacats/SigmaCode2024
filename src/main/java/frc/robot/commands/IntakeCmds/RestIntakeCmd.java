// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RestIntakeCmd extends Command {
  //Creates the private subsystem that this command will be using
  private IntakeSubsystem intakeSubsystem;
  /** Creates a new RestIntakeCmd. */
  public RestIntakeCmd(IntakeSubsystem intakeSubsystem) {
    //Assigning the private variable in class to the parameters variable
    this.intakeSubsystem = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Keeps the intake at rest position running the rollers at 0%
    intakeSubsystem.setIntakeAngle(IntakeConstants.restPos);
    intakeSubsystem.setIntakeSpeed(0);
    intakeSubsystem.setTransferSpeed(0);
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