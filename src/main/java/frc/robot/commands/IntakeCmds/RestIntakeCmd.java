// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RestIntakeCmd extends Command {
  //Creates the private subsystem that this command will be using
  private IntakeSubsystem intakeSubsystem;

  private CommandXboxController driveController;

  private int counter;
  /** Creates a new RestIntakeCmd. */
  public RestIntakeCmd(IntakeSubsystem intakeSubsystem, CommandXboxController driverController) {
    //Assigning the private variable in class to the parameters variable
    this.intakeSubsystem = intakeSubsystem;

    this.driveController = driverController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Keeps the intake at rest position running the rollers at 0%
    // if(counter <= 25){
    //   counter++;
    // }
    // else{
    //   driveController.getHID().setRumble(RumbleType.kBothRumble, 0);
    // }
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
