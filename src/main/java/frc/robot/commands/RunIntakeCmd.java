// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunIntakeCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem intakeSubsystem;
  private final double intakePos;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIntakeCmd(IntakeSubsystem intakeSubsystem, double intakePos) {
    this.intakeSubsystem = intakeSubsystem;
    this.intakePos = intakePos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if there is no note in th intake AND in the shooter (add later)
    // if(!intakeSubsystem.getIRSensor()){
    //   intakeSubsystem.setIntakeAngle(intakePos);
    //   intakeSubsystem.setIntakeSpeed(IntakeConstants.intakeSpeed);
    // }
    // else{
    //   intakeSubsystem.setIntakeAngle(IntakeConstants.restPos);
    //   intakeSubsystem.setIntakeSpeed(0);
    // }
    intakeSubsystem.setIntakeAngle(intakePos);
    intakeSubsystem.setIntakeSpeed(IntakeConstants.intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(intakeSubsystem.getIRSensor()){
    //   return true;
    // }
    return false;
  }
}
