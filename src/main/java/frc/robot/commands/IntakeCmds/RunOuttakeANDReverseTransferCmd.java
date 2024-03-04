// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class RunOuttakeANDReverseTransferCmd extends Command {
  //Creating private subsystem that we will be using for this command
  private IntakeSubsystem intakeSub;

  //Creating private variables to set speeds for the motors on the intake mechanism
  private double intakeSpeed, transferSpeed;
  /** Creates a new RunOuttakeANDReverseTransferCmd. */
  public RunOuttakeANDReverseTransferCmd(IntakeSubsystem intakeSub, double intakeSpeed, double transferSpeed) {
    //Assigning the class variable to the variable in the parameter
    this.intakeSub = intakeSub;

    this.intakeSpeed = intakeSpeed;
    this.transferSpeed = transferSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.setIntakeAngle(IntakeConstants.restPos);
    intakeSub.setIntakeSpeed(intakeSpeed);
    intakeSub.setTransferSpeed(transferSpeed);
    // if(intakeSub.getIRSensor() == false){
    //   ChassisConstants.isThereGamePiece = false;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(ChassisConstants.isThereGamePiece == false){
    //   return true;
    // }
    // else{
    //   return false;
    // }
    return false;
  }
}
