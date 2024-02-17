// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCmd;

import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunIntakeANDTransferCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //Creating a private subsystem that we will be using for this command
  private final IntakeSubsystem intakeSubsystem;

  //Creating private speeds that will be used in this command only to set the speeds of the motors on the intake
  private double intakeSpeed, transferSpeed;

  private int counter;

  private boolean isThereGamePiece, finish;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunIntakeANDTransferCmd(IntakeSubsystem intakeSubsystem, double intakeSpeed, double transferSpeed) {
    //Assigning the class vairables to the parameters variables
    this.intakeSubsystem = intakeSubsystem;

    this.intakeSpeed = intakeSpeed;
    this.transferSpeed = transferSpeed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    isThereGamePiece = false;
    finish = false;
  }

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
    if(isThereGamePiece == false){
      if(intakeSubsystem.getIRSensor()){
        isThereGamePiece = true;
      }
    }
    else{
      counter++;
    }

    intakeSubsystem.setIntakeAngle(IntakeConstants.groundIntakePos);
    intakeSubsystem.setIntakeSpeed(intakeSpeed);
    intakeSubsystem.setTransferSpeed(transferSpeed);

    // if(counter >= 8){
    //   finish = true;
    // }
    if(intakeSubsystem.getIRSensor() == true){
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
