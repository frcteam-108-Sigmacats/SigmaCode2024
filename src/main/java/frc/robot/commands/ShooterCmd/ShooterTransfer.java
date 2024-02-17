// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCmd;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ShooterMechConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTransfer extends Command {
  private ShooterSubsystem shooterSub;
  private IntakeSubsystem intakeSub;
  private double indexSpeed, flywheelSpeed;
  private int counter;
  private boolean finish, isThereGamePiece;
  /** Creates a new ShooterTransfer. */
  public ShooterTransfer(ShooterSubsystem shooterSub, IntakeSubsystem intakeSub, double indexSpeed, double flywheelSpeed) {
    this.shooterSub = shooterSub;
    this.indexSpeed = indexSpeed;
    this.flywheelSpeed = flywheelSpeed;
    this.intakeSub = intakeSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isThereGamePiece = false;
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSub.setPivotAngle(ShooterMechConstants.restPos);
    shooterSub.setFlyWheelSpeeds(flywheelSpeed);
    shooterSub.setIndexRollerSpeed(indexSpeed);
    // if(ChassisConstants.isThereGamePiece == true && intakeSub.getIRSensor() == false){
    //   shooterSub.setIndexRollerSpeed(-indexSpeed);
    // }
    if(isThereGamePiece == false){
      isThereGamePiece = intakeSub.getIRSensor();
    }
    else{
      counter++;
    }
    // if(counter >= 8){
    //   finish = true;
    // }
    if(intakeSub.getIRSensor() == true){
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
