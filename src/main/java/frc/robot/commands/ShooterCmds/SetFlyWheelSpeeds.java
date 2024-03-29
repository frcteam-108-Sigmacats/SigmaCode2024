// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ChassisConstants;
import frc.robot.Constants.ShooterMechConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SetFlyWheelSpeeds extends Command {
  private ShooterSubsystem shooterSub;
  private IntakeSubsystem intakeSub;

  private double flywheelSpeed, indexSpeed;

  private int counter;

  private boolean finish;
  /** Creates a new SetFlyWheelSpeeds. */
  public SetFlyWheelSpeeds(ShooterSubsystem shooterSub, IntakeSubsystem intakeSub, double flywheelSpeed, double indexSpeed) {
    this.shooterSub = shooterSub;
    this.intakeSub = intakeSub;

    this.flywheelSpeed = flywheelSpeed;
    this.indexSpeed = indexSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    shooterSub.setFlyWheelSpeeds(flywheelSpeed);
    //System.out.println("Shooter running");

    if(counter >= 50){
      shooterSub.setIndexRollerSpeed(indexSpeed);
    }
    else{
      shooterSub.setIndexRollerSpeed(0);
      // System.out.println("Indexers at 0");
    }
    if(!intakeSub.getIRSensor()){
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shooterSub.setFlyWheelSpeeds(0);
    // shooterSub.setIndexRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
