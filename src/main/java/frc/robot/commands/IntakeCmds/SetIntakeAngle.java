// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeAngle extends Command {
  private IntakeSubsystem intakeSub;

  private double intakePos;
  /** Creates a new SetIntakeAngle. */
  public SetIntakeAngle(IntakeSubsystem intakeSub, double intakePos) {
    this.intakeSub = intakeSub;
    this.intakePos = intakePos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.setIntakeAngle(intakePos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!intakeSub.getIRSensor()){
      return true;
    }
    else{
      return false;
    }
  }
}
