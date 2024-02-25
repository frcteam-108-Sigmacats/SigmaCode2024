// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDs;

public class SetLEDS extends Command {
  private LEDs ledSub;

  private IntakeSubsystem intakeSub;
  /** Creates a new SetLEDS. */
  public SetLEDS(LEDs ledSub, IntakeSubsystem intakeSub) {
    this.ledSub = ledSub;

    this.intakeSub = intakeSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ledSub.setLEDColor(0.2);
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
