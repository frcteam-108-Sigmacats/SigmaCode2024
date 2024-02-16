// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCmd;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class TestIntakePivot extends Command {
  private IntakeSubsystem intakeSub;

  private double speed;
  /** Creates a new TestIntakePivot. */
  public TestIntakePivot(IntakeSubsystem intakeSub, double speed) {
    this.intakeSub = intakeSub;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.testIntakePivot(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.testIntakePivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
