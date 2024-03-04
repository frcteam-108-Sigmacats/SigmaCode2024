// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetServoSpeed extends Command {
  private ElevatorSubsystem elevatorSub;

  private double speed;
  /** Creates a new SetServoSpeed. */
  public SetServoSpeed(ElevatorSubsystem elevatorSub, double speed) {
    this.elevatorSub = elevatorSub;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSub.setServoSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSub.setServoSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
