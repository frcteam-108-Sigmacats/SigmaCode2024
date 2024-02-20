// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCmd;

<<<<<<<< HEAD:src/main/java/frc/robot/commands/IntakeCmd/RunTransfer.java
========
import frc.robot.subsystems.ElevatorSubsystem;
>>>>>>>> Elevator:src/main/java/frc/robot/commands/SetElevatorPosition.java
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

<<<<<<<< HEAD:src/main/java/frc/robot/commands/IntakeCmd/RunTransfer.java
public class RunTransfer extends Command {
  private IntakeSubsystem intakeSub;
  private double speed;
  /** Creates a new RunTransfer. */
  public RunTransfer(IntakeSubsystem intakeSub, double speed) {
    this.intakeSub = intakeSub;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSub);
========
/** An example command that uses an example subsystem. */
public class SetElevatorPosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem elevatorSub;

  private double position;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetElevatorPosition(ElevatorSubsystem elevatorSub, double position) {
    this.elevatorSub = elevatorSub;
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSub);
>>>>>>>> Elevator:src/main/java/frc/robot/commands/SetElevatorPosition.java
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/IntakeCmd/RunTransfer.java
    intakeSub.setTransferSpeed(speed);
========
    elevatorSub.setElevatorPosition(position);
>>>>>>>> Elevator:src/main/java/frc/robot/commands/SetElevatorPosition.java
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
