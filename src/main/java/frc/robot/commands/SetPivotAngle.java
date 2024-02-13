// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

<<<<<<<< HEAD:src/main/java/frc/robot/commands/RunOuttakeCmd.java
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunOuttakeCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem intakeSubsystem;
  private final double intakePos;
========
import frc.robot.Constants.ShooterMechConstants;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetPivotAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooterSub;

  private double pivotAngle;
>>>>>>>> Shooter:src/main/java/frc/robot/commands/SetPivotAngle.java

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
<<<<<<<< HEAD:src/main/java/frc/robot/commands/RunOuttakeCmd.java
  public RunOuttakeCmd(IntakeSubsystem intakeSubsystem, double intakePos) {
    this.intakeSubsystem = intakeSubsystem;
    this.intakePos = intakePos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intakeSubsystem);
========
  public SetPivotAngle(ShooterSubsystem shooterSub, double pivotAngle) {
    this.shooterSub = shooterSub;
    this.pivotAngle = pivotAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
>>>>>>>> Shooter:src/main/java/frc/robot/commands/SetPivotAngle.java
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/RunOuttakeCmd.java
    intakeSubsystem.setIntakeAngle(intakePos);
    intakeSubsystem.setIntakeSpeed(IntakeConstants.outtakeSpeed);
========
    shooterSub.setPivotAngle(pivotAngle);
>>>>>>>> Shooter:src/main/java/frc/robot/commands/SetPivotAngle.java
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
<<<<<<<< HEAD:src/main/java/frc/robot/commands/RunOuttakeCmd.java
    //add this after testing each mechanism
    // if(shooterSubsystem.getIRSensor()){ 
    //   return true;
    // }
========
    if(Math.abs(pivotAngle - shooterSub.getPivotAngle()) <= 1){
      return true;
    }
>>>>>>>> Shooter:src/main/java/frc/robot/commands/SetPivotAngle.java
    return false;
  }
}
