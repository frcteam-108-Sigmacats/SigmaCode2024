// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.ShooterSubsystem;

public class SetLEDS extends Command {
  private LEDs ledSub;

  private IntakeSubsystem intakeSub;

  private ShooterSubsystem shooterSub;
  /** Creates a new SetLEDS. */
  public SetLEDS(LEDs ledSub, IntakeSubsystem intakeSub, ShooterSubsystem shooterSub) {
    this.ledSub = ledSub;

    this.intakeSub = intakeSub;

    this.shooterSub = shooterSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeSub.getIRSensor()){
      if(shooterSub.getFlywheelVelocity() <= -5900){
        ledSub.setLEDColor(0.07);
      }
      else{
        ledSub.setLEDColor(0.03);//original speed is 0.07 for slow blue pulse
      }
    }
    else{
      if(intakeSub.isAbsEncConnected() || shooterSub.isAbsEncConnected()){
        ledSub.setLEDColor(-0.85);
      }
      else{
        ledSub.setLEDColor(-0.25);
      }
    }
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
