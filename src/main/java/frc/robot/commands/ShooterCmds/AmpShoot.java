// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterMechConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShoot extends Command {
  private ShooterSubsystem shooterSub;

  private IntakeSubsystem intakeSub;

  private boolean runIndex;
  /** Creates a new AmpShoot. */
  public AmpShoot(ShooterSubsystem shooterSub, IntakeSubsystem intakeSub, boolean runIndex) {
    this.shooterSub = shooterSub;
    this.intakeSub = intakeSub;
    this.runIndex = runIndex;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSub.setPivotAngle(ShooterMechConstants.ampPos);
    if(Math.abs(ShooterMechConstants.ampPos - shooterSub.getPivotAngle()) <=5 ){
      shooterSub.setFlyWheelSpeeds(ShooterMechConstants.flywheelAmpShootSpeed);
    }
    if(runIndex){
      shooterSub.setIndexRollerSpeed(ShooterMechConstants.indexShootSpeed);
    }
    else{
      shooterSub.setIndexRollerSpeed(0);
    }
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
