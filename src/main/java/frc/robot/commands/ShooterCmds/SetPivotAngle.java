package frc.robot.commands.ShooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class SetPivotAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem shooterSub;

  private double pivotAngle;

  public SetPivotAngle(ShooterSubsystem shooterSub, double pivotAngle) {
    this.shooterSub = shooterSub;
    this.pivotAngle = pivotAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSub.setPivotAngle(pivotAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //add this after testing each mechanism
    // if(shooterSubsystem.getIRSensor()){ 
    //   return true;
    // }
    if(Math.abs(shooterSub.getSetAngle() - shooterSub.getPivotAngle()) <= 1 || shooterSub.getPivotAngle() >= 350){
      return true;
    }
    return false;
  }
}
