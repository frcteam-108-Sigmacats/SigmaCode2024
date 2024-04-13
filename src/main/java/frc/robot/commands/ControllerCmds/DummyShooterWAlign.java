// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControllerCmds;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeCmds.SetIntakeAngle;
import frc.robot.commands.ShooterCmds.DummyShooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DummyShooterWAlign extends ParallelRaceGroup {
  /** Creates a new DummyShooterWAlign. */
  public DummyShooterWAlign(ShooterSubsystem shooterSub, Vision visionSub, IntakeSubsystem intakeSub, DriveSubsystem driveSub, boolean runIndex, double shooterPos, double shootSpeed, CommandXboxController driverController, boolean fieldRelative, boolean shooterAlign, boolean fastAlign, boolean auto) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoAlignTag(driveSub, visionSub, intakeSub, driverController, fieldRelative, shooterAlign, fastAlign), new DummyShooter(shooterSub, intakeSub, runIndex, shooterPos, shootSpeed, auto), new SetIntakeAngle(intakeSub, IntakeConstants.shootIntakePos, runIndex));
  }
}
