// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControllerCmds;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterMechConstants;
import frc.robot.commands.IntakeCmds.RestIntakeCmd;
import frc.robot.commands.IntakeCmds.RunIntakeANDTransferCmd;
import frc.robot.commands.ShooterCmds.RestShooter;
import frc.robot.commands.ShooterCmds.ShooterTransfer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopTransferANDIntake extends ParallelCommandGroup {
  /** Creates a new IntakeANDTransferCmd. */
  public StopTransferANDIntake(IntakeSubsystem intakeSub, ShooterSubsystem shooterSub, CommandXboxController driveController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RestIntakeCmd(intakeSub, driveController), new RestShooter(shooterSub, intakeSub));
  }
}
