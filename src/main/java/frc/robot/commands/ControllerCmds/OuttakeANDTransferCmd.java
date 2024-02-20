// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControllerCmds;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IntakeCmds.RunOuttakeANDReverseTransferCmd;
import frc.robot.commands.ShooterCmds.ReverseShooterTransfer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OuttakeANDTransferCmd extends ParallelCommandGroup {
  /** Creates a new OuttakeANDTransferCmd. */
  public OuttakeANDTransferCmd(IntakeSubsystem intakeSub, ShooterSubsystem shooterSub, double outtakeSpeed, double reverseTransferSpeed, double indexSpeed, double flywheelSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunOuttakeANDReverseTransferCmd(intakeSub, outtakeSpeed, reverseTransferSpeed), new ReverseShooterTransfer(shooterSub, intakeSub, indexSpeed, flywheelSpeed));
  }
}
