// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControllerCmds;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorCmds.SetElevatorPosition;
import frc.robot.commands.ShooterCmds.AmpShoot;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpShootWElevator extends ParallelCommandGroup {
  /** Creates a new AmpShootWElevator. */
  public AmpShootWElevator(ElevatorSubsystem elevatorSub, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetElevatorPosition(elevatorSub, ElevatorConstants.ampPos), new AmpShoot(shooterSubsystem));
  }
}
