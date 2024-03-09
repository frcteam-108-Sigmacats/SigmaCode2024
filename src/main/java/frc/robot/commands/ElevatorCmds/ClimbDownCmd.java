// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCmds;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterMechConstants;
import frc.robot.commands.ShooterCmds.SetPivotAngle;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbDownCmd extends ParallelCommandGroup {
  /** Creates a new ClimbDownCmd. */
  public ClimbDownCmd(ElevatorSubsystem elevatorSub, ShooterSubsystem shooterSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetPivotAngle(shooterSub, ShooterMechConstants.climbPos), 
    new SetElevatorPosition(elevatorSub, ElevatorConstants.climbDownPos));
  }
}
