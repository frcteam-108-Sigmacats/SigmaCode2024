// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControllerCmds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AlignWGyro extends Command {
  private DriveSubsystem driveSub;
  private IntakeSubsystem intakeSub;
  
  private PIDController alignPID = new PIDController(0.028, 0, 0);

  private CommandXboxController driverController;

  private Translation2d translation;

  private double xAxis, yAxis, setpoint;

  private boolean fieldRelative;
  /** Creates a new AlignWGyro. */
  public AlignWGyro(DriveSubsystem driveSub, IntakeSubsystem intakeSub, CommandXboxController driverController, boolean fieldRelative) {
    this.driveSub = driveSub;
    this.intakeSub = intakeSub;

    this.driverController = driverController;

    this.fieldRelative = fieldRelative;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.getAlliance().get() == Alliance.Blue){
      setpoint = 15;
    }
    else{
      setpoint = -15;
    }
    double rotation = alignPID.calculate(driveSub.getHeading().getDegrees(), setpoint);

    xAxis = -driverController.getLeftX();
    yAxis = -driverController.getLeftY();

    yAxis = (Math.abs(yAxis) < ChassisConstants.deadband ? 0 : yAxis * 0.3);
    xAxis = (Math.abs(xAxis) < ChassisConstants.deadband ? 0 : xAxis * 0.3);

    translation = new Translation2d(yAxis, xAxis).times(ChassisConstants.kMaxSpeedMPS);

    driveSub.drive(translation, rotation, fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intakeSub.getIRSensor();
  }
}
