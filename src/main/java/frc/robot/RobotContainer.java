// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveJoystick;
import frc.robot.commands.IntakeANDTransferCmd;
import frc.robot.commands.OuttakeANDTransferCmd;
import frc.robot.commands.StopTransferANDIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ShooterMechConstants;
import frc.robot.commands.IntakeCmd.IntakeRollersRest;
import frc.robot.commands.IntakeCmd.RestIntakeCmd;
import frc.robot.commands.IntakeCmd.RunIntakeANDTransferCmd;
import frc.robot.commands.IntakeCmd.RunOuttakeANDReverseTransferCmd;
import frc.robot.commands.IntakeCmd.TestIntakePivot;
import frc.robot.commands.ShooterCmd.AmpShoot;
import frc.robot.commands.ShooterCmd.RestShooter;
import frc.robot.commands.ShooterCmd.SetAngleAndFlywheelSpeeds;
import frc.robot.commands.ShooterCmd.SetFlyWheelSpeeds;
import frc.robot.commands.ShooterCmd.SetIndexRollerSpeeds;
import frc.robot.commands.ShooterCmd.ShooterTransfer;
import frc.robot.subsystems.ShooterSubsystem;

import java.sql.DriverAction;

import frc.robot.commands.Autos;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final ShooterSubsystem shooterSub = new ShooterSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem m_exampleSubsystem = new ElevatorSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveController = new 
  CommandXboxController(OperatorConstants.kDriverControllerPort);

  //Instantiating the controllers buttons
  private Trigger dRTrigger, dLTrigger, dLBumper, dRBumper;

  private Trigger kA, kB, dPadUp, dPadDown;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    driveSubsystem.setDefaultCommand(new DriveJoystick(driveSubsystem, driveController, fieldRelative));
    //Intake does not move from rest position
    intakeSubsystem.setDefaultCommand(new RestIntakeCmd(intakeSubsystem));
    shooterSub.setDefaultCommand(new RestShooter(shooterSub));

    // Configure the trigger bindings
    configureBindings();
    dRTrigger.whileTrue(new IntakeANDTransferCmd(intakeSubsystem, shooterSub));
    dRTrigger.whileFalse(new StopTransferANDIntake(intakeSubsystem, shooterSub));
    dLTrigger.whileTrue(new SetAngleAndFlywheelSpeeds(shooterSub, intakeSubsystem, ShooterMechConstants.restPos, ShooterMechConstants.flywheelShootSpeed));
    dLTrigger.whileFalse(new SetAngleAndFlywheelSpeeds(shooterSub, intakeSubsystem, ShooterMechConstants.restPos, 0));
    dLBumper.whileTrue(new OuttakeANDTransferCmd(intakeSubsystem, shooterSub, IntakeConstants.outtakeSpeed, IntakeConstants.reverseTransferSpeed, -ShooterMechConstants.indexTransferSpeed, -ShooterMechConstants.flywheelShootSpeed));
    dLBumper.whileFalse(new StopTransferANDIntake(intakeSubsystem, shooterSub));
    dRBumper.whileTrue(new AmpShoot(shooterSub));
    dRBumper.whileFalse(new RestShooter(shooterSub));
    
    // dPadDown.whileTrue(new TestIntakePivot(intakeSubsystem, -0.5));
    // dPadDown.whileFalse(new TestIntakePivot(intakeSubsystem, 0));
    // dPadUp.whileTrue(new TestIntakePivot(intakeSubsystem, 0.5));
    // dPadUp.whileFalse(new TestIntakePivot(intakeSubsystem, 0));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    dRTrigger = driveController.rightTrigger();
    dLTrigger = driveController.leftTrigger();
    dLBumper = driveController.leftBumper();
    dRBumper = driveController.rightBumper();
    kA = driveController.a();
    kB = driveController.b();
    dPadUp = driveController.povUp();
    dPadDown = driveController.povDown();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
