// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCmds.IntakeRollersRest;
import frc.robot.commands.IntakeCmds.RestIntakeCmd;
import frc.robot.commands.IntakeCmds.RunIntakeANDTransferCmd;
import frc.robot.commands.IntakeCmds.RunOuttakeANDReverseTransferCmd;
import frc.robot.commands.IntakeCmds.TestIntakePivot;
import frc.robot.commands.ShooterCmds.AmpShoot;
import frc.robot.commands.ShooterCmds.AutoShooter;
import frc.robot.commands.ShooterCmds.RestShooter;
import frc.robot.commands.ShooterCmds.SetAngleAndFlywheelSpeeds;
import frc.robot.commands.ShooterCmds.SetFlyWheelSpeeds;
import frc.robot.commands.ShooterCmds.SetIndexRollerSpeeds;
import frc.robot.commands.ShooterCmds.ShooterTransfer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ShooterMechConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

import java.sql.DriverAction;

import frc.robot.commands.ControllerCmds.AmpShootWElevator;
import frc.robot.commands.ControllerCmds.AutoAlignNote;
import frc.robot.commands.ControllerCmds.AutoAlignTag;
import frc.robot.commands.ControllerCmds.AutoShooterWithAlign;
import frc.robot.commands.ControllerCmds.DriveJoystick;
import frc.robot.commands.ControllerCmds.IntakeANDTransferCmd;
import frc.robot.commands.ControllerCmds.OuttakeANDTransferCmd;
import frc.robot.commands.ControllerCmds.StopTransferANDIntake;
import frc.robot.commands.ElevatorCmds.SetElevatorPosition;
import frc.robot.commands.ElevatorCmds.SetElevatorSpeed;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem shooterSub = new ShooterSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final Vision visionSub = new Vision();

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
    // dLTrigger.whileTrue(new SetAngleAndFlywheelSpeeds(shooterSub, intakeSubsystem, ShooterMechConstants.restPos, ShooterMechConstants.flywheelShootSpeed, ShooterMechConstants.indexShootSpeed));
    // dLTrigger.whileFalse(new SetAngleAndFlywheelSpeeds(shooterSub, intakeSubsystem, ShooterMechConstants.restPos, 0, 0));
    // dLTrigger.whileTrue(new AutoShooter(shooterSub, visionSub));
    // dLTrigger.whileFalse(new RestShooter(shooterSub));
    dLTrigger.whileTrue(new AutoShooterWithAlign(driveSubsystem, visionSub, shooterSub, intakeSubsystem, driveController, fieldRelative));
    dLTrigger.whileFalse(new RestShooter(shooterSub));
    dLBumper.whileTrue(new OuttakeANDTransferCmd(intakeSubsystem, shooterSub, IntakeConstants.outtakeSpeed, IntakeConstants.reverseTransferSpeed, -ShooterMechConstants.indexTransferSpeed, -ShooterMechConstants.flywheelShootSpeed));
    dLBumper.whileFalse(new StopTransferANDIntake(intakeSubsystem, shooterSub));
    dRBumper.whileTrue(new AmpShootWElevator(elevatorSubsystem, shooterSub));
    dRBumper.whileFalse(new ParallelCommandGroup(new RestShooter(shooterSub), new SetElevatorPosition(elevatorSubsystem, 5)));
    // kA.whileTrue(new AutoAlignNote(driveSubsystem, visionSub, driveController, fieldRelative));
    // kA.whileFalse(new DriveJoystick(driveSubsystem, driveController, fieldRelative));
    kA.onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));
    // kB.whileTrue(new AutoAlignTag(driveSubsystem, visionSub, driveController, fieldRelative));
    // kB.whileFalse(new DriveJoystick(driveSubsystem, driveController, fieldRelative));
    
    dPadDown.whileTrue(new SetElevatorSpeed(elevatorSubsystem, -1.0));
    dPadDown.whileFalse(new SetElevatorSpeed(elevatorSubsystem, 0));
    dPadUp.whileTrue(new SetElevatorSpeed(elevatorSubsystem, 0.3));
    dPadUp.whileFalse(new SetElevatorSpeed(elevatorSubsystem, 0));
    // dPadUp.whileTrue(new SetElevatorPosition(elevatorSubsystem, 58));
    // dPadDown.whileTrue(new SetElevatorPosition(elevatorSubsystem, 5));
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
