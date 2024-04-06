// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCmds.IntakeRollersRest;
import frc.robot.commands.IntakeCmds.RestIntakeCmd;
import frc.robot.commands.IntakeCmds.RunIntakeANDTransferCmd;
import frc.robot.commands.IntakeCmds.RunOuttakeANDReverseTransferCmd;
import frc.robot.commands.IntakeCmds.TestIntakePivot;
import frc.robot.commands.LEDs.SetLEDS;
import frc.robot.commands.ShooterCmds.AmpShoot;
import frc.robot.commands.ShooterCmds.AutoShooter;
import frc.robot.commands.ShooterCmds.AutonomousAutoShooterWAlign;
import frc.robot.commands.ShooterCmds.DummyShooter;
import frc.robot.commands.ShooterCmds.RestShooter;
import frc.robot.commands.ShooterCmds.ReverseShooterTransfer;
import frc.robot.commands.ShooterCmds.SetAngleAndFlywheelSpeeds;
import frc.robot.commands.ShooterCmds.SetFlyWheelSpeeds;
import frc.robot.commands.ShooterCmds.SetIndexRollerSpeeds;
import frc.robot.commands.ShooterCmds.SetPivotAngle;
import frc.robot.commands.ShooterCmds.SetShooterPivotSpeed;
import frc.robot.commands.ShooterCmds.ShooterTransfer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.Constants.ShooterMechConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

import java.sql.Driver;
import java.sql.DriverAction;
import java.time.Instant;

import org.opencv.features2d.ORB;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.commands.ControllerCmds.AmpShootWElevator;
import frc.robot.commands.ControllerCmds.AutoAlignNote;
import frc.robot.commands.ControllerCmds.AutoAlignTag;
import frc.robot.commands.ControllerCmds.AutoShooterWithAlign;
import frc.robot.commands.ControllerCmds.DriveJoystick;
import frc.robot.commands.ControllerCmds.DummyShooterWAlign;
import frc.robot.commands.ControllerCmds.IntakeANDTransferANDAlignCmd;
import frc.robot.commands.ControllerCmds.IntakeANDTransferCmd;
import frc.robot.commands.ControllerCmds.OuttakeANDTransferCmd;
import frc.robot.commands.ControllerCmds.StopTransferANDIntake;
import frc.robot.commands.ElevatorCmds.ClimbDownCmd;
import frc.robot.commands.ElevatorCmds.ClimbingUpCmd;
import frc.robot.commands.ElevatorCmds.KeepClimbCmd;
import frc.robot.commands.ElevatorCmds.SetElevatorPosition;
import frc.robot.commands.ElevatorCmds.SetElevatorSpeed;
import frc.robot.commands.ElevatorCmds.SetServoSpeed;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final LEDs ledSubsystem = new LEDs();

  private SendableChooser<Command> chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveController = new 
  CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController operatorController = new 
  CommandXboxController(OperatorConstants.kOperatorControllerPort);

  //Instantiating the controllers buttons for driver
  private Trigger dRTrigger, dLTrigger, dLBumper, dRBumper;

  private Trigger dBA, dBB, dBY, dBX, dPadUp, dPadDown;

  //Instantiating the controller buttons for operator
  private Trigger oLTrigger, oRTrigger, oLBumper, oRBumper;

  private Trigger oBA, oBY, oBX, oPadUp, oPadDown, oPadLeft, oPadRight;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;

    //When no commands are running the driver will drive the robot
    driveSubsystem.setDefaultCommand(new DriveJoystick(driveSubsystem, driveController, fieldRelative));

    //Intake does not move from rest position
    intakeSubsystem.setDefaultCommand(new RestIntakeCmd(intakeSubsystem, driveController));

    //When no commands are running the shooter will stay at rest position with a constant speed at -40%
    shooterSub.setDefaultCommand(new RestShooter(shooterSub, intakeSubsystem));

    //LEDs will change based on whether or not we have a note in the robot
    ledSubsystem.setDefaultCommand(new SetLEDS(ledSubsystem, intakeSubsystem, shooterSub));

    //Elevator will stay at rest position when no commands are running
    elevatorSubsystem.setDefaultCommand(new SetElevatorPosition(elevatorSubsystem, 0));

    // Configure the trigger bindings
    configureBindings();

    //Making the autonomous on robot start up
    makeAuto();

    //Driver Commands
      //Driver Trigger Commands
        dRTrigger.whileTrue(new AutoShooterWithAlign(driveSubsystem, visionSub, shooterSub, intakeSubsystem, driveController, fieldRelative, false, true));
        // dRTrigger.whileFalse(new RestShooter(shooterSub));
        dRTrigger.whileFalse(new AutoShooterWithAlign(driveSubsystem, visionSub, shooterSub, intakeSubsystem, driveController, fieldRelative, true, true));

        dLTrigger.whileTrue(new IntakeANDTransferANDAlignCmd(intakeSubsystem, shooterSub, driveSubsystem, visionSub, driveController, fieldRelative));
        dLTrigger.whileFalse(new RestIntakeCmd(intakeSubsystem, driveController));
      
      //Driver Bumper Commands
        dLBumper.whileTrue(new OuttakeANDTransferCmd(intakeSubsystem, shooterSub, IntakeConstants.outtakeSpeed, IntakeConstants.reverseTransferSpeed, ShooterMechConstants.indexOuttakeSpeed, ShooterMechConstants.flywheelOuttakeSpeed));
        dLBumper.whileFalse(new StopTransferANDIntake(intakeSubsystem, shooterSub, driveController));

        // dRBumper.whileTrue(new AmpShootWElevator(elevatorSubsystem, shooterSub));
        // dRBumper.whileFalse(new RestShooter(shooterSub));
        dRBumper.whileTrue(new AmpShootWElevator(elevatorSubsystem, shooterSub, intakeSubsystem, false));
        dRBumper.whileFalse(new AmpShootWElevator(elevatorSubsystem, shooterSub, intakeSubsystem, true));

      //Driver Button Commands
        dBA.onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));

        // dBB.whileTrue(new AutoAlignTag(driveSubsystem, visionSub, intakeSubsystem, driveController, fieldRelative, false));
        // dBB.whileFalse(new DriveJoystick(driveSubsystem, driveController, fieldRelative));

        dBX.whileTrue(new DummyShooterWAlign(shooterSub, visionSub, intakeSubsystem, driveSubsystem, false, ShooterMechConstants.subwooferPos, -0.8, driveController, fieldRelative, true));
        dBX.whileFalse(new DummyShooterWAlign(shooterSub, visionSub, intakeSubsystem, driveSubsystem, true, ShooterMechConstants.subwooferPos, -0.8, driveController, fieldRelative, true));

        dBY.whileTrue(new DummyShooterWAlign(shooterSub, visionSub, intakeSubsystem, driveSubsystem, false, ShooterMechConstants.midPos, -0.65, driveController, fieldRelative, true));
        dBY.whileFalse(new DummyShooterWAlign(shooterSub, visionSub, intakeSubsystem, driveSubsystem, true, ShooterMechConstants.midPos, -0.65, driveController, fieldRelative, true));
        
        dBB.whileTrue(new DummyShooterWAlign(shooterSub, visionSub, intakeSubsystem, driveSubsystem, false, ShooterMechConstants.podiumPos, -0.8, driveController, fieldRelative, true));
        dBB.whileFalse(new DummyShooterWAlign(shooterSub, visionSub, intakeSubsystem, driveSubsystem, true, ShooterMechConstants.podiumPos, -0.8, driveController, fieldRelative, true));

      //Operator Commands
        //Operator Trigger Commands
          oLTrigger.whileTrue(new SetFlyWheelSpeeds(shooterSub, intakeSubsystem, ShooterMechConstants.flywheelShootSpeed, ShooterMechConstants.indexShootSpeed));
          oLTrigger.whileFalse(new RestShooter(shooterSub, intakeSubsystem));

          oRTrigger.whileTrue(new ReverseShooterTransfer(shooterSub, intakeSubsystem, ShooterMechConstants.indexOuttakeSpeed, ShooterMechConstants.flywheelOuttakeSpeed));
          oRTrigger.whileFalse(new RestShooter(shooterSub, intakeSubsystem));

        //Operator Bumper Commands
          oLBumper.whileTrue(new ClimbingUpCmd(elevatorSubsystem, shooterSub, ElevatorConstants.climbUpPos));
          oLBumper.whileFalse(new KeepClimbCmd(elevatorSubsystem, shooterSub, ElevatorConstants.climbUpPos));
          
          oRBumper.whileTrue(new ClimbDownCmd(elevatorSubsystem, shooterSub));
          oRBumper.whileFalse(new KeepClimbCmd(elevatorSubsystem, shooterSub, ElevatorConstants.climbDownPos));

        //Operator Button Commands
          oBY.whileTrue(new SetElevatorSpeed(elevatorSubsystem, 0.1));//Test later
          oBY.whileFalse(new SetElevatorSpeed(elevatorSubsystem, 0));//Test later

          oBA.whileTrue(new SetElevatorSpeed(elevatorSubsystem, -0.1));//Test later
          oBA.whileFalse(new SetElevatorSpeed(elevatorSubsystem, 0));//Test later

          oBX.whileTrue(new ParallelCommandGroup(new SetElevatorPosition(elevatorSubsystem, 0), new SetPivotAngle(shooterSub, ShooterMechConstants.restPos)));
          oBX.whileFalse(new ParallelCommandGroup(new SetElevatorPosition(elevatorSubsystem, 0), new RestShooter(shooterSub, intakeSubsystem)));

        //Operator POV Commands
        oPadUp.whileTrue(new SetShooterPivotSpeed(shooterSub, 0.1));
        oPadUp.whileFalse(new SetShooterPivotSpeed(shooterSub, 0));

        oPadDown.whileTrue(new SetShooterPivotSpeed(shooterSub, -0.1));
        oPadDown.whileFalse(new SetShooterPivotSpeed(shooterSub, 0));

        oPadRight.whileTrue(new TestIntakePivot(intakeSubsystem, 0.1));
        oPadRight.whileFalse(new TestIntakePivot(intakeSubsystem, 0));

        oPadLeft.whileTrue(new TestIntakePivot(intakeSubsystem, -0.1));
        oPadLeft.whileFalse(new TestIntakePivot(intakeSubsystem, 0));

    //Extra commands during the testing phase of the robot
      // dRTrigger.whileTrue(new IntakeANDTransferCmd(intakeSubsystem, shooterSub));
      // dRTrigger.whileFalse(new StopTransferANDIntake(intakeSubsystem, shooterSub));
      // dLTrigger.whileTrue(new SetAngleAndFlywheelSpeeds(shooterSub, intakeSubsystem, ShooterMechConstants.restPos, ShooterMechConstants.flywheelShootSpeed, ShooterMechConstants.indexShootSpeed));
      // dLTrigger.whileFalse(new SetAngleAndFlywheelSpeeds(shooterSub, intakeSubsystem, ShooterMechConstants.restPos, 0, 0));
      // dLTrigger.whileTrue(new AutoShooter(shooterSub, visionSub));
      // dLTrigger.whileFalse(new RestShooter(shooterSub));
      // dLTrigger.whileTrue(new AutoShooterWithAlign(driveSubsystem, visionSub, shooterSub, intakeSubsystem, driveController, fieldRelative));
      // dLTrigger.whileFalse(new RestShooter(shooterSub));
      // dLBumper.whileTrue(new OuttakeANDTransferCmd(intakeSubsystem, shooterSub, IntakeConstants.outtakeSpeed, IntakeConstants.reverseTransferSpeed, -ShooterMechConstants.indexTransferSpeed, -ShooterMechConstants.flywheelShootSpeed));
      // dLBumper.whileFalse(new StopTransferANDIntake(intakeSubsystem, shooterSub));
      // dRBumper.whileTrue(new AmpShoot(shooterSub));
      // kA.whileTrue(new AutoAlignNote(driveSubsystem, visionSub, driveController, fieldRelative));
      // kA.whileFalse(new DriveJoystick(driveSubsystem, driveController, fieldRelative));
      // kB.whileTrue(new AutoAlignTag(driveSubsystem, visionSub, driveController, fieldRelative));
      // kB.whileFalse(new DriveJoystick(driveSubsystem, driveController, fieldRelative));
      // kB.whileTrue(new SetServoSpeed(elevatorSubsystem, -0.01));
      // kB.whileFalse(new SetServoSpeed(elevatorSubsystem, 0));
      
      // dPadDown.whileTrue(new SetElevatorSpeed(elevatorSubsystem, -0.3));
      // dPadDown.whileFalse(new SetElevatorSpeed(elevatorSubsystem, 0));
      // dPadUp.whileTrue(new SetElevatorSpeed(elevatorSubsystem, 0.3));
      // dPadUp.whileFalse(new SetElevatorSpeed(elevatorSubsystem, 0));
      // dPadUp.whileTrue(new SetElevatorPosition(elevatorSubsystem, 1.99));
      // dPadDown.whileTrue(new SetElevatorPosition(elevatorSubsystem, 0.05));
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

    //Getting the driver's trigger input
    dRTrigger = driveController.rightTrigger();
    dLTrigger = driveController.leftTrigger();

    //Getting the driver's bumper input
    dLBumper = driveController.leftBumper();
    dRBumper = driveController.rightBumper();

    //Getting the driver's button input
    dBA = driveController.a();
    dBB = driveController.b();
    dBY = driveController.y();
    dBX = driveController.x();

    //Getting the driver's pov input (DPAD)
    dPadUp = driveController.povUp();
    dPadDown = driveController.povDown();

    //Getting the operator's trigger input
    oLTrigger = operatorController.leftTrigger();
    oRTrigger = operatorController.rightTrigger();

    //Getting the operator's bumper input
    oLBumper = operatorController.leftBumper();
    oRBumper = operatorController.rightBumper();

    //Getting the operator's button input
    oBA = operatorController.a();
    oBY = operatorController.y();
    oBX = operatorController.x();

    //Getting the operator's pov input (DPAD)
    oPadUp = operatorController.povUp();
    oPadDown = operatorController.povDown();
    oPadLeft = operatorController.povLeft();
    oPadRight = operatorController.povRight();
  }

  public void makeAuto(){
    //Registering the commands to the pathplanner software
    NamedCommands.registerCommand("RestShooter", new RestShooter(shooterSub, intakeSubsystem));
    NamedCommands.registerCommand("RunIntake", new IntakeANDTransferCmd(intakeSubsystem, shooterSub));
    NamedCommands.registerCommand("AutoShooter", new AutonomousAutoShooterWAlign(driveSubsystem, visionSub, shooterSub, intakeSubsystem, driveController, true));
    NamedCommands.registerCommand("RestIntake", new RestIntakeCmd(intakeSubsystem, driveController));

    //Getting the first path for each auto side
    PathPlannerPath sourceZonePath = PathPlannerPath.fromPathFile("SourceZonePath1");
    PathPlannerPath sourceZonePathNoteRuin = PathPlannerPath.fromPathFile("SourceZonePathNoteRuin1");
    PathPlannerPath middlePath = PathPlannerPath.fromPathFile("MiddlePath1");

    //Making the red source zone auto 
    Command redSourceZoneAuto = new SequentialCommandGroup(new InstantCommand(() -> 
    driveSubsystem.resetOdometry(sourceZonePath.flipPath().getPreviewStartingHolonomicPose())), 
    AutoBuilder.buildAuto("SourceZoneAuto"));

    //Making the blue source zone auto
    Command blueSourceZoneAuto = new SequentialCommandGroup(new InstantCommand(() -> 
    driveSubsystem.resetOdometry(sourceZonePath.getPreviewStartingHolonomicPose())), 
    AutoBuilder.buildAuto("SourceZoneAuto"));

    Command blueSourceZoneAutoNoteRuin = new SequentialCommandGroup(new InstantCommand(() -> 
    driveSubsystem.resetOdometry(sourceZonePathNoteRuin.getPreviewStartingHolonomicPose())), 
    AutoBuilder.buildAuto("NoteRuinAutoSource"));

    //Making the red middle auto
    Command redMiddleAuto = new SequentialCommandGroup(new 
    InstantCommand(() -> 
    driveSubsystem.resetOdometry(middlePath.flipPath().getPreviewStartingHolonomicPose())), 
    AutoBuilder.buildAuto("MiddleAuto"));

    Command redMiddleAutoTest = new SequentialCommandGroup(new InstantCommand(() -> 
    driveSubsystem.resetOdometry(middlePath.flipPath().getPreviewStartingHolonomicPose())), 
    AutoBuilder.buildAuto("MiddleAutoTest"));

    //Making the blue middle auto
    Command blueMiddleAuto = new SequentialCommandGroup(new InstantCommand(() -> 
    driveSubsystem.resetOdometry(middlePath.getPreviewStartingHolonomicPose())), 
    AutoBuilder.buildAuto("MiddleAuto"));

    Command blueMiddleAutoTest = new SequentialCommandGroup(new InstantCommand(() -> 
    driveSubsystem.resetOdometry(middlePath.getPreviewStartingHolonomicPose())), 
    AutoBuilder.buildAuto("MiddleAutoTest"));

    //Testing auto
    Command testPath = new SequentialCommandGroup(new InstantCommand(() -> driveSubsystem.resetOdometry(sourceZonePath.flipPath().getPreviewStartingHolonomicPose())), AutoBuilder.followPath(sourceZonePath));

    //Making a multiple choice selection for auto to choose from 
    chooser.setDefaultOption("Nothing", null);
    chooser.addOption("Red Source Zone Auto", redSourceZoneAuto);
    chooser.addOption("Red Middle Auto Test", redMiddleAutoTest);
    chooser.addOption("Red Middle Auto", redMiddleAuto);
    chooser.addOption("Blue Middle Auto Test", blueMiddleAutoTest);
    chooser.addOption("Blue Middle Auto", blueMiddleAuto);
    chooser.addOption("Blue Source Zone Path", blueSourceZoneAuto);
    chooser.addOption("Blue Source Zone Note Ruin Auto", blueSourceZoneAutoNoteRuin);
    chooser.addOption("Testing Path Follow", testPath);

    //Sending the chooser to the SmartDashboard
    SmartDashboard.putData(chooser);

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    //Will run the autonomous that is selected in the SmartDashboard
    return chooser.getSelected();
  }
}
