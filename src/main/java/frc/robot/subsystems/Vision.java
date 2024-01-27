// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightNoteConstants;
import frc.robot.Constants.LimelightSpeakerConstants;

public class Vision extends SubsystemBase {
  //Setting up limelights
  private NetworkTable aprilTagLimelight = NetworkTableInstance.getDefault().getTable("limelight-apriltag");
  private NetworkTable aiObjectLimelight = NetworkTableInstance.getDefault().getTable("limelight-aiobject");

  //Getting X and Y offsets of both limelights
  private double aprilXOffset;
  private double aprilYOffset;

  private double aiXOffset;
  private double aiYOffset;
  /** Creates a new Vision. */
  public Vision() {

    aiXOffset = aiObjectLimelight.getEntry("tx").getDouble(0.0);
    aiYOffset = aiObjectLimelight.getEntry("ty").getDouble(0.0);
  }

  //Gets the offsets between the center of the apriltag to the crosshair's x axis (The plus sign when looking at the limelight video)
  public double getAprilTagXOffset(){
    aprilXOffset = aprilTagLimelight.getEntry("tx").getDouble(0.0);
    return aprilXOffset;
  }

  //Gets the offsets between the center of the apriltag to the crosshair's y axis (The plus sign when looking at the limelight video)
  public double getAprilTagYOffset(){
    aprilYOffset = aprilTagLimelight.getEntry("ty").getDouble(0.0);
    return aprilYOffset;
  }

    //Gets the offsets between the center of the note to the crosshair's x axis (The plus sign when looking at the limelight video)
  public double getAIObjectXOffset(){
    aiXOffset = aiObjectLimelight.getEntry("tx").getDouble(0.0);
    return aiXOffset;
  }

  //Gets the offsets between the center of the note to the crosshair's y axis (The plus sign when looking at the limelight video)
  public double getAIObjectYOffset(){
    aiYOffset = aiObjectLimelight.getEntry("ty").getDouble(0.0);
    return aiYOffset;
  }

  //Gets distance from the note to the robot
  public double getNoteDistanceFromRobot(){
    double angleToGoalDegrees = LimelightNoteConstants.limelightMountAngleDegrees + getAIObjectYOffset();
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double distanceFromLimelightToGoalInches = (LimelightNoteConstants.goalHeightInches - LimelightNoteConstants.limelightLensHeightInches);
    return distanceFromLimelightToGoalInches;
  }
    //Gets distance from the speaker to the robot
  public double getSpeakerDistanceFromRobot(){
    double angleToGoalDegrees = LimelightSpeakerConstants.limelightMountAngleDegrees + getAIObjectYOffset();
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double distanceFromLimelightToGoalInches = (LimelightSpeakerConstants.goalHeightInches - LimelightSpeakerConstants.limelightLensHeightInches);
    return distanceFromLimelightToGoalInches;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
