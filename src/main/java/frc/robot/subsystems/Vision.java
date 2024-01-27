// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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


  public double getAprilTagXOffset(){
    aprilXOffset = aprilTagLimelight.getEntry("tx").getDouble(0.0);
    return aprilXOffset;
  }

  public double getAprilTagYOffset(){
    aprilYOffset = aprilTagLimelight.getEntry("ty").getDouble(0.0);
    return aprilYOffset;
  }

  public double getAIObjectXOffset(){
    aiXOffset = aiObjectLimelight.getEntry("tx").getDouble(0.0);
    return aiXOffset;
  }

  public double getAIObjectYOffset(){
    aiYOffset = aiObjectLimelight.getEntry("ty").getDouble(0.0);
    return aiYOffset;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
