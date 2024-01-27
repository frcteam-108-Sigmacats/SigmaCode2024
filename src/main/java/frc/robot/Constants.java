// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  //Class for limelight using AI Object Detection
  public static class LimelightNoteConstants{
    //How many degrees back is your limelight rotated from perfectly vertical
    public static final double limelightMountAngleDegrees = 0; //Change later when it is on the robot

    //Distance between the center of the limelight's lens to the floor
    public static final double limelightLensHeightInches = 0;

    //Distance form the target to the floor
    public static final double goalHeightInches = 0; 
  }

  //Class for limelight using AprilTag
  public static class LimelightSpeakerConstants{
    //How many degrees back is your limelight rotated from perfectly vertical
    public static final double limelightMountAngleDegrees = 0; //Change later when it is on the robot

    //Distance between the center of the limelight's lens to the floor
    public static final double limelightLensHeightInches = 0;

    //Distance form the target to the floor
    public static final double goalHeightInches = 0; 
  }
}
