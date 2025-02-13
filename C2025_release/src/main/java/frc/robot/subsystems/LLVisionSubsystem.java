// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;

public class LLVisionSubsystem extends SubsystemBase {
  public static AprilTagFieldLayout fieldLayout;
  /** Creates a new LLVisionSubsystem. */
  public LLVisionSubsystem() {}

  public void initialize(){
    try {
      // Load the built-in AprilTag field layout for the current game
      fieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
  } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTag field layout: " + e.getMessage(), true);
      fieldLayout = null;
  }
}

  }

  public Pose2d getRobotAprilTagPose() {
    return;
  }

  public boolean isAprilTagVisible() {
    return false;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (fieldLayout != null) {
      // Example: Get the pose of AprilTag with ID 1
      int tagId = 1;
      Pose3d tagPose = fieldLayout.getTagPose(tagId).orElse(null);

      if (tagPose != null) {
          System.out.println("AprilTag " + tagId + " Pose: " + tagPose);
      } else {
          System.out.println("AprilTag " + tagId + " not found in the field layout.");
      }
  }
}


