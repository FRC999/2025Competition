// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotContainer;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.LLVisionConstants.LLCamera;
import frc.robot.Constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.lib.VisionHelpers;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LLVisionSubsystem extends SubsystemBase {
  public static AprilTagFieldLayout fieldLayout;

  private HashMap<Double,Pose2d> posesMap = new HashMap<>();
  private double bestPoseKey;
  /** Creates a new LLVisionSubsystem. */
  public LLVisionSubsystem() {
    if(!EnabledSubsystems.ll){
      return;
    }

    try {
      // Load the built-in AprilTag field layout for the current game
      fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); //TODO: May need to change field type before the competition
    } catch (Exception e) {
      DriverStation.reportError("Failed to load AprilTag field layout: " + e.getMessage(), true);
      fieldLayout = null;
    }

    VisionHelpers.createHashMapOfTags(); // load Pose2d of all apriltags
    VisionHelpers.addRobotPosesForCoralPlacement(); // load Pose2d of robot to place/pickup elements
    VisionHelpers.mapTagIDToTagKey(); // create a map of Tag ID to the Tag name/alias

  }

  public Pose2d getRobotAprilTagPose() {
    return null;
  }

  public Pose2d getKnownPose(String poseName) {
    //System.out.println(RobotPoseConstants.visionRobotPoses.keySet());
    if(RobotPoseConstants.visionRobotPoses.containsKey(poseName)){
      return RobotPoseConstants.visionRobotPoses.get(poseName);
    } else {
      return null; 
    }
  }

  public boolean isAprilTagVisibleBySomeCamera() {
    boolean result = false; 
    for (LLCamera llcamera : LLCamera.values()) {
      result = result || isAprilTagVisible(llcamera.getCameraName());
      if(result){
        return result; 
      }
    }
    return result; 
  }

  public boolean isAprilTagVisible(String cameraName) {
    return LimelightHelpers.getTV(cameraName); 
  }

  private void addCurrentLLPoseToHashMap(PoseEstimate pestimate) {
    posesMap.put(pestimate.timestampSeconds,pestimate.pose);
    if (pestimate.timestampSeconds>bestPoseKey) { // we got new winner - later timestamp
      if(posesMap.size()>0) {
        posesMap.remove(bestPoseKey); // so hashmap size will not constantly increase
      }
      bestPoseKey=pestimate.timestampSeconds;
    }
  }

  public Pose2d getBestPose2d() {
    if (posesMap.isEmpty()) { return null;}
    if (!posesMap.containsKey(bestPoseKey)) { return null;}
    return posesMap.get(bestPoseKey);
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


    // Wipe out poses from previouos camera loop
    if(!isAprilTagVisibleBySomeCamera()) {
      posesMap.clear();
    }

    bestPoseKey= 0;

    for (LLCamera llcamera : LLCamera.values()) {
      String cn = llcamera.getCameraName();

      double yaw = RobotContainer.driveSubsystem.getYaw();
      double yawrate = RobotContainer.driveSubsystem.getTurnRate();

      // Update LLs with current YAW, so they can return correct position for Megatag2
      LimelightHelpers.SetRobotOrientation(cn, yaw, yawrate, 0, 0, 0, 0);

      // Select the best coordinates
      if (LimelightHelpers.getTV(cn)) {
        addCurrentLLPoseToHashMap(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cn));
      }

    }

  }
}


