// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.RobotContainer;
import frc.robot.lib.VisionHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopPanLeftReefVision extends SequentialCommandGroup {
  /** Creates a new TeleopPanLeftReefVision. */
  public TeleopPanLeftReefVision() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new DeferredCommand(
        () -> 
        RobotContainer.runTrajectory2PosesSlow(
          RobotContainer.llVisionSubsystem.getBestPoseAllCameras(),
             RobotPoseConstants.visionRobotPoses.get(
               VisionHelpers.getLeftReefName(
                  RobotPoseConstants.reefTagPoses.get(
                    VisionHelpers.getClosestReefTagToRobot(RobotContainer.llVisionSubsystem.getBestPoseAllCameras())))),
                true)
      ,Set.of())
    );

  }
}
