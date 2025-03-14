// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBluReverse2CoralVision extends SequentialCommandGroup {
  /** Creates a new AutoBluReverse2CoralVision. */
  public AutoBluReverse2CoralVision() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( new InstantCommand(() -> RobotContainer.driveSubsystem.initialSetYawAndOdometryYaw(0)),

        new DeferredCommand(
            () -> new SetOdometryToVisionPose()
                .andThen(new PrintCommand(
                    "---A From: " + RobotContainer.llVisionSubsystem.getBestPoseAllCameras().toString() +
                        " To: " + RobotPoseConstants.visionRobotPoses.get("RobotBluReef5Right").toString())),
            Set.of()),
        RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
                "Blu-BargeToReef2", true, false),
        new CoralPlaceOnFour(),
        new ElevatorAllTheWayDown()
            .alongWith(RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
                "Blu-Reef4ToCoralBottom", false, false)),
        new TeleopCoralIntakeSequence(),
        RobotContainer.runTrajectory2PosesSlow(
            RobotPoseConstants.visionRobotPoses.get("RobotBluStationUp"),
            RobotPoseConstants.visionRobotPoses.get("RobotBlueReef6Right"),
            false),
        new CoralPlaceOnFour(),
        new ElevatorAllTheWayDown());
  }
}
