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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRed3CoralVision extends SequentialCommandGroup {
  /** Creates a new AutoBlu2CoralVision. */
  public AutoRed3CoralVision() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //TODO: Values in this command need to be changed as needed 
        // Set initial IMU to 180; robot facing the team
        new InstantCommand(() -> RobotContainer.driveSubsystem.initialSetYawAndOdometryYaw(180)),

        new DeferredCommand(
            () -> new SetOdometryToVisionPose()
                .andThen(new PrintCommand(
                    "---A From: " + RobotContainer.llVisionSubsystem.getBestPoseAllCameras().toString() +
                        " To: " + RobotPoseConstants.visionRobotPoses.get("RobotRedReef3Right").toString())),
            Set.of()),
        new DeferredCommand(
            () -> RobotContainer.runTrajectory2PosesSlow(
                RobotContainer.driveSubsystem.getInitialVisionAidedOdometryPose( new Pose2d(7.219, 6.130, Rotation2d.k180deg)), // if vision is not available at the start, use that pose
                RobotPoseConstants.visionRobotPoses.get("RobotRedReef3Right"),
                true),
            Set.of()),
        new CoralPlaceOnFour(),
        new ElevatorAllTheWayDown()
            .alongWith(RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
                "Red-Reef11toCoralBottom", false, false)),
        new TeleopCoralIntakeSequence(),
        RobotContainer.runTrajectory2PosesSlow(
            RobotPoseConstants.visionRobotPoses.get("RobotRedStationUp"),
            RobotPoseConstants.visionRobotPoses.get("RobotRedReef2Right"),
            false),
        new CoralPlaceOnFour(),
        new ElevatorAllTheWayDown()
            .alongWith(RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
                "Red-Reef10toCoralBottom", false, false)),
        new TeleopCoralIntakeSequence(),
        RobotContainer.runTrajectory2PosesSlow(
            RobotPoseConstants.visionRobotPoses.get("RobotRedStationUp"),
            RobotPoseConstants.visionRobotPoses.get("RobotRedReef2Left"),
            false),
        new CoralPlaceOnFour(),
        new ElevatorAllTheWayDown());
  }
}
