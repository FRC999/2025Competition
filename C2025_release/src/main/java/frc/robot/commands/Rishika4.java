// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.RobotContainer;
import java.util.Set;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Rishika4 extends SequentialCommandGroup {
  /** Creates a new AutoRed2CoralVisionRishika. */
  public Rishika4() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Set initial IMU to 180; robot facing the team
      new InstantCommand(() -> RobotContainer.driveSubsystem.initialSetYawAndOdometryYaw(60)),

      new DeferredCommand(
          () -> new SetOdometryToVisionPose()
              .andThen(new PrintCommand(
                  "---A From: " + RobotContainer.driveSubsystem.getInitialVisionAidedOdometryPose( new Pose2d(2.0, 3.8, Rotation2d.fromDegrees(60.0))) +
                      " To: " + RobotPoseConstants.visionRobotPoses.get("RobotBluReef6Left").toString())),
          Set.of()),
      new DeferredCommand(
          () -> RobotContainer.runTrajectory2PosesSlow(
              RobotContainer.driveSubsystem.getInitialVisionAidedOdometryPose( new Pose2d(2.0, 3.8, Rotation2d.fromDegrees(60.0))), // if vision is not available at the start, use that pose
              RobotPoseConstants.visionRobotPoses.get("RobotBluReef6Left"),
              true),
          Set.of()),
          new WaitCommand(3),
          RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
              "Blu-Reef4ToCoralBottom", true, false)
      //new CoralPlaceOnFour()
      // new ElevatorAllTheWayDown()
      //     .alongWith(RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
      //         "Red-Reef11toCoralBottom", false, true))
      //new TeleopCoralIntakeSequence()
      //RobotContainer.runTrajectory2PosesSlow(
          //RobotPoseConstants.visionRobotPoses.get("RobotRedStationDown"),
          //RobotPoseConstants.visionRobotPoses.get("RobotRedReef6Right"),
          //false)
      // new CoralPlaceOnFour(),
      // new ElevatorAllTheWayDown()
    );
  }
}
