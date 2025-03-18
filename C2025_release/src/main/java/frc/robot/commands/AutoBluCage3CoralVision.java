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
public class AutoBluCage3CoralVision extends SequentialCommandGroup {
  /** Creates a new AutoRedCage3CoralVision. */
  public AutoBluCage3CoralVision() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
       // Set initial IMU to 180; robot facing the team
       new InstantCommand(() -> RobotContainer.driveSubsystem.initialSetYawAndOdometryYaw(180)),

       new DeferredCommand(
           () -> new SetOdometryToVisionPose()
               .andThen(new PrintCommand(
                   "---A From: " + RobotContainer.driveSubsystem.getInitialVisionAidedOdometryPose( new Pose2d(7.219, 6.130, Rotation2d.k180deg)) +
                       " To: " + RobotPoseConstants.visionRobotPoses.get("RobotBluReef3Right").toString())),
           Set.of()),
      RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
            "Blu-BargetoReef11", false, false),
       new CoralPlaceOnFour(),
       new ElevatorAllTheWayDown()
           .alongWith(RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
               "Blu-Reef11ToCoralTop", false, false)),
       new TeleopCoralIntakeSequence(),
       RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
            "Blu-CoralTopToReef10", false, false),
       new CoralPlaceOnFour(),
       new ElevatorAllTheWayDown()
           .alongWith(RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
               "Blu-Reef10ToCoralTop", false, false)),
      new TeleopCoralIntakeSequence(),
       RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
            "Blu-CoralTopToReef9", false, false),
       new CoralPlaceOnFour(),
       new ElevatorAllTheWayDown()
    );
  }
}
