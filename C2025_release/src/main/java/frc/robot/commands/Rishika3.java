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
public class Rishika3 extends SequentialCommandGroup {
  /** Creates a new AutoRed2CoralVisionRishika. */
  public Rishika3() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Set initial IMU to 180; robot facing the team
      new InstantCommand(() -> RobotContainer.driveSubsystem.initialSetYawAndOdometryYaw(0)),

      new DeferredCommand(
          () -> new SetOdometryToVisionPose()
              .andThen(new PrintCommand(
                  "---A From: " + RobotContainer.driveSubsystem.getInitialVisionAidedOdometryPose( new Pose2d(10.331, 1.920, Rotation2d.kZero)) +
                      " To: " + RobotPoseConstants.visionRobotPoses.get("RobotRedReef5Right").toString())),
          Set.of()),
      new DeferredCommand(
          () -> RobotContainer.runTrajectory2PosesSlow(
              RobotContainer.driveSubsystem.getInitialVisionAidedOdometryPose( new Pose2d(10.331, 1.920, Rotation2d.kZero)), // if vision is not available at the start, use that pose
              RobotPoseConstants.visionRobotPoses.get("RobotRedReef5Right"),
              true),
          Set.of()),
      new CoralPlaceOnFour(),
      new ElevatorAllTheWayDown()
          .alongWith(RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose(
              "Red-Reef11toCoralBottom", false, false)),
      new TeleopCoralIntakeSequence(),
      RobotContainer.runTrajectory2PosesSlow(
          RobotPoseConstants.visionRobotPoses.get("RobotRedStationDown"),
          RobotPoseConstants.visionRobotPoses.get("RobotRedReef6Right"),
          false),
      new CoralPlaceOnFour(),
      new ElevatorAllTheWayDown()
    );
  }
}
