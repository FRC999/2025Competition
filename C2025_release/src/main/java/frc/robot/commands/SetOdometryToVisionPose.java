// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionHelperConstants.RobotPoseConstants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetOdometryToVisionPose extends SequentialCommandGroup {
  /** Creates a new SetOdometryToVisionPose. */
  public SetOdometryToVisionPose() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DeferredCommand(
        () -> new ConditionalCommand(

          new InstantCommand(() -> RobotContainer.driveSubsystem.setOdometryPoseToSpecificPose(
                RobotContainer.llVisionSubsystem.getBestPoseAllCameras()
              )
            )
            .andThen(new PrintCommand("* Pose set from vision"))
          ,
          new PrintCommand("* No vision pose to reset odometry")
            .andThen(new InstantCommand(() -> RobotContainer.driveSubsystem.setOdometrySetWithVision(false)))
            ,
          RobotContainer.llVisionSubsystem::isAprilTagVisibleBySomeCamera
        )
        , Set.of())
    );
  }
}
