// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueOneCoral extends SequentialCommandGroup {
  /** Creates a new AutoBlueOneCoral. */
  public AutoBlueOneCoral() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( ()-> RobotContainer.driveSubsystem.setOdometryToIdealPoseFromTrajectory("Blu-BargeToCenterReef"))
        // just in case - wait 0.1s for the pose to take place
        .andThen(new WaitCommand(0.1))
        .andThen(RobotContainer.runTrajectoryPathPlannerWithForceResetOfStartingPose("Blu-BargeToCenterReef", true,false))
        .andThen(new WaitCommand(0.1))
        .andThen(new TeleopMoveToL4RotateArm()) 
        .andThen(new AutoEjectCoralBringArmToCruiseElevatorDown())
    );
  }
}
